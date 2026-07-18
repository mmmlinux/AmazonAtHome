"""
HectorDriver — ROS 2 node bridging nav_server to the physical Hector robot
over a persistent WebSocket connection.

Drop-in replacement for robot_sim / real_robot_driver.  Implements the same two
interfaces that nav_server depends on:

  Action server   {ns}/move_to_waypoint  — accepts one-hop navigation goals
  Topic publisher {ns}/robot_status      — publishes position / battery at 4 Hz

Communication with Hector
-------------------------
This node acts as a WebSocket *server* on ws_host:ws_port.
Hector connects once on boot and holds the connection open.
All messages are newline-delimited UTF-8 JSON.

WM → Hector messages
  {"type": "move",  "id": "<hex>", "target_waypoint": "<str>",
   "distance_m": <float>, "heading_deg": <float>, "turn_deg": <float>}
  {"type": "abort"}

Hector → WM messages
  {"type": "status",    "battery_pct": <float>, "progress": <float>,
   "moving": <bool>,    "left_enc": <int>, "right_enc": <int>,
   "lift_up": <bool>,   "payload_present": <bool>}
  {"type": "move_done", "id": "<hex>", "success": <bool>, "message": "<str>"}
  {"type": "fault",     "code": "<str>", "data": <int>}

Parameters (in addition to real_robot_driver parameters)
----------
  ws_host   WebSocket bind address (default 0.0.0.0)
  ws_port   WebSocket server port  (default 8765)

How to use
----------
  1. Add a robot entry in robots.yaml with robot_type: hector and ws_port set.
  2. Launch the warehouse manager normally — hector_driver starts automatically.
  3. On the Pi Zero W run:
       python hector_agent.py --ws ws://<WM_IP>:<ws_port> --port /dev/ttyS0
"""

import asyncio
import json
import threading
import time
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor

try:
    import websockets
    import websockets.exceptions
except ImportError:
    raise ImportError(
        'websockets library not found — install it with: pip install websockets'
    )

from logistics_robot_sim.real_robot_driver import RealRobotDriver


class HectorDriver(RealRobotDriver):
    """
    Concrete implementation of RealRobotDriver for the Hector robot.

    All hardware I/O goes over a single persistent WebSocket connection.
    The asyncio event loop for the WebSocket server runs in a daemon thread so
    it does not interfere with the ROS MultiThreadedExecutor.
    """

    def __init__(self):
        super().__init__()

        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)

        ws_host = self.get_parameter('ws_host').get_parameter_value().string_value
        ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value

        # ── Hector WebSocket connection ───────────────────────────────────────
        self._ws_client = None          # set when Hector connects
        self._ws_lock   = threading.Lock()

        # ── Pending move tracking ─────────────────────────────────────────────
        self._active_move_id: str | None = None
        self._move_done    = threading.Event()
        self._move_success = False

        # ── Latest telemetry from Hector ──────────────────────────────────────
        self._telem_lock     = threading.Lock()
        self._telem_battery  = 100.0
        self._telem_progress = 0.0
        self._telem_moving   = False

        # ── Asyncio event loop in a dedicated daemon thread ───────────────────
        self._loop = asyncio.new_event_loop()
        threading.Thread(
            target=self._run_ws_server,
            args=(ws_host, ws_port),
            daemon=True,
            name='hector-ws-server',
        ).start()

        self.get_logger().info(
            f'HectorDriver: WebSocket server listening on ws://{ws_host}:{ws_port}'
        )

    # ── WebSocket server ──────────────────────────────────────────────────────

    def _run_ws_server(self, host: str, port: int) -> None:
        """Entry point for the WebSocket daemon thread."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ws_server_main(host, port))

    async def _ws_server_main(self, host: str, port: int) -> None:
        async with websockets.serve(self._handle_client, host, port):
            await asyncio.Future()   # run forever

    async def _handle_client(self, ws) -> None:
        """Handle one Hector connection for its lifetime."""
        addr = getattr(ws, 'remote_address', 'unknown')
        self.get_logger().info(f'HectorDriver: Hector connected from {addr}')

        # Accept only one client at a time; close the previous one if it lingers
        with self._ws_lock:
            old = self._ws_client
            self._ws_client = ws

        if old is not None:
            try:
                await old.close()
            except Exception:
                pass

        try:
            async for raw in ws:
                try:
                    await self._on_hector_message(json.loads(raw))
                except (json.JSONDecodeError, KeyError) as exc:
                    self.get_logger().warning(
                        f'HectorDriver: malformed message from Hector: {exc}'
                    )
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            with self._ws_lock:
                if self._ws_client is ws:
                    self._ws_client = None

            # If Hector drops while a move is in progress, unblock the polling loop
            if self._active_move_id is not None and not self._move_done.is_set():
                self._move_success = False
                self._move_done.set()

            self.get_logger().warning('HectorDriver: Hector disconnected')

    async def _on_hector_message(self, msg: dict) -> None:
        # TODO: Enforce the registration handshake documented in CLAUDE.md.
        # The first message from Hector on every (re)connect must be a
        # {"type": "register", "robot_id": "<id>"} frame.  Currently this
        # driver ignores that message entirely.  Add state tracking so:
        #   - commands received before registration are silently dropped, and
        #   - a connection that never registers within N seconds is closed.
        mtype = msg.get('type')

        if mtype == 'status':
            with self._telem_lock:
                self._telem_battery  = float(msg.get('battery_pct',  self._telem_battery))
                self._telem_progress = float(msg.get('progress',     0.0))
                self._telem_moving   = bool( msg.get('moving',        False))

        elif mtype == 'move_done':
            if msg.get('id') == self._active_move_id:
                self._move_success = bool(msg.get('success', False))
                if not self._move_success:
                    self.get_logger().warning(
                        f'HectorDriver: move failed — {msg.get("message", "")}'
                    )
                self._move_done.set()

        elif mtype == 'fault':
            self.get_logger().error(
                f'HectorDriver: Hector fault  code={msg.get("code")}  '
                f'data={msg.get("data")}'
            )

        else:
            self.get_logger().debug(f'HectorDriver: unknown message type {mtype!r}')

    # ── Thread-safe WebSocket send ────────────────────────────────────────────

    async def _async_send(self, payload: dict) -> bool:
        with self._ws_lock:
            ws = self._ws_client
        if ws is None:
            return False
        try:
            await ws.send(json.dumps(payload))
            return True
        except websockets.exceptions.ConnectionClosed:
            return False

    def _send_to_hector(self, payload: dict, timeout: float = 5.0) -> bool:
        """Send a JSON message from a non-async (ROS) thread."""
        future = asyncio.run_coroutine_threadsafe(self._async_send(payload), self._loop)
        try:
            return future.result(timeout=timeout)
        except Exception as exc:
            self.get_logger().warning(f'HectorDriver: send error — {exc}')
            return False

    # ── RealRobotDriver hardware interface ────────────────────────────────────

    def _send_move_command(
        self,
        target_waypoint: str,
        distance_m: float,
        heading_deg: float,
        turn_deg: float,
    ) -> None:
        """
        Send a one-hop navigation goal to Hector.

        Blocks until a Hector client is connected (up to 30 s), then fires
        the command and returns immediately — actual completion is polled via
        _is_move_complete().
        """
        deadline = time.monotonic() + 30.0
        while True:
            with self._ws_lock:
                connected = self._ws_client is not None
            if connected:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(
                    'HectorDriver: no Hector connection after 30 s — aborting move'
                )
            time.sleep(0.25)

        self._active_move_id = uuid.uuid4().hex[:8]
        self._move_done.clear()
        self._move_success = False

        ok = self._send_to_hector({
            'type':            'move',
            'id':              self._active_move_id,
            'target_waypoint': target_waypoint,
            'distance_m':      round(distance_m, 4),
            'heading_deg':     round(heading_deg, 2),
            'turn_deg':        round(turn_deg,    2),
        })

        if not ok:
            raise RuntimeError('HectorDriver: failed to send move command to Hector')

    def _is_move_complete(self) -> bool:
        return self._move_done.is_set()

    def _get_travel_progress(self) -> float:
        with self._telem_lock:
            return self._telem_progress

    def _read_battery(self) -> float:
        with self._telem_lock:
            return self._telem_battery

    def _abort_move(self) -> None:
        self._send_to_hector({'type': 'abort'}, timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = HectorDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
