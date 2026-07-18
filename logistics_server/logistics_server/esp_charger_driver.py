"""
ESP32 charger driver — communicates over WebSocket.

The WM acts as a WebSocket CLIENT connecting to ws://{ip}:{port}.
The ESP32 is the server; it must be reachable at a fixed IP on the LAN.
The connection loop reconnects automatically if the link drops.

All messages are newline-delimited UTF-8 JSON.

WM → ESP32
----------
  {"type": "engage",    "robot_id": "<str>"}   — extend arm / close contacts
  {"type": "disengage", "robot_id": "<str>"}   — retract arm / open contacts
  {"type": "ping"}                              — keep-alive

ESP32 → WM
----------
  {"type": "status",  "robot_detected": bool, "charging": bool,
                      "current_ma": float, "voltage_v": float}
      Sent periodically (recommended 4 Hz) and whenever state changes.

  {"type": "ack",     "command": "engage"|"disengage",
                      "success": bool, "message": "<str>"}
      Sent once in response to every engage/disengage command.
      success=false means the charger could not fulfil the request
      (e.g. arm fault, no robot detected).

  {"type": "pong"}    — keep-alive response to ping
  {"type": "fault",   "code": "<str>", "data": <any>}
      Asynchronous fault notification (arm jam, power failure, etc.).

Motion model on the ESP32
--------------------------
  engage:
    1. Extend charging arm / close contacts.
    2. Detect robot presence (IR / contact sensor).
    3. Enable power output.
    4. Send ack {"success": true} once current is flowing,
       or {"success": false, "message": "no_robot_detected"} on timeout.

  disengage:
    1. Disable power output.
    2. Retract arm / open contacts.
    3. Send ack {"success": true} when arm is fully retracted.
"""

import asyncio
import json
import logging
import threading
import time

try:
    import websockets
    import websockets.exceptions
except ImportError:
    raise ImportError('websockets not found — pip install websockets')

from .charger_driver import ChargerDriver


class EspChargerDriver(ChargerDriver):
    """
    WebSocket-client driver for an ESP32-based physical charger.

    An asyncio event loop runs in a daemon thread so it does not interfere
    with the ROS MultiThreadedExecutor.  All blocking callers (engage /
    disengage) use threading.Event to wait for the ACK from the ESP32.
    """

    CONNECT_RETRY_S  = 3.0    # wait between reconnect attempts
    ACK_TIMEOUT_S    = 15.0   # max wait for engage/disengage ack
    PING_INTERVAL_S  = 15.0   # keep-alive interval

    def __init__(
        self,
        waypoint_id: str,
        ip: str,
        port: int = 80,
        logger: logging.Logger | None = None,
    ):
        super().__init__(waypoint_id)
        self._ip    = ip
        self._port  = port
        self._uri   = f'ws://{ip}:{port}'
        self._log   = logger or logging.getLogger(__name__)

        self._ws_client = None
        self._ws_lock   = threading.Lock()

        self._loop = asyncio.new_event_loop()

        # Synchronisation for engage/disengage ack
        self._ack_event   = threading.Event()
        self._ack_success = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Start the asyncio daemon thread; begin connecting to the ESP32."""
        threading.Thread(
            target=self._run_loop,
            daemon=True,
            name=f'charger-ws-{self.waypoint_id}',
        ).start()

    def disconnect(self) -> None:
        """Signal the asyncio loop to stop."""
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

    # ── Asyncio event loop ────────────────────────────────────────────────────

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._connection_loop())

    async def _connection_loop(self) -> None:
        """Keep connecting to the ESP32, reconnecting after any disconnect."""
        while True:
            try:
                async with websockets.connect(self._uri) as ws:
                    with self._ws_lock:
                        self._ws_client = ws
                    self._log.info(
                        f'Charger [{self.waypoint_id}]: connected to {self._uri}'
                    )
                    ping_task = asyncio.create_task(self._ping_loop(ws))
                    try:
                        await self._receive_loop(ws)
                    finally:
                        ping_task.cancel()
            except (OSError, websockets.exceptions.WebSocketException) as exc:
                self._log.warning(
                    f'Charger [{self.waypoint_id}]: connection error — {exc}'
                )
            finally:
                with self._ws_lock:
                    self._ws_client = None
                # Unblock any waiting engage/disengage call on disconnect
                if not self._ack_event.is_set():
                    self._ack_success = False
                    self._ack_event.set()

            await asyncio.sleep(self.CONNECT_RETRY_S)

    async def _receive_loop(self, ws) -> None:
        async for raw in ws:
            try:
                await self._on_message(json.loads(raw))
            except (json.JSONDecodeError, KeyError) as exc:
                self._log.warning(
                    f'Charger [{self.waypoint_id}]: malformed message — {exc}'
                )

    async def _ping_loop(self, ws) -> None:
        while True:
            await asyncio.sleep(self.PING_INTERVAL_S)
            try:
                await ws.send(json.dumps({'type': 'ping'}))
            except websockets.exceptions.ConnectionClosed:
                break

    async def _on_message(self, msg: dict) -> None:
        mtype = msg.get('type')
        if mtype == 'status':
            with self._lock:
                self._robot_detected = bool(msg.get('robot_detected', False))
                self._charging       = bool(msg.get('charging',       False))
                self._current_ma     = float(msg.get('current_ma',    0.0))
                self._voltage_v      = float(msg.get('voltage_v',     0.0))
        elif mtype == 'ack':
            self._ack_success = bool(msg.get('success', False))
            if not self._ack_success:
                self._log.warning(
                    f'Charger [{self.waypoint_id}]: command failed — '
                    f'{msg.get("message", "")}'
                )
            self._ack_event.set()
        elif mtype == 'pong':
            pass
        elif mtype == 'fault':
            self._log.error(
                f'Charger [{self.waypoint_id}]: fault  '
                f'code={msg.get("code")}  data={msg.get("data")}'
            )
        else:
            self._log.debug(
                f'Charger [{self.waypoint_id}]: unknown message type {mtype!r}'
            )

    # ── Thread-safe send ──────────────────────────────────────────────────────

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

    def _send_command(self, payload: dict, timeout: float = 5.0) -> bool:
        """Send a JSON message from a synchronous (ROS) thread."""
        future = asyncio.run_coroutine_threadsafe(
            self._async_send(payload), self._loop
        )
        try:
            return future.result(timeout=timeout)
        except Exception as exc:
            self._log.warning(f'Charger [{self.waypoint_id}]: send error — {exc}')
            return False

    # ── ChargerDriver interface ───────────────────────────────────────────────

    def engage(self, robot_id: str) -> bool:
        """Send engage command and wait for ACK (up to ACK_TIMEOUT_S)."""
        self._ack_event.clear()
        self._ack_success = False

        ok = self._send_command({'type': 'engage', 'robot_id': robot_id})
        if not ok:
            self._log.warning(
                f'Charger [{self.waypoint_id}]: no connection — engage failed'
            )
            return False

        fired = self._ack_event.wait(timeout=self.ACK_TIMEOUT_S)
        if not fired:
            self._log.warning(
                f'Charger [{self.waypoint_id}]: engage ack timed out '
                f'after {self.ACK_TIMEOUT_S:.0f} s'
            )
            return False

        with self._lock:
            self._engaged = self._ack_success
        return self._ack_success

    def disengage(self, robot_id: str) -> bool:
        """Send disengage command and wait for ACK (up to ACK_TIMEOUT_S)."""
        self._ack_event.clear()
        self._ack_success = False

        ok = self._send_command({'type': 'disengage', 'robot_id': robot_id})
        if not ok:
            self._log.warning(
                f'Charger [{self.waypoint_id}]: no connection — disengage failed'
            )
            return False

        fired = self._ack_event.wait(timeout=self.ACK_TIMEOUT_S)
        if not fired:
            self._log.warning(
                f'Charger [{self.waypoint_id}]: disengage ack timed out '
                f'after {self.ACK_TIMEOUT_S:.0f} s'
            )
            return False

        if self._ack_success:
            with self._lock:
                self._engaged        = False
                self._robot_detected = False
                self._charging       = False
        return self._ack_success
