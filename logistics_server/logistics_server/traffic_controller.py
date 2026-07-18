"""
Traffic Controller — waypoint reservation, aisle locking, and deadlock resolution.

Prevents two robots from occupying the same waypoint simultaneously and ensures
at most one robot is inside each aisle at a time.

Deadlock detection
------------------
The TC tracks each robot's pending (blocked) waypoint request.  When robot A
is waiting for a waypoint held by B, and B is simultaneously waiting for a
waypoint held by A, that is a circular wait.  The TC resolves it by publishing
a yield signal to the appropriate robot:

  - Aisle-exit vs aisle-entry deadlock: the entering robot always yields,
    regardless of priority (physically, the robot already inside must exit
    before a new one can enter).
  - All other cases: the lower-priority robot yields.  Equal priority → the
    robot with the lexicographically lower ID yields (deterministic tie-break).

The yielded robot backs up one hop to a free waypoint, releasing its current
position so the other robot can proceed.

Services
--------
  traffic/acquire   AcquireWaypoint   request permission to step onto a waypoint
  traffic/release   ReleaseWaypoint   release a waypoint after leaving it
"""

import re
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty

from logistics_interfaces.srv import AcquireWaypoint, ReleaseWaypoint


# ── Aisle detection ───────────────────────────────────────────────────────────

_AISLE_RE   = re.compile(r'^(?:aisle|shelf)_([A-Ja-j])')
_QA_RE      = re.compile(r'^qa_([WwEe])')
_SPINE_W_RE = re.compile(r'^spine_W(\d+)$')   # spine_W1..5 → aisles A-E
_SPINE_E_RE = re.compile(r'^spine_E(\d+)$')   # spine_E1..5 → aisles F-J
_SPINE_QA_RE = re.compile(r'^spine_([WwEe])qa$')


def _aisle_key(waypoint: str) -> str | None:
    """Return the aisle lock key for this waypoint, or None if not in an aisle."""
    m = _AISLE_RE.match(waypoint)
    if m:
        return f'aisle_{m.group(1).upper()}'
    m = _QA_RE.match(waypoint)
    if m:
        return f'qa_{m.group(1).upper()}'
    return None


def _junction_aisle_key(waypoint: str) -> str | None:
    """
    Return the aisle key that this *junction* waypoint leads into, or None.

    spine_W1 → aisle_A, spine_W2 → aisle_B, …, spine_W5 → aisle_E
    spine_E1 → aisle_F, spine_E2 → aisle_G, …, spine_E5 → aisle_J
    spine_Wqa / spine_Eqa → qa_W / qa_E

    This is derived purely from the naming convention, so no map loading is needed.
    """
    m = _SPINE_W_RE.match(waypoint)
    if m:
        n = int(m.group(1))
        if 1 <= n <= 5:
            return f'aisle_{chr(ord("A") + n - 1)}'
    m = _SPINE_E_RE.match(waypoint)
    if m:
        n = int(m.group(1))
        if 1 <= n <= 5:
            return f'aisle_{chr(ord("A") + n - 1 + 5)}'
    m = _SPINE_QA_RE.match(waypoint)
    if m:
        return f'qa_{m.group(1).upper()}'
    return None


class TrafficControllerNode(Node):

    def __init__(self):
        super().__init__('traffic_controller')

        self._lock = threading.Lock()

        # waypoint → (robot_id, priority)
        self._waypoint_owners: dict[str, tuple[str, int]] = {}

        # aisle_key → robot_id  (e.g. 'aisle_A' → 'robot_1')
        self._aisle_holders: dict[str, str] = {}

        # Pending blocked requests: robot_id → (wanted_waypoint, blocking_robot_id)
        self._pending: dict[str, tuple[str, str]] = {}

        # Tracks the last time each robot was yielded and what it wanted.
        # Used to detect robots that can't retreat (e.g. at a dead-end waypoint):
        # if the same robot re-deadlocks for the same waypoint within the memory
        # window, it couldn't retreat — yield the other robot instead.
        # TODO: Periodically prune _recent_yields entries older than
        # _YIELD_MEMORY_S so the dict doesn't grow without bound over long runs
        # (e.g. schedule a cleanup timer every 60 s).
        self._recent_yields: dict[str, tuple[str, float]] = {}
        self._YIELD_MEMORY_S = 15.0

        # Per-robot yield publishers, created lazily.
        self._yield_pubs: dict[str, rclpy.publisher.Publisher] = {}

        # TODO: Add a stale-lock cleanup mechanism.  If a robot crashes without
        # calling release, its waypoint locks stay forever and block other robots.
        # Options: (a) a heartbeat topic from each nav_server — clear locks for
        # any robot that hasn't published in > N seconds; (b) a TTL on each lock
        # entry reset on every acquire.  Without this, a single crashed robot
        # requires a full system restart to clear.
        # TODO: Add a /traffic/reset_robot service that operators can call to
        # manually clear all locks held by a specific robot after a crash.

        cb = ReentrantCallbackGroup()
        self.create_service(
            AcquireWaypoint, 'traffic/acquire', self._on_acquire, callback_group=cb,
        )
        self.create_service(
            ReleaseWaypoint, 'traffic/release', self._on_release, callback_group=cb,
        )

        self.get_logger().info('Traffic controller ready')

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _robot_prio_in_aisle(self, robot_id: str, key: str) -> int:
        return max(
            (p for wp, (rid, p) in self._waypoint_owners.items()
             if rid == robot_id and _aisle_key(wp) == key),
            default=0,
        )

    def _robot_still_in_aisle(self, robot_id: str, key: str) -> bool:
        return any(
            _aisle_key(wp) == key and rid == robot_id
            for wp, (rid, _) in self._waypoint_owners.items()
        )

    def _get_yield_pub(self, robot_id: str):
        if robot_id not in self._yield_pubs:
            self._yield_pubs[robot_id] = self.create_publisher(
                Empty, f'/{robot_id}/yield_request', 10,
            )
        return self._yield_pubs[robot_id]

    def _resolve_deadlock(
        self,
        robot_a: str, prio_a: int, wants_a: str,
        robot_b: str, prio_b: int, wants_b: str,
    ) -> None:
        """
        Decide which robot should yield and send it a yield signal.

        Decision order
        ~~~~~~~~~~~~~~
        1. Retreat-stuck check: if a robot was recently yielded for the same
           waypoint and is deadlocked again, it couldn't retreat (e.g. it is at
           a dead-end like a dock or charger spur).  Yield the OTHER robot instead.
        2. Aisle-exit rule: if one robot is entering an aisle and the other is
           exiting, the entering robot always yields.
        3. Priority: lower movement priority yields.
        4. Tie-break: lexicographically lower robot ID yields (deterministic).
        """
        now = time.time()

        # Check whether either robot has been recently yielded for the same want,
        # which indicates it couldn't retreat (dead-end waypoint).
        a_rec = self._recent_yields.get(robot_a)
        b_rec = self._recent_yields.get(robot_b)
        a_stuck = a_rec and a_rec[0] == wants_a and now - a_rec[1] < self._YIELD_MEMORY_S
        b_stuck = b_rec and b_rec[0] == wants_b and now - b_rec[1] < self._YIELD_MEMORY_S

        if a_stuck and not b_stuck:
            yield_target = robot_b   # A is stuck at a dead end → yield B instead
            reason = f'{robot_a} already yielded for {wants_a} and could not retreat'
        elif b_stuck and not a_stuck:
            yield_target = robot_a
            reason = f'{robot_b} already yielded for {wants_b} and could not retreat'
        else:
            key_a = _aisle_key(wants_a)
            key_b = _aisle_key(wants_b)
            if key_a and not key_b:
                yield_target = robot_a   # A entering aisle, B exiting → A yields
                reason = f'{robot_a} entering aisle, {robot_b} must exit first'
            elif key_b and not key_a:
                yield_target = robot_b
                reason = f'{robot_b} entering aisle, {robot_a} must exit first'
            elif prio_a < prio_b:
                yield_target = robot_a
                reason = f'{robot_a} has lower priority (p{prio_a} < p{prio_b})'
            elif prio_b < prio_a:
                yield_target = robot_b
                reason = f'{robot_b} has lower priority (p{prio_b} < p{prio_a})'
            else:
                yield_target = min(robot_a, robot_b)
                reason = 'equal priority — tie-break by ID'

        self.get_logger().info(
            f'Deadlock: {robot_a}(p{prio_a},→{wants_a}) ↔ '
            f'{robot_b}(p{prio_b},→{wants_b}) — '
            f'yielding {yield_target} [{reason}]'
        )
        self._recent_yields[yield_target] = (
            wants_a if yield_target == robot_a else wants_b, now
        )
        self._get_yield_pub(yield_target).publish(Empty())
        self._pending.pop(yield_target, None)

    # ── Service: acquire ──────────────────────────────────────────────────────

    def _on_acquire(
        self,
        req: AcquireWaypoint.Request,
        resp: AcquireWaypoint.Response,
    ) -> AcquireWaypoint.Response:
        robot_id = req.robot_id
        waypoint = req.waypoint
        priority = req.priority

        with self._lock:
            key = _aisle_key(waypoint)

            # ── Aisle-lock check ──────────────────────────────────────────────
            if key:
                holder = self._aisle_holders.get(key)
                if holder and holder != robot_id:
                    h_prio = self._robot_prio_in_aisle(holder, key)
                    self.get_logger().debug(
                        f'{robot_id}(p{priority}) waiting for aisle {key} '
                        f'— held by {holder}(p{h_prio})'
                    )

                    # Proactive junction check: if this robot is already parked
                    # at the entry junction for this aisle, it is physically
                    # blocking the aisle exit even though no deadlock cycle has
                    # formed yet.  Yield it immediately rather than waiting for
                    # the reactive deadlock detector to catch up.
                    robot_wps = [
                        wp for wp, (rid, _) in self._waypoint_owners.items()
                        if rid == robot_id
                    ]
                    if any(_junction_aisle_key(wp) == key for wp in robot_wps):
                        self.get_logger().info(
                            f'{robot_id} is at entry junction of {key} '
                            f'(held by {holder}) — yielding immediately'
                        )
                        self._get_yield_pub(robot_id).publish(Empty())
                        # Don't record pending — the robot will retreat and replan.
                        resp.granted         = False
                        resp.holder_id       = holder
                        resp.holder_priority = h_prio
                        return resp

                    self._pending[robot_id] = (waypoint, holder)
                    self._check_deadlock(robot_id, priority, waypoint, holder, h_prio)
                    resp.granted         = False
                    resp.holder_id       = holder
                    resp.holder_priority = h_prio
                    return resp

            # ── Waypoint check ────────────────────────────────────────────────
            current = self._waypoint_owners.get(waypoint)

            if current is None or current[0] == robot_id:
                # Free or already owned by this robot.
                self._waypoint_owners[waypoint] = (robot_id, priority)
                if key:
                    self._aisle_holders[key] = robot_id
                self._pending.pop(robot_id, None)
                resp.granted = True
                return resp

            owner_id, owner_priority = current
            self.get_logger().debug(
                f'{robot_id}(p{priority}) waiting for {waypoint} '
                f'— held by {owner_id}(p{owner_priority})'
            )
            self._pending[robot_id] = (waypoint, owner_id)
            self._check_deadlock(robot_id, priority, waypoint, owner_id, owner_priority)
            resp.granted         = False
            resp.holder_id       = owner_id
            resp.holder_priority = owner_priority
            return resp

    def _check_deadlock(
        self,
        robot_id: str, priority: int, wanted_wp: str,
        blocking_id: str, blocking_prio: int,
    ) -> None:
        """
        Called inside _lock.  Check if blocking_id is also waiting on something
        held by robot_id (circular wait), and if so resolve it.
        """
        blocker_pending = self._pending.get(blocking_id)
        if blocker_pending is None:
            return
        _, blocker_blocking = blocker_pending
        if blocker_blocking != robot_id:
            return

        # Circular wait confirmed.
        blocker_wanted, _ = blocker_pending
        self._resolve_deadlock(
            robot_id, priority, wanted_wp,
            blocking_id, blocking_prio, blocker_wanted,
        )

    # ── Service: release ──────────────────────────────────────────────────────

    def _on_release(
        self,
        req: ReleaseWaypoint.Request,
        resp: ReleaseWaypoint.Response,
    ) -> ReleaseWaypoint.Response:
        robot_id = req.robot_id
        waypoint = req.waypoint

        with self._lock:
            current = self._waypoint_owners.get(waypoint)
            if current and current[0] == robot_id:
                del self._waypoint_owners[waypoint]
                self._pending.pop(robot_id, None)

                key = _aisle_key(waypoint)
                if key and self._aisle_holders.get(key) == robot_id:
                    if not self._robot_still_in_aisle(robot_id, key):
                        del self._aisle_holders[key]
                        self.get_logger().debug(
                            f'{robot_id} released aisle lock {key}'
                        )

                resp.success = True
            else:
                resp.success = False

        return resp


def main(args=None):
    rclpy.init(args=args)
    node = TrafficControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
