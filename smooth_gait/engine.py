"""
Continuous XYZ gait layer for SunFounder PiCrawler.

Working approach: replay stock MoveList gaits, but interpolate each
pose→pose transition in foot XYZ (then IK), instead of one big joint-angle
lerp. That keeps locomotion that already works, and removes a lot of the
mechanical "snap" between keyframes.

Experimental: move_body() uses PiCrawler per-leg local-frame signs
(see MoveList.move_body_absolute).
"""

from __future__ import annotations

import copy
import math
from typing import List, Optional, Sequence, Union

RF, LF, LR, RR = 0, 1, 2, 3

Point = List[float]
Pose = List[Point]


def _dist(a: Point, b: Point) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _lerp(a: Point, b: Point, t: float) -> Point:
    return [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]


def _clamp_point(p: Point) -> Point:
    return [
        max(35.0, min(80.0, p[0])),
        max(-10.0, min(90.0, p[1])),
        max(-75.0, min(-20.0, p[2])),
    ]


def body_foot_deltas(dx: float, dy: float, dz: float) -> Pose:
    """Foot deltas for a body translation; PiCrawler local-frame signs."""
    return [
        [-dx, -dy, -dz],  # RF
        [dx, -dy, -dz],  # LF
        [dx, dy, -dz],  # LR
        [-dx, dy, -dz],  # RR
    ]


class FakeCrawler:
    def __init__(self) -> None:
        self.current_coord: Pose = [
            [45.0, 45.0, -50.0],
            [45.0, 0.0, -50.0],
            [45.0, 0.0, -50.0],
            [45.0, 45.0, -50.0],
        ]
        self.history: List[Pose] = []
        self.stand_position = 0
        self.move_list = None

    def do_step(self, step: Union[str, Sequence[Sequence[float]]], speed: int = 50) -> None:
        if isinstance(step, str):
            if step == "stand":
                self.current_coord = [
                    [45.0, 45.0, -50.0],
                    [45.0, 0.0, -50.0],
                    [45.0, 0.0, -50.0],
                    [45.0, 45.0, -50.0],
                ]
            elif step == "sit":
                self.current_coord = [
                    [45.0, 45.0, -30.0],
                    [70.0, 0.0, -30.0],
                    [70.0, 0.0, -30.0],
                    [45.0, 45.0, -30.0],
                ]
            else:
                raise ValueError(f"FakeCrawler: unknown named step {step!r}")
        else:
            self.current_coord = [[float(v) for v in leg] for leg in step]
        self.history.append(copy.deepcopy(self.current_coord))

    def current_step_all_leg_value(self) -> Pose:
        return copy.deepcopy(self.current_coord)


class SmoothGait:
    """Cartesian interpolation helpers + smoothed stock gaits."""

    def __init__(
        self,
        crawler,
        step_mm: float = 2.0,
        servo_speed: int = 80,
        epsilon: float = 0.75,
    ) -> None:
        self.crawler = crawler
        self.step_mm = step_mm
        self.servo_speed = servo_speed
        self.epsilon = epsilon
        self._goals: Optional[Pose] = None

    def feet(self) -> Pose:
        return self.crawler.current_step_all_leg_value()

    def set_goals(self, goals: Pose) -> None:
        self._goals = [_clamp_point(list(map(float, g))) for g in goals]

    def tick(self) -> bool:
        if self._goals is None:
            return False
        current = self.feet()
        nxt: Pose = []
        busy = False
        for i in range(4):
            d = _dist(current[i], self._goals[i])
            if d <= self.epsilon:
                nxt.append(list(self._goals[i]))
            else:
                busy = True
                t = min(1.0, self.step_mm / d)
                nxt.append(_clamp_point(_lerp(current[i], self._goals[i], t)))
        self.crawler.do_step(nxt, speed=self.servo_speed)
        return busy

    def wait(self) -> None:
        while self.tick():
            pass

    def move_feet_to(self, goals: Pose, step_mm: Optional[float] = None) -> None:
        old = self.step_mm
        if step_mm is not None:
            self.step_mm = step_mm
        try:
            self.set_goals(goals)
            self.wait()
        finally:
            self.step_mm = old

    def move_feet_relatively(self, deltas: Sequence[Sequence[float]], step_mm: Optional[float] = None) -> None:
        cur = self.feet()
        goals = [
            [cur[i][0] + deltas[i][0], cur[i][1] + deltas[i][1], cur[i][2] + deltas[i][2]]
            for i in range(4)
        ]
        self.move_feet_to(goals, step_mm=step_mm)

    def move_body(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, step_mm: Optional[float] = None) -> None:
        self.move_feet_relatively(body_foot_deltas(dx, dy, dz), step_mm=step_mm)

    def _sync_move_list_standing(self, standing: bool) -> None:
        """
        Picrawler.__init__ builds step_list by evaluating stand then sit, which
        leaves move_list.z_current at Z_UP. do_step('stand'/'sit') uses those
        cached poses and never updates z_current — so check_stand keeps
        injecting a full stand animation into every gait. Keep the flag in sync.
        """
        ml = getattr(self.crawler, "move_list", None)
        if ml is None:
            return
        if standing:
            ml.z_current = ml.Z_DEFAULT
            ml.ready_state = 1
        else:
            ml.z_current = ml.Z_UP

    def stand(self, speed: int = 40) -> None:
        self.crawler.do_step("stand", speed)
        self._sync_move_list_standing(True)
        self._goals = self.feet()

    def sit(self, speed: int = 40) -> None:
        self.crawler.do_step("sit", speed)
        self._sync_move_list_standing(False)
        self._goals = self.feet()

    def _gait_keyframes(self, motion_name: str):
        """Fetch MoveList poses with standing flags set so check_stand is a no-op."""
        move_list = self.crawler.move_list
        self._sync_move_list_standing(True)
        move_list.stand_position = getattr(self.crawler, "stand_position", 0)
        return move_list[motion_name]

    def smooth_action(self, motion_name: str, times: int = 1, step_mm: float = 2.0) -> None:
        """
        Run a named MoveList gait (forward, backward, turn left, ...) with
        XYZ interpolation between every keyframe. Caller should already stand().
        """
        if getattr(self.crawler, "move_list", None) is None:
            raise RuntimeError("smooth_action requires a Picrawler with move_list")

        toggle = motion_name in (
            "forward",
            "backward",
            "turn left",
            "turn right",
            "turn left angle",
            "turn right angle",
        )

        for i in range(times):
            action = self._gait_keyframes(motion_name)
            if toggle:
                self.crawler.stand_position = self.crawler.stand_position + 1 & 1
            print(f"  step {i + 1}/{times} ({len(action)} keyframes)")
            for pose in action:
                self.move_feet_to([[float(v) for v in leg] for leg in pose], step_mm=step_mm)

    def crawl_forward(self, cycles: int = 1) -> None:
        """Smoothed stock forward gait (reliable locomotion)."""
        self.smooth_action("forward", times=cycles, step_mm=2.0)

    def crawl_backward(self, cycles: int = 1) -> None:
        self.smooth_action("backward", times=cycles, step_mm=2.0)

    def demo_body_sway(self) -> None:
        self.move_body(6, 0, 0)
        self.move_body(-12, 0, 0)
        self.move_body(6, 0, 0)
        self.move_body(0, 6, 0)
        self.move_body(0, -12, 0)
        self.move_body(0, 6, 0)
