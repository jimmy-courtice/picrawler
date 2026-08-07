"""
Continuous XYZ gait engine for SunFounder PiCrawler.

Inspired by Freenove FNQR (FNK0030):
  - interpolate foot targets in Cartesian space, then IK each tick
  - body translate via per-leg local-frame deltas (PiCrawler is not one shared XYZ)
  - crawl = body shift + one-leg lift/swing/plant

Does not modify picrawler's MoveList. Wrap a Picrawler (or FakeCrawler) and call
stand / crawl_forward / move_body from your own scripts.
"""

from __future__ import annotations

import copy
import math
from typing import List, Optional, Sequence, Union

# Leg order matches Picrawler / MoveList:
#   0 right front, 1 left front, 2 left rear, 3 right rear
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
    """Stay inside a conservative PiCrawler workspace (mm)."""
    return [
        max(35.0, min(80.0, p[0])),
        max(-10.0, min(90.0, p[1])),
        max(-75.0, min(-20.0, p[2])),
    ]


def body_foot_deltas(dx: float, dy: float, dz: float) -> Pose:
    """
    Foot deltas that produce a body translation (dx, dy, dz).

    PiCrawler stores each leg in its own local frame (see MoveList.move_body_absolute).
    Front/rear and left/right flip the signs — you cannot add the same vector to all four feet.
    """
    return [
        [-dx, -dy, -dz],  # RF
        [dx, -dy, -dz],  # LF
        [dx, dy, -dz],  # LR
        [-dx, dy, -dz],  # RR
    ]


class FakeCrawler:
    """In-memory stand-in so you can dry-run gait math off the robot."""

    def __init__(self) -> None:
        # Approximate stock stand (stand_position 0)
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
    """Cartesian gait controller layered on Picrawler."""

    # Conservative defaults — easier to increase than to un-tip the robot
    X_DEFAULT = 45.0
    Y_DEFAULT = 45.0
    Y_START = 0.0
    Z_DOWN = -50.0
    Z_UP = -30.0

    BODY_SHIFT = 5.0
    STRIDE = 18.0
    LIFT = 15.0

    def __init__(
        self,
        crawler,
        step_mm: float = 2.0,
        servo_speed: int = 90,
        epsilon: float = 0.8,
    ) -> None:
        self.crawler = crawler
        self.step_mm = step_mm
        self.servo_speed = servo_speed
        self.epsilon = epsilon
        self._goals: Optional[Pose] = None
        # 0 = next swing pair LF/RR, 1 = RF/LR
        self.phase = 0

    # ── primitives ──────────────────────────────────────────────────────────

    def feet(self) -> Pose:
        return self.crawler.current_step_all_leg_value()

    def set_goals(self, goals: Pose) -> None:
        self._goals = [_clamp_point(list(map(float, g))) for g in goals]

    def tick(self) -> bool:
        """Advance all feet one step_mm toward goals. Returns True if still moving."""
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
        """Translate the body using PiCrawler per-leg local frames."""
        self.move_feet_relatively(body_foot_deltas(dx, dy, dz), step_mm=step_mm)

    def swing_leg(
        self,
        leg: int,
        dx: float = 0.0,
        dy: float = 0.0,
        lift: Optional[float] = None,
        step_mm: Optional[float] = None,
    ) -> None:
        """Lift one foot, translate in local XY, plant. Stance feet hold still."""
        lift = self.LIFT if lift is None else lift
        cur = self.feet()
        z0 = cur[leg][2]

        up = copy.deepcopy(cur)
        up[leg][2] = min(z0 + lift, self.Z_UP + 5)
        self.move_feet_to(up, step_mm=step_mm)

        mid = self.feet()
        mid[leg][0] += dx
        mid[leg][1] += dy
        self.move_feet_to(mid, step_mm=step_mm)

        down = self.feet()
        down[leg][2] = z0
        self.move_feet_to(down, step_mm=step_mm)

    # ── high-level ──────────────────────────────────────────────────────────

    def stand(self, speed: int = 40) -> None:
        self.crawler.do_step("stand", speed)
        self.phase = 0
        self._goals = self.feet()

    def sit(self, speed: int = 40) -> None:
        self.crawler.do_step("sit", speed)
        self._goals = self.feet()

    def crawl_forward(self, cycles: int = 1) -> None:
        """Creeping crawl: body shifts; stance feet slide; one leg swings ahead."""
        order = ([LF, RR], [RF, LR])
        for _ in range(cycles):
            for swing in order[self.phase]:
                self._crawl_one_leg(swing, direction=1.0)
            self.phase = 1 - self.phase

    def crawl_backward(self, cycles: int = 1) -> None:
        order = ([LF, RR], [RF, LR])
        for _ in range(cycles):
            for swing in order[self.phase]:
                self._crawl_one_leg(swing, direction=-1.0)
            self.phase = 1 - self.phase

    def _crawl_one_leg(self, swing: int, direction: float = 1.0) -> None:
        shift = self.BODY_SHIFT * direction
        stride = self.STRIDE * direction
        ground_z = self.Z_DOWN
        stance_deltas = body_foot_deltas(0.0, shift, 0.0)

        # Body forward/back with correct local-frame signs
        self.move_body(0.0, shift, 0.0, step_mm=2.0)

        # Lift swing leg
        cur = self.feet()
        up = copy.deepcopy(cur)
        up[swing][2] = min(cur[swing][2] + self.LIFT, self.Z_UP + 5.0)
        self.move_feet_to(up, step_mm=2.0)

        # Swing leg forward in local +Y (stock gait convention for all legs),
        # while stance feet take another body-shift delta.
        mid = self.feet()
        for i in range(4):
            if i == swing:
                mid[i][1] += stride
            else:
                mid[i][0] += stance_deltas[i][0]
                mid[i][1] += stance_deltas[i][1]
                mid[i][2] += stance_deltas[i][2]
        self.move_feet_to(mid, step_mm=2.0)

        # Plant
        down = self.feet()
        down[swing][2] = ground_z
        self.move_feet_to(down, step_mm=2.0)

        # Follow-through
        self.move_body(0.0, shift * 0.5, 0.0, step_mm=2.0)

    def smooth_stock_forward(self, steps: int = 1) -> None:
        """
        Replay Picrawler's built-in forward keyframes, but lerp each
        pose-to-pose transition in XYZ. Safest "does it look smoother?" test.
        """
        move_list = getattr(self.crawler, "move_list", None)
        if move_list is None:
            raise RuntimeError("smooth_stock_forward requires a Picrawler with move_list")

        for _ in range(steps):
            move_list.stand_position = getattr(self.crawler, "stand_position", 0)
            action = move_list["forward"]
            if hasattr(self.crawler, "stand_position"):
                self.crawler.stand_position = self.crawler.stand_position + 1 & 1
            for pose in action:
                self.move_feet_to([[float(v) for v in leg] for leg in pose], step_mm=2.5)

    def demo_body_sway(self) -> None:
        """Small body slides with feet planted (optional; can look like shaking if too big)."""
        self.move_body(8, 0, 0)
        self.move_body(-16, 0, 0)
        self.move_body(8, 0, 0)
        self.move_body(0, 8, 0)
        self.move_body(0, -16, 0)
        self.move_body(0, 8, 0)
