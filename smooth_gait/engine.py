"""
Smoothed gait layer for SunFounder PiCrawler.

Replays stock MoveList gaits, inserting a few XYZ midpoints between
keyframes (then IK). Avoids the glacial "2mm tick" approach — each
do_step() already spends tens..hundreds of ms inside servo_move, so
lots of tiny ticks = slow stop-motion.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Union

RF, LF, LR, RR = 0, 1, 2, 3

Point = List[float]
Pose = List[Point]


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
    return [
        [-dx, -dy, -dz],
        [dx, -dy, -dz],
        [dx, dy, -dz],
        [-dx, dy, -dz],
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
        self.history.append(copy_pose(self.current_coord))

    def current_step_all_leg_value(self) -> Pose:
        return copy_pose(self.current_coord)


def copy_pose(pose: Pose) -> Pose:
    return [[float(v) for v in leg] for leg in pose]


class SmoothGait:
    def __init__(
        self,
        crawler,
        segments: int = 2,
        servo_speed: int = 100,
    ) -> None:
        """
        segments: intermediate XYZ poses between keyframes (1 = same as stock,
                  2 = one midpoint, 3 = two midpoints…). Keep this small.
        servo_speed: passed to do_step; use 90–100 so each segment isn't slow.
        """
        self.crawler = crawler
        self.segments = max(1, int(segments))
        self.servo_speed = int(servo_speed)

    def feet(self) -> Pose:
        return self.crawler.current_step_all_leg_value()

    def move_feet_to(self, goals: Pose, segments: Optional[int] = None, speed: Optional[int] = None) -> None:
        """Go to goals with a few XYZ-linear midpoints (not mm-by-mm crawling)."""
        segs = self.segments if segments is None else max(1, int(segments))
        spd = self.servo_speed if speed is None else int(speed)
        start = self.feet()
        end = [_clamp_point(list(map(float, g))) for g in goals]

        for i in range(1, segs + 1):
            t = i / segs
            pose = [_clamp_point(_lerp(start[j], end[j], t)) for j in range(4)]
            self.crawler.do_step(pose, speed=spd)

    def move_feet_relatively(self, deltas: Sequence[Sequence[float]], **kwargs) -> None:
        cur = self.feet()
        goals = [
            [cur[i][0] + deltas[i][0], cur[i][1] + deltas[i][1], cur[i][2] + deltas[i][2]]
            for i in range(4)
        ]
        self.move_feet_to(goals, **kwargs)

    def move_body(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, **kwargs) -> None:
        self.move_feet_relatively(body_foot_deltas(dx, dy, dz), **kwargs)

    def _sync_move_list_standing(self, standing: bool) -> None:
        ml = getattr(self.crawler, "move_list", None)
        if ml is None:
            return
        if standing:
            ml.z_current = ml.Z_DEFAULT
            ml.ready_state = 1
        else:
            ml.z_current = ml.Z_UP

    def stand(self, speed: int = 50) -> None:
        self.crawler.do_step("stand", speed)
        self._sync_move_list_standing(True)

    def sit(self, speed: int = 50) -> None:
        self.crawler.do_step("sit", speed)
        self._sync_move_list_standing(False)

    def _gait_keyframes(self, motion_name: str):
        move_list = self.crawler.move_list
        self._sync_move_list_standing(True)
        move_list.stand_position = getattr(self.crawler, "stand_position", 0)
        return move_list[motion_name]

    def smooth_action(self, motion_name: str, times: int = 1) -> None:
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
            print(
                f"  step {i + 1}/{times} "
                f"({len(action)} keyframes × {self.segments} segments, speed={self.servo_speed})"
            )
            for pose in action:
                self.move_feet_to([[float(v) for v in leg] for leg in pose])

    def crawl_forward(self, cycles: int = 1) -> None:
        self.smooth_action("forward", times=cycles)

    def crawl_backward(self, cycles: int = 1) -> None:
        self.smooth_action("backward", times=cycles)

    def demo_body_sway(self) -> None:
        self.move_body(6, 0, 0)
        self.move_body(-12, 0, 0)
        self.move_body(6, 0, 0)
        self.move_body(0, 6, 0)
        self.move_body(0, -12, 0)
        self.move_body(0, 6, 0)
