#!/usr/bin/env python3
"""
Smoothed gait demo for PiCrawler.

  cd ~/picrawler && git pull

  # Default: 2 XYZ segments between keyframes, servo speed 100 (near stock pace)
  sudo python3 -m smooth_gait.demo_crawl --cycles 3

  # Even closer to stock speed (only endpoints — good A/B baseline):
  sudo python3 -m smooth_gait.demo_crawl --segments 1 --cycles 3

  # A bit smoother, still quick:
  sudo python3 -m smooth_gait.demo_crawl --segments 2 --speed 100 --cycles 3

  # Compare stock vs smooth (3 steps each):
  sudo python3 -m smooth_gait.demo_crawl --mode compare
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from smooth_gait import FakeCrawler, SmoothGait


def build_crawler(dry_run: bool):
    if dry_run:
        return FakeCrawler(), True
    from picrawler import Picrawler

    return Picrawler(), False


def main() -> int:
    parser = argparse.ArgumentParser(description="Smooth gait demo for PiCrawler")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--mode", choices=("crawl", "compare", "sway"), default="crawl")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument(
        "--segments",
        type=int,
        default=2,
        help="XYZ midpoints per keyframe (1=stock-like count, 2=one midpoint). Keep small.",
    )
    parser.add_argument("--speed", type=int, default=100, help="do_step servo speed 1-100")
    args = parser.parse_args()

    crawler, dry = build_crawler(args.dry_run)
    gait = SmoothGait(crawler, segments=args.segments, servo_speed=args.speed)
    did_sit = False

    try:
        print(f"1) Stand  (segments={args.segments}, speed={args.speed})")
        gait.stand(50)
        time.sleep(0.6)

        if args.mode == "sway":
            print("2) Body sway")
            gait.demo_body_sway()
        elif args.mode == "compare" and not dry:
            # Always 3 steps per mode for a fair side-by-side (ignore --cycles)
            steps = 3
            print(f"2) Stock forward @ speed 60  ({steps} steps)")
            for i in range(steps):
                print(f"  stock step {i + 1}/{steps}")
                crawler.do_action("forward", 1, 60)
            time.sleep(0.5)
            print(f"3) Smoothed forward  ({steps} steps)")
            gait._sync_move_list_standing(True)
            gait.crawl_forward(cycles=steps)
        else:
            if dry:
                print("2) Dry-run skip walk")
            else:
                print("2) Smoothed forward")
                gait.crawl_forward(cycles=args.cycles)

        print("3) Sit")
        gait.sit(50)
        did_sit = True
        time.sleep(0.3)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        if not did_sit:
            try:
                gait.sit(50)
            except Exception:
                pass

    if dry and isinstance(crawler, FakeCrawler):
        print(f"Dry-run frames: {len(crawler.history)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
