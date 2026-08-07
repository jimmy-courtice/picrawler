#!/usr/bin/env python3
"""
Smoothed gait demo for PiCrawler.

  cd ~/picrawler && git pull

  # Symmetric rest → walk → symmetric rest → sit
  sudo python3 -m smooth_gait.demo_crawl --cycles 3

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
    parser.add_argument("--mode", choices=("crawl", "compare", "sway", "rest"), default="crawl")
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
    finished = False

    try:
        print(f"1) Symmetric rest  (all legs 45,45,-50)  segments={args.segments} speed={args.speed}")
        # Rise via stock stand first if coming from sit/boot, then settle even
        gait.stand(50)
        time.sleep(0.3)
        gait.rest_symmetric()
        time.sleep(0.6)

        if args.mode == "rest":
            print("   (rest-only mode — holding)")
            time.sleep(2.0)

        elif args.mode == "sway":
            print("2) Body sway")
            gait.demo_body_sway()
            print("3) Symmetric rest")
            gait.rest_symmetric()

        elif args.mode == "compare" and not dry:
            steps = 3
            print(f"2) Prepare diagonal stand → stock forward × {steps}")
            gait.prepare_to_walk()
            for i in range(steps):
                print(f"  stock step {i + 1}/{steps}")
                crawler.do_action("forward", 1, 60)
            time.sleep(0.4)
            print("3) Symmetric rest")
            gait.rest_symmetric()
            time.sleep(0.5)
            print(f"4) Prepare → smoothed forward × {steps}")
            gait.crawl_forward(cycles=steps, prepare=True)
            print("5) Symmetric rest")
            gait.rest_symmetric()

        else:
            if dry:
                print("2) Dry-run skip walk")
                gait.rest_symmetric()
            else:
                print("2) Prepare diagonal stand → smoothed forward")
                gait.crawl_forward(cycles=args.cycles, prepare=True)
                print("3) Symmetric rest")
                gait.rest_symmetric()
                time.sleep(0.4)

        print("4) Sit")
        gait.sit(50)
        finished = True
        time.sleep(0.3)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        if not finished:
            try:
                gait.rest_symmetric()
                gait.sit(50)
            except Exception:
                pass

    if dry and isinstance(crawler, FakeCrawler):
        print(f"Dry-run frames: {len(crawler.history)}")
        print("Last pose:", crawler.current_coord)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
