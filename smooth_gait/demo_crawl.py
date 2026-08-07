#!/usr/bin/env python3
"""
Smoother gait demo for PiCrawler.

  cd ~/picrawler && git pull
  sudo python3 -m smooth_gait.demo_crawl --cycles 3

  # Compare stock vs smoothed:
  sudo python3 -m smooth_gait.demo_crawl --mode compare --cycles 2
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
    parser.add_argument(
        "--mode",
        choices=("crawl", "compare", "sway"),
        default="crawl",
        help="crawl = smoothed forward; compare = stock then smooth; sway = body only",
    )
    parser.add_argument("--cycles", type=int, default=3, help="Forward steps")
    args = parser.parse_args()

    crawler, dry = build_crawler(args.dry_run)
    gait = SmoothGait(crawler, step_mm=2.0, servo_speed=80)
    did_sit = False

    try:
        print("1) Stand (once)")
        gait.stand(40)
        time.sleep(1.0)

        if args.mode == "sway":
            print("2) Body sway (no walk)")
            gait.demo_body_sway()

        elif args.mode == "compare" and not dry:
            print("2) Stock forward")
            for i in range(args.cycles):
                print(f"  stock step {i + 1}/{args.cycles}")
                crawler.do_action("forward", 1, 60)
                time.sleep(0.1)
            time.sleep(0.8)

            print("3) Smoothed forward")
            # still standing — do not stand again
            gait._sync_move_list_standing(True)
            gait.crawl_forward(cycles=args.cycles)

        else:
            if dry:
                print("2) Dry-run skip walk")
            else:
                print("2) Smoothed forward (no extra stand)")
                gait.crawl_forward(cycles=args.cycles)

        print("3) Sit (once)")
        gait.sit(40)
        did_sit = True
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        if not did_sit:
            try:
                print("Sit (cleanup)")
                gait.sit(40)
                time.sleep(0.4)
            except Exception:
                pass

    if dry and isinstance(crawler, FakeCrawler):
        print(f"Dry-run frames: {len(crawler.history)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
