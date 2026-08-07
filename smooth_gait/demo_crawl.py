#!/usr/bin/env python3
"""
Smoother gait demo for PiCrawler.

On the robot:
  cd ~/picrawler && git pull

  # Should actually walk (smoothed stock forward):
  sudo python3 -m smooth_gait.demo_crawl --cycles 3

  # Side-by-side: stock snap vs smoothed
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
        choices=("crawl", "stock", "compare", "sway"),
        default="crawl",
        help="crawl/stock = smoothed forward walk; compare = raw then smooth; sway = body only",
    )
    parser.add_argument("--cycles", type=int, default=3, help="Forward steps")
    args = parser.parse_args()

    crawler, dry = build_crawler(args.dry_run)
    gait = SmoothGait(crawler, step_mm=2.0, servo_speed=80)

    try:
        print("Standing...")
        gait.stand(40)
        time.sleep(1.0)

        if args.mode == "sway":
            print("Body sway only (won't walk)")
            gait.demo_body_sway()

        elif args.mode == "compare" and not dry:
            print("--- A: stock do_action('forward') ---")
            for _ in range(args.cycles):
                crawler.do_action("forward", 1, 60)
                time.sleep(0.15)
            time.sleep(0.8)

            print("--- B: smoothed XYZ forward (same gait, less snap) ---")
            gait.stand(40)
            time.sleep(0.8)
            gait.crawl_forward(cycles=args.cycles)

        else:
            # crawl and stock are the same working path now
            if dry:
                print("Dry-run: stand/sit only (need real Picrawler.move_list to walk)")
            else:
                print(f"Smoothed forward × {args.cycles}")
                gait.crawl_forward(cycles=args.cycles)

        print("Done — sitting...")
    except KeyboardInterrupt:
        print("\nInterrupted — sitting...")
    finally:
        try:
            gait.sit(40)
            time.sleep(0.5)
        except Exception:
            pass

    if dry and isinstance(crawler, FakeCrawler):
        print(f"Dry-run frames: {len(crawler.history)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
