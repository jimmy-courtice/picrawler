#!/usr/bin/env python3
"""
Demo: smoother motion experiments on PiCrawler.

On the robot:
  cd ~/picrawler
  git pull
  # Safest first — stock forward keyframes, XYZ-smoothed:
  sudo python3 -m smooth_gait.demo_crawl --mode stock --cycles 2

  # Freenove-style crawl (after the coordinate fix):
  sudo python3 -m smooth_gait.demo_crawl --mode crawl --cycles 1

Dry-run:
  python3 -m smooth_gait.demo_crawl --dry-run --mode crawl
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
    parser.add_argument("--dry-run", action="store_true", help="No hardware; FakeCrawler")
    parser.add_argument(
        "--mode",
        choices=("stock", "crawl", "sway", "compare"),
        default="stock",
        help="stock=smooth stock forward (default); crawl=creeping gait; sway=body only; compare=all",
    )
    parser.add_argument("--cycles", type=int, default=2, help="Steps / cycles")
    args = parser.parse_args()

    crawler, dry = build_crawler(args.dry_run)
    gait = SmoothGait(crawler, step_mm=2.0, servo_speed=90)

    try:
        gait.stand(40)
        time.sleep(0.8)

        if args.mode in ("stock", "compare"):
            if dry:
                print("stock mode needs Picrawler.move_list; skipping on dry-run")
            else:
                print("--- smooth stock forward ---")
                gait.smooth_stock_forward(steps=args.cycles)
                time.sleep(0.4)

        if args.mode in ("crawl", "compare"):
            print("--- crawl_forward ---")
            gait.stand(40)
            time.sleep(0.5)
            gait.crawl_forward(cycles=args.cycles)
            time.sleep(0.3)
            gait.crawl_backward(cycles=max(1, args.cycles // 2))

        if args.mode in ("sway", "compare"):
            print("--- body sway ---")
            gait.stand(40)
            time.sleep(0.5)
            gait.demo_body_sway()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            gait.sit(40)
            time.sleep(0.5)
        except Exception:
            pass

    if dry and isinstance(crawler, FakeCrawler):
        print(f"Dry-run OK — {len(crawler.history)} pose frames.")
        print("Last pose:", crawler.current_coord)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
