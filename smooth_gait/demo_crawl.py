#!/usr/bin/env python3
"""
Demo: Freenove-style smooth crawl on PiCrawler.

On the robot (after picrawler + robot_hat are installed):
  cd ~/picrawler
  sudo python3 -m smooth_gait.demo_crawl

Dry-run on any machine (no hardware):
  python3 -m smooth_gait.demo_crawl --dry-run

Compare with stock gait:
  sudo python3 -m smooth_gait.demo_crawl --compare
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# Allow `python3 smooth_gait/demo_crawl.py` from repo root
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from smooth_gait import FakeCrawler, SmoothGait


def build_crawler(dry_run: bool):
    if dry_run:
        return FakeCrawler(), True
    from picrawler import Picrawler

    return Picrawler(), False


def run_smooth(gait: SmoothGait, cycles: int) -> None:
    print("stand → crawl_ready → body sway → crawl_forward ×", cycles)
    gait.stand(40)
    time.sleep(0.5)
    gait.crawl_ready()
    time.sleep(0.3)
    gait.demo_body_sway()
    time.sleep(0.3)
    gait.crawl_forward(cycles=cycles)
    time.sleep(0.3)
    gait.crawl_backward(cycles=max(1, cycles // 2))


def run_compare(gait: SmoothGait, crawler, cycles: int) -> None:
    print("--- stock do_action('forward') ---")
    crawler.do_step("stand", 40)
    time.sleep(0.5)
    for _ in range(cycles):
        crawler.do_action("forward", 1, 60)
        time.sleep(0.2)

    print("--- smooth XYZ replay of stock forward keyframes ---")
    gait.stand(40)
    time.sleep(0.5)
    gait.smooth_stock_forward(steps=cycles)

    print("--- Freenove-style crawl_forward ---")
    gait.crawl_ready()
    gait.crawl_forward(cycles=cycles)


def main() -> int:
    parser = argparse.ArgumentParser(description="Smooth gait demo for PiCrawler")
    parser.add_argument("--dry-run", action="store_true", help="No hardware; FakeCrawler")
    parser.add_argument("--compare", action="store_true", help="Stock vs smooth vs crawl")
    parser.add_argument("--cycles", type=int, default=2, help="Crawl / forward cycles")
    args = parser.parse_args()

    crawler, dry = build_crawler(args.dry_run)
    gait = SmoothGait(crawler, step_mm=2.5, servo_speed=100)

    try:
        if args.compare:
            if dry:
                print("Note: --compare with --dry-run skips stock do_action (needs move_list).")
                print("Running Freenove-style crawl only.")
                gait.stand()
                gait.crawl_ready()
                gait.crawl_forward(cycles=args.cycles)
            else:
                run_compare(gait, crawler, args.cycles)
        else:
            run_smooth(gait, args.cycles)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            gait.sit(40)
            time.sleep(0.5)
        except Exception:
            pass

    if dry and isinstance(crawler, FakeCrawler):
        print(f"Dry-run OK — {len(crawler.history)} pose frames generated.")
        print("Last pose:", crawler.current_coord)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
