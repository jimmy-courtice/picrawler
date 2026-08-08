#!/usr/bin/env python3
"""
Demo using the *native* Picrawler API (symmetric stand + smooth segments).

  cd ~/picrawler && git pull
  sudo pip3 install . --break-system-packages --no-build-isolation --no-deps
  sudo python3 -m smooth_gait.demo_crawl
  sudo python3 examples/21_smooth_stand.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--mode", choices=("crawl", "compare"), default="crawl")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--segments", type=int, default=2)
    parser.add_argument("--speed", type=int, default=100)
    args = parser.parse_args()

    if args.dry_run:
        print("Dry-run: import check only")
        from picrawler.picrawler import Picrawler as _P  # noqa: F401
        print("Picrawler class OK (hardware not exercised)")
        return 0

    from picrawler import Picrawler

    crawler = Picrawler(smooth_segments=args.segments)
    try:
        print(f"1) Stand / symmetric rest  (smooth_segments={args.segments})")
        crawler.do_step("stand", 50)
        time.sleep(0.8)

        if args.mode == "compare":
            steps = 3
            print(f"2) Stock-ish: temporarily segments=1, forward × {steps}")
            crawler.smooth_segments = 1
            crawler.do_action("forward", steps, 60)
            time.sleep(0.5)
            print(f"3) Smooth: segments={args.segments}, forward × {steps}")
            crawler.smooth_segments = args.segments
            crawler.do_step("stand", 50)
            time.sleep(0.4)
            crawler.do_action("forward", steps, args.speed)
        else:
            print(f"2) Forward × {args.cycles} @ speed {args.speed}")
            crawler.do_action("forward", args.cycles, args.speed)

        print("3) Symmetric rest")
        crawler.do_step("stand", 50)
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            print("4) Sit")
            crawler.do_step("sit", 50)
            time.sleep(0.3)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
