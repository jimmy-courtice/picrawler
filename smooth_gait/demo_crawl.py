#!/usr/bin/env python3
"""
Thin runner for the native Picrawler defaults (edit MoveList in picrawler.py).

  cd ~/picrawler && git pull
  sudo pip3 install . --break-system-packages --no-build-isolation --no-deps
  sudo python3 -m smooth_gait.demo_crawl
"""

from __future__ import annotations

import argparse
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--segments", type=int, default=2)
    parser.add_argument("--speed", type=int, default=100)
    parser.add_argument(
        "--mode",
        choices=("crawl", "compare"),
        default="crawl",
        help="compare = 3 steps at segments=1 then 3 at your --segments",
    )
    args = parser.parse_args()

    from picrawler import Picrawler

    crawler = Picrawler(smooth_segments=args.segments)
    try:
        print("stand (diagonal — defined in MoveList.stand)")
        crawler.do_step("stand", 50)
        time.sleep(0.8)

        if args.mode == "compare":
            print("stock-ish playback (segments=1) × 3")
            crawler.smooth_segments = 1
            crawler.do_action("forward", 3, 60)
            time.sleep(0.4)
            print(f"smooth playback (segments={args.segments}) × 3")
            crawler.smooth_segments = args.segments
            crawler.do_step("stand", 50)
            time.sleep(0.3)
            crawler.do_action("forward", 3, args.speed)
        else:
            print(f"forward × {args.cycles}")
            crawler.do_action("forward", args.cycles, args.speed)

        print("stand")
        crawler.do_step("stand", 50)
        time.sleep(0.4)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            crawler.do_step("sit", 50)
            time.sleep(0.3)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
