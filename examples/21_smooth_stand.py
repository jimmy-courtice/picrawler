#!/usr/bin/env python3
"""
Uses the replaced defaults in picrawler.py (diagonal stand + smooth playback).

Customize by editing MoveList.stand / MoveList.forward in:
  picrawler/picrawler.py

  sudo python3 examples/21_smooth_stand.py
"""
from time import sleep
from picrawler import Picrawler

crawler = Picrawler(smooth_segments=2)  # 1 = stock snap between poses


def main():
    try:
        print("stand (all legs at 45,45,-50)")
        crawler.do_step("stand", 50)
        sleep(1.0)

        print("forward × 3 (returns to all-45 rest)")
        crawler.do_action("forward", 3, 100)
        sleep(0.5)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        try:
            crawler.do_step("sit", 50)
            sleep(0.3)
        except Exception:
            pass


if __name__ == "__main__":
    main()
