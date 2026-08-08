#!/usr/bin/env python3
"""
Native Picrawler API with symmetric rest + XYZ-smoothed gait.

  sudo python3 examples/21_smooth_stand.py

Picrawler(smooth_segments=2) makes do_step/do_action interpolate foot XYZ
between keyframes. do_step('stand') ends with all legs at (45, 45, -50).
do_action('forward') auto-transitions through the diagonal gait stance.
"""
from time import sleep
from picrawler import Picrawler

# smooth_segments=1 → stock timing; 2 → one XYZ midpoint (default in this fork)
crawler = Picrawler(smooth_segments=2)


def main():
    try:
        print("Stand (symmetric rest)")
        crawler.do_step("stand", 50)
        sleep(1.0)

        print("Forward × 3 (auto diagonal prep + smooth segments)")
        crawler.do_action("forward", 3, 100)
        sleep(0.4)

        print("Back to symmetric rest")
        crawler.do_step("stand", 50)
        sleep(0.6)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        print("Sit")
        try:
            crawler.do_step("sit", 50)
            sleep(0.4)
        except Exception:
            pass


if __name__ == "__main__":
    main()
