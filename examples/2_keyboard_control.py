from picrawler import Picrawler
from time import sleep
import readchar

crawler = Picrawler()

SPEED_MIN = 50
SPEED_MAX = 90
speed = 70

YAW_TRIM_MIN = -15.0
YAW_TRIM_MAX = 15.0
YAW_TRIM_STEP = 0.5

STEP = 1            # Number of action steps per key press
ACTION_GAP = 0.25   # Delay after each action to reduce current spikes

manual = """
Keyboard Control - PiCrawler

Movement:
  W: Forward
  A: Turn left
  S: Backward
  D: Turn right

Speed:
  + / ] : Increase speed
  - / [ : Decrease speed

Yaw trim (walk steer bias):
  , / < : Decrease trim (more right / less left)
  . / > : Increase trim (more left / less right)
  Y     : Save trim to ~/.config/picrawler_yaw_trim
  0     : Reset trim to 0 (session only until you save)

Other:
  Space  : Stop (no action)
  Ctrl+C : Quit (auto sit)
"""

def clamp(value, min_value, max_value):
    """Limit value within a specified range."""
    return max(min_value, min(max_value, value))

def yaw_trim_supported():
    return hasattr(crawler, "yaw_trim") and hasattr(Picrawler, "load_yaw_trim")

def show_info():
    """Clear terminal and display control instructions."""
    print("\033[H\033[J", end="")  # Clear terminal screen
    print(manual)
    print(f"Current speed: {speed}  (range {SPEED_MIN}-{SPEED_MAX})")
    if yaw_trim_supported():
        saved = Picrawler.load_yaw_trim()
        print(f"Yaw trim: {crawler.yaw_trim:g}  (saved: {saved:g})")
    else:
        print("Yaw trim: unavailable — reinstall picrawler from smooth-gait")
    print(f"Action gap: {ACTION_GAP:.2f}s")

def do_move(action_name):
    """Execute movement action with safety delay."""
    crawler.do_action(action_name, STEP, speed)
    sleep(ACTION_GAP)

def nudge_yaw(delta):
    crawler.yaw_trim = clamp(
        crawler.yaw_trim + delta,
        YAW_TRIM_MIN,
        YAW_TRIM_MAX,
    )

def safe_sit():
    """Safely sit down before program exit."""
    try:
        crawler.do_step("sit", clamp(speed, 20, 40))
        sleep(1.0)
    except Exception:
        pass

def main():
    show_info()

    try:
        while True:
            key = readchar.readkey()
            k = key.lower()

            if k == "w":
                do_move("forward")
            elif k == "s":
                do_move("backward")
            elif k == "a":
                do_move("turn left")
            elif k == "d":
                do_move("turn right")

            elif k in ("+", "]"):
                global speed
                speed = clamp(speed + 5, SPEED_MIN, SPEED_MAX)

            elif k in ("-", "["):
                speed = clamp(speed - 5, SPEED_MIN, SPEED_MAX)

            elif k in (",", "<"):
                if yaw_trim_supported():
                    nudge_yaw(-YAW_TRIM_STEP)

            elif k in (".", ">"):
                if yaw_trim_supported():
                    nudge_yaw(YAW_TRIM_STEP)

            elif k == "y":
                if yaw_trim_supported():
                    crawler.save_yaw_trim()

            elif k == "0":
                if yaw_trim_supported():
                    crawler.yaw_trim = 0.0

            elif k == " ":
                pass

            elif key == readchar.key.CTRL_C:
                print("\nQuit.")
                break

            show_info()
            sleep(0.02)

    except KeyboardInterrupt:
        print("\nQuit (KeyboardInterrupt).")

    finally:
        safe_sit()

if __name__ == "__main__":
    main()
