from picrawler.voice_assistant import VoiceAssistant
from picrawler import Picrawler
import time
import queue
import threading


class VoiceActiveCrawler(VoiceAssistant):

    ACTION_MAP = {
        "forward":      ("do_action", {"motion_name": "forward", "step": 1, "speed": 80}),
        "backward":     ("do_action", {"motion_name": "backward", "step": 1, "speed": 80}),
        "turn left":    ("do_action", {"motion_name": "turn left", "step": 1, "speed": 80}),
        "turn right":   ("do_action", {"motion_name": "turn right", "step": 1, "speed": 80}),
        "sit":          ("do_step", {"_step": "sit", "speed": 50}),
        "stand":        ("do_step", {"_step": "stand", "speed": 50}),
        "wave":         ("do_action", {"motion_name": "wave", "step": 1, "speed": 60}),
        "push up":      ("do_action", {"motion_name": "push_up", "step": 1, "speed": 50}),
        "dance":        ("do_action", {"motion_name": "dance", "step": 1, "speed": 80}),
        "look left":    ("do_action", {"motion_name": "look_left", "step": 1, "speed": 60}),
        "look right":   ("do_action", {"motion_name": "look_right", "step": 1, "speed": 60}),
        "look up":      ("do_action", {"motion_name": "look_up", "step": 1, "speed": 60}),
        "look down":    ("do_action", {"motion_name": "look_down", "step": 1, "speed": 60}),
    }

    def __init__(self, *args, **kwargs):
        self.action_queue = queue.Queue()
        self._action_thread = None
        self._action_running = False
        try:
            super().__init__(*args, **kwargs)
        except Exception as e:
            self._handle_init_error(e)
            raise
        self._init_crawler()

    @staticmethod
    def _handle_init_error(error):
        err_msg = str(error)
        if 'PortAudioError' in type(error).__name__ or 'query_devices' in err_msg:
            print("=" * 55)
            print("  No microphone detected!")
            print("  The voice assistant requires a microphone for")
            print("  speech recognition and wake word detection.")
            print()
            print("  Options:")
            print("  1. Connect a USB microphone to the Raspberry Pi")
            print("  2. Connect an I2S MEMS mic and enable it:")
            print("     Edit /boot/firmware/config.txt")
            print('     Uncomment: dtparam=i2s=on')
            print("=" * 55)
        else:
            print(f"VoiceActiveCrawler init failed: {err_msg}")

    def _init_crawler(self):
        try:
            self.crawler = Picrawler()
            time.sleep(1)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize Picrawler: {e}")

    # ── lifecycle overrides ──────────────────────────────────────────

    def on_start(self):
        self._action_running = True
        self._action_thread = threading.Thread(
            target=self._action_handler, daemon=True
        )
        self._action_thread.start()
        self.crawler.do_step("sit", speed=50)

    def before_listen(self):
        self.crawler.do_step("sit", speed=50)

    def on_wake(self):
        pass

    def on_heard(self, text):
        pass

    def before_think(self, text):
        pass

    def parse_response(self, text):
        result = text.strip().split('ACTIONS: ')

        response_text = result[0].strip()
        if len(result) > 1:
            actions_str = result[1].strip()
            if actions_str:
                actions = [a.strip() for a in actions_str.split(', ')]
            else:
                actions = ['stop']
        else:
            actions = ['stop']

        for action in actions:
            self.action_queue.put(action)

        return response_text

    def before_say(self, text):
        pass

    def after_say(self, text):
        self._wait_actions_done()
        self.crawler.do_step("sit", speed=50)

    def on_finish_a_round(self):
        self._wait_actions_done()
        self.crawler.do_step("sit", speed=50)

    def on_stop(self):
        self._action_running = False
        self.crawler.do_step("sit", speed=50)

    # ── action dispatch ──────────────────────────────────────────────

    def _action_handler(self):
        while self._action_running:
            try:
                action = self.action_queue.get(timeout=0.5)
                if action == 'stop':
                    self.crawler.do_step("sit", speed=50)
                elif action in self.ACTION_MAP:
                    method_name, kwargs = self.ACTION_MAP[action]
                    getattr(self.crawler, method_name)(**kwargs)
                else:
                    print(f"Unknown action: {action}")
            except queue.Empty:
                continue

    def _wait_actions_done(self):
        while not self.action_queue.empty():
            time.sleep(0.05)
        time.sleep(0.1)
