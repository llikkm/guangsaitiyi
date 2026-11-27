"""Fixed-step scheduling and the ordered V1.1 input stream."""
import json


DIVISORS = (1, 2, 5, 10, 20, 30, 60)
FRAME_RATES = (15, 30, 60, 120)


class StepClock:
    def __init__(self, rate):
        self.rate = rate
        self.credit = 0

    def advance(self, milliseconds, divisor):
        self.credit += milliseconds * self.rate * (60 // divisor)
        steps, self.credit = divmod(self.credit, 60000)
        return steps


class Timeline:
    def __init__(self, events=None):
        self.events = [] if events is None else events
        self.index = 0
        self.tick = 0

    def record(self, action, value=None):
        self.events.append(dict(action=action, value=value, step=self.tick))
        if action == "drive":
            self.tick += 1

    def advance(self, world):
        """Apply events at the current tick, then at most one physics step."""
        while self.index < len(self.events):
            event = self.events[self.index]
            self.index += 1
            action = event["action"]
            if event["step"] != self.tick:
                raise ValueError("Invalid replay step")
            if action == "drive":
                world.step(event["value"])
                self.tick += 1
                return
            if action == "penalty":
                world.match.penalize(event["value"])
            elif action == "unjam":
                world.reset_cars()
            elif action != "pause":
                raise ValueError(f"Unknown replay action {action}")


def load_replay(path):
    with open(path, encoding="utf8") as stream:
        header = json.loads(next(stream))
        if header.get("software") != "1.1" or header.get("format") != 2:
            raise ValueError("Replay requires a V1.1 input log")
        events = [row for line in stream if "action" in (row := json.loads(line))]
    return header, Timeline(events)
