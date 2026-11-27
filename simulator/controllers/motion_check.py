"""A stand-alone acceptance controller. Times are seconds since reset."""
class Controller:
    def reset(self, config):
        self.start = None
        self.speed = config.get("speed_mm_s", 180)

    def update(self, robot, dt):
        if self.start is None:
            self.start = robot.time
        elapsed = robot.time - self.start
        if elapsed < 1:
            return robot.set_wheel_speed(self.speed, self.speed)
        if elapsed < 2:
            return robot.set_pwm(-0.3, 0.3)
        return robot.stop()
