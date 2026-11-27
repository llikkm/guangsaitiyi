"""Controller-facing values. This module has no simulator or hardware imports."""
from dataclasses import dataclass
import math
from wall_localization import WallFix


@dataclass(frozen=True, slots=True)
class MotorCommand:
    left_pwm: float
    right_pwm: float

    def __post_init__(self):
        if not all(-1 <= x <= 1 for x in (self.left_pwm, self.right_pwm)):
            raise ValueError("PWM must be in [-1, 1]")


@dataclass(frozen=True, slots=True)
class WheelSpeed:
    left: float
    right: float

    def __post_init__(self):
        if not all(math.isfinite(x) for x in (self.left, self.right)):
            raise ValueError("Wheel targets must be finite mm/s")


@dataclass(frozen=True, slots=True)
class Observation:
    time: float
    estimated_pose: tuple
    lidar: tuple
    lidar_time: float
    gyro_rad_s: float
    imu_time: float
    encoder_counts: tuple
    encoder_time: float
    wall_fix: WallFix | None = None  # WallFix for the most recent scan

    def read_lidar(self):
        return self.lidar

    def read_imu(self):
        return self.gyro_rad_s

    def read_encoders(self):
        return self.encoder_counts

    @staticmethod
    def set_pwm(left_pwm, right_pwm):
        return MotorCommand(left_pwm, right_pwm)

    @staticmethod
    def set_wheel_speed(left, right):
        return WheelSpeed(left, right)

    @staticmethod
    def stop():
        return MotorCommand(0, 0)
