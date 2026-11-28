"""Controller-facing values. This module has no simulator or hardware imports."""
from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from imu_sensor import ImuSample
    from encoder_sensor import EncoderSample
from wall_localization import WallFix, pavo_scan


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
class ScanSample:
    time: float
    available_time: float
    beams: tuple
    times: tuple

    @classmethod
    def from_pavo(cls, raw_beams, beam_times, available_time):
        """Assemble a driver frame from (packet degrees, metres) and same-clock seconds."""
        beams = pavo_scan(raw_beams)
        times = tuple(beam_times)
        if (not times or len(times) != len(beams) or not all(map(math.isfinite, times))
                or any(b < a for a, b in zip(times, times[1:]))
                or not math.isfinite(available_time) or available_time < times[-1]):
            raise ValueError("Invalid PAVO acquisition/delivery times")
        return cls(times[-1], available_time, beams, times)


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

    lidar_sample: ScanSample | None = None
    imu_sample: "ImuSample | None" = None
    encoder_sample: "EncoderSample | None" = None
    corrected_pose: tuple | None = None

    @property
    def navigation_pose(self):
        return self.corrected_pose if self.corrected_pose is not None else self.estimated_pose

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
