"""Raw TIM3/TIM8 interface. Counts are latched together on the MCU."""
from dataclasses import dataclass

from units import turns_to_counts


def encoder_delta(current, previous):
    """Signed 16-bit difference, identical to the STM32 timer arithmetic."""
    return ((current - previous + 32768) % 65536) - 32768


@dataclass(frozen=True, slots=True)
class EncoderSample:
    time: float
    available_time: float
    tim3: int
    tim8: int

    def __post_init__(self):
        if any(type(v) is not int or not 0 <= v <= 65535 for v in (self.tim3, self.tim8)):
            raise ValueError("Encoder registers must be unsigned 16-bit integers")

    @classmethod
    def from_wheels(cls, car, time, latency):
        left, right = (turns_to_counts(v, car.c["counts"]) for v in car.rotations)
        return cls(time, time + latency, left % 65536, (-right) % 65536)

    def travel_delta(self, previous):
        return (encoder_delta(self.tim3, previous.tim3),
                -encoder_delta(self.tim8, previous.tim8))
