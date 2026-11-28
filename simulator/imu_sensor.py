"""LSM6DS3TR-C planar-vehicle model. Sensor frame is right handed; SI outputs."""
from dataclasses import dataclass
import math


G = 9.80665
GYRO_SENSITIVITY = {125: .004375, 250: .00875, 500: .0175, 1000: .035, 2000: .070}



def imu_defaults():
    """Nominal parameters. Empirical values reproduced by imu_analysis/calibrate.py."""
    return dict(
        gyro_bias_dps=(-1.42202727, -1.67353959, 0.28879459),
        accel_bias_g=(-0.005551, -0.0158905, 0.022509),
        gyro_noise_dps=(0.02906337, 0.02069074, 0.01932339),
        accel_noise_g=(0.000522137, 0.000548142, 0.001173819),
        gyro_scale=(1., 1., 1.00225711),
        accel_scale=(1.000705, 0.9966485, 1.001376),
        mount_rpy_deg=(0.61843354, -1.18660039, 0.),
        gyro_walk_dps_sqrt_s=(0.0002, 0.0002, 0.0002),
        accel_walk_g_sqrt_s=(0.000005, 0.000005, 0.000005),
        latency_s=0.005, stress_factor=3.,
        gyro_range_dps=250., accel_range_g=2.,
        dps_per_count=0.00875, g_per_count=0.000061)


@dataclass(frozen=True)
class ImuSample:
    time: float
    available_time: float
    gyro_rad_s: tuple
    accel_m_s2: tuple
    yaw_rate: float


def _mount_rows(rpy):
    """Rows of sensor-to-body Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = map(math.radians, rpy)
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return ((cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
            (sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
            (-sp, cp*sr, cp*cr))


def _measurement(time, available_time, gyro_dps, accel_g, vertical):
    gyro = tuple(map(math.radians, gyro_dps))
    return ImuSample(time, available_time, gyro, tuple(value*G for value in accel_g),
                     sum(value*z for value, z in zip(gyro, vertical)))


def sample_from_raw(row, settings, available_time=None, mount_rpy_deg=None):
    """Decode an imu_data CSV DictReader row with its settings.json (same clock in seconds)."""
    now = int(row["t_sample_us"]) * 1e-6
    ready = now if available_time is None else available_time
    if not math.isfinite(ready) or ready < now:
        raise ValueError("available_time must be a finite sample-clock time >= acquisition")
    if mount_rpy_deg is None:
        mount_rpy_deg = imu_defaults()["mount_rpy_deg"]
    gyro = tuple(int(row["g" + axis + "_raw"]) * settings["dps_per_count"] for axis in "xyz")
    accel = tuple(int(row["a" + axis + "_raw"]) * settings["g_per_count"] for axis in "xyz")
    return _measurement(now, ready, gyro, accel, _mount_rows(mount_rpy_deg)[2])


class ImuSensor:
    """One sensor at the chassis center. Each car owns its RNG and drift state."""
    def __init__(self, options, mode, rng):
        defaults = imu_defaults()
        self.options = defaults | options
        self.mode = mode.lower()
        if self.mode not in ("ideal", "nominal", "stress"):
            raise ValueError("IMU mode must be Ideal, Nominal or Stress")
        self.rng = rng
        for key, value in self.options.items():
            if key in ("latency_s", "stress_factor", "gyro_range_dps", "accel_range_g",
                       "dps_per_count", "g_per_count"):
                if not math.isfinite(value) or value < 0 or (key != "latency_s" and value == 0):
                    raise ValueError(f"Invalid IMU {key}")
            elif key in defaults:
                if len(value) != 3 or not all(map(math.isfinite, value)):
                    raise ValueError(f"IMU {key} must have three finite components")
                if ("noise" in key or "walk" in key) and min(value) < 0:
                    raise ValueError(f"Negative IMU noise in {key}")
                if "scale" in key and min(value) <= 0:
                    raise ValueError(f"Nonpositive IMU gain in {key}")
        self.rows = _mount_rows(self.options["mount_rpy_deg"])
        self.previous_velocity = None
        self.gyro_walk = [0., 0., 0.]
        self.accel_walk = [0., 0., 0.]
        self.factor = self.options["stress_factor"] if self.mode == "stress" else 1.

    def _convert(self, true, gains, bias, noise, diffusion, walk, dt, step, limit):
        values = []
        low = max(-32768, math.ceil(-limit/step))
        high = min(32767, math.floor(limit/step))
        for i in range(3):
            walk[i] += self.rng.gauss(0., diffusion[i] * self.factor * math.sqrt(dt))
            value = true[i]*gains[i] + bias[i] + walk[i] + self.rng.gauss(0., noise[i]*self.factor)
            count = max(low, min(high, round(value / step)))
            values.append(count * step)
        return values

    def sample(self, car, now, dt):
        """dt is elapsed acquisition time, including scheduler jitter. A reset starts at dt=0."""
        if not math.isfinite(dt) or dt < 0 or not math.isfinite(now):
            raise ValueError("Invalid IMU timestamp or interval")
        body = car.body
        vx, vy = body.velocity  # Physics uses world mm/s.
        ax = ay = 0.
        if self.previous_velocity is not None and dt > 0:
            ax = (vx - self.previous_velocity[0]) / (dt*1000*G)
            ay = (vy - self.previous_velocity[1]) / (dt*1000*G)
        self.previous_velocity = (vx, vy)
        c, s = math.cos(body.angle), math.sin(body.angle)
        forward, left = ax*c + ay*s, -ax*s + ay*c
        x, y, up = self.rows
        accel = [forward*x[i] + left*y[i] + up[i] for i in range(3)]
        gyro = [math.degrees(body.angular_velocity)*z for z in up]
        latency = 0.
        if self.mode != "ideal":
            opt = self.options
            gyro = self._convert(gyro, opt["gyro_scale"], opt["gyro_bias_dps"], opt["gyro_noise_dps"],
                                 opt["gyro_walk_dps_sqrt_s"], self.gyro_walk, dt,
                                 opt["dps_per_count"], opt["gyro_range_dps"])
            accel = self._convert(accel, opt["accel_scale"], opt["accel_bias_g"], opt["accel_noise_g"],
                                  opt["accel_walk_g_sqrt_s"], self.accel_walk, dt,
                                  opt["g_per_count"], opt["accel_range_g"])
            latency = opt["latency_s"]*self.factor
        return _measurement(now, now + latency, gyro, accel, up)
