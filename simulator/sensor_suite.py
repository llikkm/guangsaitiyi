"""V2.2 acquisition, delivery and odometry, one instance per vehicle."""
from collections import deque
from bisect import bisect_right
import math
import random

from encoder_sensor import EncoderSample
from imu_sensor import GYRO_SENSITIVITY, ImuSensor, imu_defaults
from lidar_sensor import LidarSensor, lidar_defaults
from robot_api import ScanSample
from units import counts_to_travel
from wall_localization import RectangleLocator, WallTracker


def sensor_options(data):
    mode = data.get("mode", "Ideal")
    if mode not in ("Ideal", "Nominal", "Stress"):
        raise ValueError("Sensor mode must be Ideal, Nominal or Stress")
    imu = imu_defaults() | data.get("imu", {})
    if "dps_per_count" not in data.get("imu", {}) and imu["gyro_range_dps"] in GYRO_SENSITIVITY:
        imu["dps_per_count"] = GYRO_SENSITIVITY[imu["gyro_range_dps"]]
    if imu["gyro_range_dps"] > 32768 * imu["dps_per_count"]:
        raise ValueError("gyro_range_dps exceeds the 16-bit range at this dps_per_count")
    return dict(mode=mode, imu=imu,
                lidar=lidar_defaults() | {"motion_compensation": "odometry"} | data.get("lidar", {}),
                encoder={"latency_s": 0., "stress_latency_s": 0.02} | data.get("encoder", {}))


def pose_at(history, stamps, time):
    right = bisect_right(stamps, time)
    if right == 0:
        return history[0][1]
    t0, (x, y, h) = history[right-1]
    if right == len(history) or time == t0:
        return x, y, h
    t1, (nx, ny, nh) = history[right]
    f = (time-t0)/(t1-t0)
    return x+f*(nx-x), y+f*(ny-y), h+f*math.remainder(nh-h, math.tau)


def deskew(frame, history, stamps, end_pose, mount):
    """Express each return in the sensor frame at the last beam's time."""
    mx, my, yaw = mount
    def sensor_pose(body):
        x, y, h = body
        c, s = math.cos(h), math.sin(h)
        return x+c*mx-s*my, y+s*mx+c*my, h+yaw

    ex, ey, eh = sensor_pose(end_pose)
    beams = []
    for (angle, distance), time in zip(frame.beams, frame.times):
        if distance is None:
            continue
        x, y, h = sensor_pose(pose_at(history, stamps, time))
        dx = x-ex + distance*math.cos(h+angle)
        dy = y-ey + distance*math.sin(h+angle)
        beams.append((math.atan2(dy, dx)-eh, math.hypot(dx, dy)))
    return beams


class SensorSuite:
    def __init__(self, world, index, options):
        mode = options["mode"]
        seed = (world.seed * 31 + world.match.round) * 2 + index
        now = world.elapsed / world.c["rate"]
        lidar = options["lidar"]
        self.lidar = LidarSensor(lidar, mode, random.Random(seed * 2), now)
        self.imu = ImuSensor(options["imu"], mode, random.Random(seed * 2 + 1))
        mount = (lidar["lidar_mount_x_mm"], lidar["lidar_mount_y_mm"],
                 math.radians(lidar["lidar_mount_yaw_deg"]))
        self.tracker = WallTracker(RectangleLocator(world.c["width"], world.c["height"], mount))
        self.mount = mount
        enc = options["encoder"]
        if any(not math.isfinite(v) or v < 0 for v in enc.values()):
            raise ValueError("Invalid encoder delay")
        self.encoder_delay = {"Ideal": 0., "Nominal": enc["latency_s"],
                              "Stress": enc["stress_latency_s"]}[mode]
        self.pending_imu = deque()
        self.pending_encoders = deque()
        self.last_imu_acquisition = now
        self.encoder = self.imu_sample = self.scan = None
        method = lidar.get("motion_compensation", "off")  # Old recorded profiles used raw beams.
        if method not in ("odometry", "off"):
            raise ValueError("Unknown lidar motion compensation")
        self.deskew = self.lidar.dense and method == "odometry"
        span = self.lidar.beam_dt * (len(self.lidar.angles)-1) if self.deskew else 0.
        self.history = deque(maxlen=math.ceil((self.lidar.latency+span)*world.c["rate"]) + 3)
        self.delivery_tick = None

    def update(self, world, index, dt, initial=False):
        c, tick = world.c, world.elapsed
        now = tick / c["rate"]
        car = world.cars[index]
        if initial or world.schedule.due("encoder_hz", tick):
            self.pending_encoders.append(EncoderSample.from_wheels(car, now, self.encoder_delay))
        while self.pending_encoders and self.pending_encoders[0].available_time <= now + 1e-12:
            packet = self.pending_encoders.popleft()
            if self.encoder is not None:
                delta = packet.travel_delta(self.encoder)
                period = packet.time - self.encoder.time
                world.counts[index] = tuple(a+b for a,b in zip(world.counts[index], delta))
                world.measured_speed[index] = tuple(
                    counts_to_travel(n, car.c["wheel"], car.c["counts"]) / period for n in delta)
            self.encoder = packet
        if initial or world.schedule.due("imu_hz", tick):
            packet = self.imu.sample(car, now, now - self.last_imu_acquisition)
            self.last_imu_acquisition = now
            self.pending_imu.append(packet)
        while self.pending_imu and self.pending_imu[0].available_time <= now + 1e-12:
            self.imu_sample = self.pending_imu.popleft()
            world.gyros[index] = self.imu_sample.yaw_rate
        estimate = world.estimates[index]
        estimate.update(world.counts[index], world.gyros[index], dt)
        self.history.append((now, estimate.pose))
        frame = self.lidar.update(world, car, now)
        if frame is None:
            return
        self.scan = ScanSample(frame.time, frame.available_time, frame.beams, frame.times)
        self.delivery_tick = tick
        # The locator needs the prediction at acquisition, even for delayed frames.
        history = list(self.history)
        stamps = [time for time, _ in history]
        prior = pose_at(history, stamps, frame.time)
        beams = deskew(frame, history, stamps, prior, self.mount) if self.deskew else frame.beams
        world.wall_fixes[index] = self.tracker.update(beams, frame.time, prior)
        world.scans[index] = frame.beams
        world.scan_poses[index] = frame.pose
        world.scan_tick = tick
