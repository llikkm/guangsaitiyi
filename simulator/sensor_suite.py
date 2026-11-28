"""V2.2 acquisition, delivery and odometry, one instance per vehicle."""
from collections import deque
import math
import random

from encoder_sensor import EncoderSample
from imu_sensor import ImuSensor, imu_defaults
from lidar_sensor import LidarSensor, lidar_defaults
from robot_api import ScanSample
from units import counts_to_travel
from wall_localization import RectangleLocator, WallTracker


def sensor_options(data):
    mode = data.get("mode", "Ideal")
    if mode not in ("Ideal", "Nominal", "Stress"):
        raise ValueError("Sensor mode must be Ideal, Nominal or Stress")
    return dict(mode=mode, imu=imu_defaults() | data.get("imu", {}),
                lidar=lidar_defaults() | data.get("lidar", {}),
                encoder={"latency_s": 0., "stress_latency_s": 0.02} | data.get("encoder", {}))


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
        self.history = deque(maxlen=math.ceil(self.lidar.latency * world.c["rate"]) + 3)
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
        while len(self.history) >= 2 and self.history[1][0] <= frame.time:
            self.history.popleft()
        t0, prior = self.history[0]
        if len(self.history) > 1 and t0 < frame.time:
            t1, following = self.history[1]
            f = (frame.time - t0) / (t1 - t0)
            x,y,h = prior
            prior = (x + f*(following[0]-x), y + f*(following[1]-y),
                     h + f*math.remainder(following[2]-h, math.tau))
        world.wall_fixes[index] = self.tracker.update(frame.beams, frame.time, prior)
        world.scans[index] = frame.beams
        world.scan_poses[index] = frame.pose
        world.scan_tick = tick
