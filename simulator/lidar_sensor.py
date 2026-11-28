"""Rotating 2-D scanner. Distances are mm and timestamps are simulation seconds."""
from collections import deque
from dataclasses import dataclass
import math

import pymunk as pm


@dataclass(frozen=True)
class ScanFrame:
    beams: tuple                 # (sensor-local angle rad, distance mm or None)
    times: tuple                 # acquisition time of each beam
    time: float                  # last active beam
    available_time: float        # transport complete
    pose: tuple | None = None    # optional simulator-only reference body pose


def lidar_defaults():
    return dict(
        lidar_model="fast", lidar_fast_rays=91, lidar_fast_fov_deg=270.,
        lidar_fast_hz=12., lidar_fast_min_mm=0., lidar_fast_max_mm=4000.,
        lidar_dense_step_deg=.08, lidar_dense_fov_deg=270., lidar_dense_hz=10.06,
        lidar_min_mm=30., lidar_max_mm=10000., lidar_height_mm=55.,
        lidar_mount_x_mm=0., lidar_mount_y_mm=0., lidar_mount_yaw_deg=0.,
        lidar_noise_mm=2.2, lidar_bias_mm=0., lidar_quantum_mm=2.,
        lidar_angle_error_deg=0., lidar_dropout=0.01, lidar_frame_dropout=0.,
        lidar_outlier=0.001, lidar_latency_s=.012, lidar_blind_sectors_deg=[],
        lidar_clutter=[], lidar_stress_factor=3.,
        lidar_stress_blind_deg=[[-45., -35.]], lidar_stress_frame_loss=0.03)


class LidarSensor:
    def __init__(self, options, mode, rng, start_time):
        c = self.c = lidar_defaults() | options
        self.rng = rng
        self.start = self.previous_time = float(start_time)
        self.previous_pose = None
        self.frame_index = self.beam_index = 0
        self.pending = deque()
        self.beams, self.times = [], []
        mode = mode.lower()
        if mode not in ("ideal", "nominal", "stress"):
            raise ValueError("lidar mode must be Ideal, Nominal or Stress")
        for key, default in lidar_defaults().items():
            if isinstance(default, (int, float)) and not math.isfinite(c[key]):
                raise ValueError(f"Invalid {key}")
        self.factor = {"ideal": 0., "nominal": 1., "stress": c["lidar_stress_factor"]}[mode]
        if c["lidar_stress_factor"] < 1:
            raise ValueError("lidar_stress_factor must be at least 1")
        for key in ("lidar_dropout", "lidar_outlier", "lidar_frame_dropout", "lidar_stress_frame_loss"):
            if not 0 <= c[key] <= 1:
                raise ValueError(f"{key} must be a probability")
        for key in ("lidar_latency_s", "lidar_height_mm", "lidar_noise_mm", "lidar_quantum_mm",
                    "lidar_angle_error_deg"):
            if c[key] < 0:
                raise ValueError(f"{key} must be nonnegative")
        self.dense = c["lidar_model"] == "dense"
        if c["lidar_model"] not in ("fast", "dense"):
            raise ValueError("lidar_model must be fast or dense")
        prefix = "lidar_dense_" if self.dense else "lidar_fast_"
        fov, hz = c[prefix+"fov_deg"], c[prefix+"hz"]
        if not 0 < fov <= 360 or hz <= 0:
            raise ValueError("Invalid lidar field of view or frequency")
        self.period = 1 / hz
        if self.dense:
            step = c["lidar_dense_step_deg"]
            if not 0 < step <= fov:
                raise ValueError("Invalid lidar angular step")
            count = math.ceil(fov/step - 1e-10)
            # Symmetric bin centres. Excludes the inactive rear sector.
            offsets = [(i-(count-1)/2)*step for i in range(count)]
            self.beam_dt = self.period*step/360
        else:
            count = c["lidar_fast_rays"]
            if type(count) is not int or count < 1:
                raise ValueError("lidar_fast_rays must be a positive integer")
            offsets = [fov*(i/(count-1)-.5) for i in range(count)] if count > 1 else [0.]
            self.beam_dt = 0.
        self.angles = tuple(map(math.radians, offsets))
        prefix = "lidar_" if self.dense else "lidar_fast_"
        self.lo, self.hi = c[prefix+"min_mm"], c[prefix+"max_mm"]
        if not 0 <= self.lo < self.hi:
            raise ValueError("Invalid lidar range gates")
        self.blind = set()
        sectors = list(c["lidar_blind_sectors_deg"])
        self.frame_loss = min(1., c["lidar_frame_dropout"] * self.factor)
        if mode == "stress":
            sectors.extend(c["lidar_stress_blind_deg"])
            self.frame_loss = max(self.frame_loss, c["lidar_stress_frame_loss"])
        for lo, hi in sectors:
            if not -180 <= lo <= hi <= 180:
                raise ValueError("Blind sector must be [start,end] within [-180,180]")
            self.blind.update(i for i, angle in enumerate(offsets) if lo <= angle <= hi)
        self.clutter = []
        for x, y, radius, height in c["lidar_clutter"]:
            if not all(map(math.isfinite, (x,y,radius,height))) or radius <= 0 or height < 0:
                raise ValueError("Invalid lidar clutter cylinder [x,y,radius,height]")
            if height >= c["lidar_height_mm"]:
                self.clutter.append((x,y,radius))
        self.mount = c["lidar_mount_x_mm"], c["lidar_mount_y_mm"]
        self.yaw = math.radians(c["lidar_mount_yaw_deg"])
        self.angle_sigma = math.radians(c["lidar_angle_error_deg"])*self.factor
        self.range_sigma = c["lidar_noise_mm"]*self.factor
        self.latency = c["lidar_latency_s"]*self.factor

    def _chance(self, probability):
        return probability > 0 and self.rng.random() < probability*self.factor

    def _range(self, world, query, pose, i):
        if i in self.blind:
            return None
        c = self.c
        if self.factor and self._chance(c["lidar_dropout"]):
            return None
        x, y, h = pose
        mx, my = self.mount
        co, si = math.cos(h), math.sin(h)
        x, y = x+mx*co-my*si, y+mx*si+my*co
        direction = h+self.yaw+self.angles[i]
        if self.angle_sigma:
            direction += self.rng.gauss(0., self.angle_sigma)
        dx, dy = math.cos(direction), math.sin(direction)
        hits = world.space.segment_query((x,y), (x+dx*self.hi,y+dy*self.hi), 0, query)
        distance = min((hit.alpha*self.hi for hit in hits
                        if world.heights[hit.shape] >= c["lidar_height_mm"]), default=math.inf)
        for cx, cy, radius in self.clutter:
            rx, ry = cx-x, cy-y
            along = rx*dx + ry*dy
            disc = radius**2 - (rx*dy-ry*dx)**2
            if disc >= 0 and along+math.sqrt(disc) >= 0:
                distance = min(distance, max(0., along-math.sqrt(disc)))
        if not self.lo <= distance <= self.hi:
            return None
        if self.factor:
            if self._chance(c["lidar_outlier"]):
                distance = self.rng.uniform(self.lo, self.hi)
            else:
                distance += c["lidar_bias_mm"] * self.factor
                if self.range_sigma:
                    distance += self.rng.gauss(0., self.range_sigma)
            quantum = c["lidar_quantum_mm"]
            if quantum:
                distance = round(distance/quantum)*quantum
        return distance if self.lo <= distance <= self.hi else None

    def update(self, world, car, now):
        """Acquire due beams; return the newest delivered frame once."""
        if not math.isfinite(now) or now < self.previous_time:
            raise ValueError("Sensor time must be finite and monotone")
        current = (*car.body.position, car.body.angle)
        previous = self.previous_pose or current
        interval = now - self.previous_time
        query = pm.ShapeFilter(group=car.shape.filter.group)
        while True:
            start = self.start + self.frame_index*self.period
            sample = start + self.beam_index*self.beam_dt
            if sample > now + 1e-12:
                break
            fraction = max(0., min(1., (sample-self.previous_time)/interval)) if interval else 1.
            pose = tuple(a+(b-a)*fraction for a,b in zip(previous,current))
            end = self.beam_index+1 if self.dense else len(self.angles)
            for i in range(self.beam_index, end):
                self.beams.append((self.angles[i], self._range(world,query,pose,i)))
                self.times.append(sample)
            self.beam_index = end
            if end == len(self.angles):
                if self.frame_loss == 0 or self.rng.random() >= self.frame_loss:
                    self.pending.append(ScanFrame(tuple(self.beams), tuple(self.times), sample,
                                                  sample+self.latency, pose))
                self.beams, self.times = [], []
                self.frame_index += 1
                self.beam_index = 0
        self.previous_pose, self.previous_time = current, now
        latest = None
        while self.pending and self.pending[0].available_time <= now+1e-12:
            latest = self.pending.popleft()
        return latest
