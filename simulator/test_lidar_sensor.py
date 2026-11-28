import dataclasses
import math
import random
import unittest
from types import SimpleNamespace

import pymunk as pm

from lidar_sensor import LidarSensor, ScanFrame, lidar_defaults


class LidarTests(unittest.TestCase):
    def setUp(self):
        self.world = SimpleNamespace(space=pm.Space(), heights={})
        body = pm.Body(1, 1)
        shape = pm.Circle(body, 3)
        shape.filter = pm.ShapeFilter(group=1)
        self.car = SimpleNamespace(body=body, shape=shape)
        self.world.space.add(body, shape)
        self.world.heights[shape] = 200
        self.wall(200, 80)

    def wall(self, x, height):
        wall = pm.Segment(self.world.space.static_body, (x,-1000), (x,1000), 0)
        self.world.space.add(wall)
        self.world.heights[wall] = height

    def sensor(self, mode="ideal", **opts):
        return LidarSensor(dict(lidar_fast_rays=1) | opts, mode, random.Random(99), 0.)

    def tick(self, sensor, t):
        return sensor.update(self.world, self.car, t)

    def test_fast_snapshot_height_occlusion_mount_and_clutter(self):
        self.wall(75, 20)
        self.wall(130, 55)
        sensor = self.sensor(lidar_mount_x_mm=10)
        self.assertEqual(self.tick(sensor,0).beams, ((0.,120.),))
        self.assertIsNone(self.tick(sensor,.01))
        self.assertEqual(self.tick(self.sensor(lidar_height_mm=10),0).beams, ((0.,75.),))
        sensor = self.sensor(lidar_clutter=[[100,0,10,65]])
        self.assertEqual(self.tick(sensor,0).beams, ((0.,90.),))
        sensor = self.sensor(lidar_mount_yaw_deg=180)
        self.assertIsNone(self.tick(sensor,0).beams[0][1])

    def test_range_gates_blind_and_self_filter(self):
        for options in [dict(lidar_fast_min_mm=201),dict(lidar_fast_max_mm=199),
                        dict(lidar_blind_sectors_deg=[[-2,2]])]:
            self.assertIsNone(self.tick(self.sensor(**options),0).beams[0][1])
        frame = self.tick(self.sensor(),0)
        self.assertEqual(frame.beams[0][1],200)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            frame.time = 90
        real = ScanFrame(frame.beams,frame.times,0,.02)
        self.assertIsNone(real.pose)

    def test_progressive_translation_rotation_and_delivery(self):
        opts = dict(lidar_model="dense",lidar_dense_fov_deg=90,lidar_dense_step_deg=30,
                    lidar_dense_hz=1,lidar_latency_s=.05,lidar_dropout=0,lidar_outlier=0,
                    lidar_noise_mm=0,lidar_quantum_mm=0)
        sensor = self.sensor("nominal", **opts)
        self.assertIsNone(self.tick(sensor,0))
        for tick in range(1,23):
            t = tick/100
            self.car.body.position = 60*t,0
            self.car.body.angle = t
            frame = self.tick(sensor,t)
            if tick < 22:
                self.assertIsNone(frame)
        for i, (a,r) in enumerate(frame.beams):
            stamp = frame.times[i]
            self.assertAlmostEqual(stamp,i/12)
            self.assertAlmostEqual(r,(200-60*stamp)/math.cos(a+stamp))
        self.assertAlmostEqual(frame.pose[0],10.)
        self.assertAlmostEqual(frame.time,1/6)
        self.assertAlmostEqual(frame.available_time,1/6+.05)
        self.assertIsNone(self.tick(sensor,.221))

    def test_dense_measured_geometry_and_rate(self):
        sensor = self.sensor(lidar_model="dense")
        seen = []
        for tick in range(201):
            frame = self.tick(sensor,tick/200)
            if frame: seen.append(frame)
        self.assertEqual(len(seen),10)
        frame = seen[-1]
        self.assertEqual(len(frame.beams),3375)
        self.assertAlmostEqual(math.degrees(frame.beams[1][0]-frame.beams[0][0]),.08)
        self.assertAlmostEqual(frame.times[-1]-frame.times[0],3374/(4500*10.06))
        self.assertAlmostEqual(frame.time-seen[-2].time,1/10.06)

    def test_modes_seed_dropouts_and_outliers(self):
        opts = dict(lidar_latency_s=0,lidar_noise_mm=0,lidar_outlier=0,lidar_dropout=0,
                    lidar_quantum_mm=0,lidar_bias_mm=5)
        for mode,d in [("ideal",200),("nominal",205),("stress",215)]:
            self.assertAlmostEqual(self.tick(self.sensor(mode,**opts),0).beams[0][1],d)
        opts["lidar_frame_dropout"] = 1
        self.assertIsNone(self.tick(self.sensor("nominal",**opts),0))
        opts.update(lidar_frame_dropout=0,lidar_dropout=1)
        self.assertIsNone(self.tick(self.sensor("nominal",**opts),0).beams[0][1])
        opts.update(lidar_dropout=0,lidar_outlier=1)
        runs = [self.tick(self.sensor("stress",**opts),0) for _ in range(2)]
        self.assertEqual(*runs)
        self.assertNotEqual(runs[0].beams[0][1],215)

    def test_defaults_independent_and_invalid_inputs(self):
        lidar_defaults()["lidar_clutter"].append([1,1,1,1])
        self.assertEqual(lidar_defaults()["lidar_clutter"],[])
        for opts in [dict(lidar_dropout=1.1),dict(lidar_fast_hz=0),dict(lidar_model="other"),
                     dict(lidar_model="dense",lidar_dense_step_deg=0)]:
            with self.assertRaises(ValueError):
                self.sensor(**opts)


if __name__ == "__main__":
    unittest.main()
