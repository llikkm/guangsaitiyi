"""V2.2 controller, firmware counter, delayed delivery and replay regressions."""
import io
import json
import math
import unittest
from unittest.mock import patch

from control import navigate
from encoder_sensor import EncoderSample, encoder_delta
from robot_api import MotorCommand, ScanSample, WheelSpeed
from sil import execute, inputs
from sil_core import SENSOR_REFERENCE, Simulation, settings


class SensorIntegrationTests(unittest.TestCase):
    def test_window_displays_raw_estimate_alongside_navigation(self):
        import pygame as pg
        from sil_view import watch

        options = settings(SENSOR_REFERENCE)
        options["scene"] = dict(poses=[[2600, 500, 0], [400, 1700, math.pi]], pieces=[])
        sim = Simulation(options)
        sim.manual[0] = MotorCommand(.8, .8)
        for _ in range(800):
            sim.step()
        obs = sim.world.observation(0)
        self.assertGreater(obs.estimated_pose[0] - obs.navigation_pose[0], 1200)

        rendered = []
        native_font = pg.font.Font

        class RecordingFont:
            def __init__(self, *args):
                self.font = native_font(*args)

            def render(self, text, *args):
                rendered.append(text)
                return self.font.render(text, *args)

        with patch.object(pg.font, "Font", RecordingFont):
            watch(sim, lambda: False, speed=16)
        x, y, h = obs.estimated_pose
        self.assertIn(f"Estimated xy  {x:.1f}, {y:.1f}", rendered)
        self.assertIn(f"Estimated yaw  {h:.3f} rad", rendered)
        nx, ny, _ = obs.navigation_pose
        self.assertIn(f"Nav xy mm  {nx:.1f}, {ny:.1f}", rendered)

    def test_click_navigation_after_wall_slip(self):
        options = settings(SENSOR_REFERENCE)
        options["scene"] = dict(poses=[[2600, 500, 0], [400, 1700, math.pi]], pieces=[])
        sim = Simulation(options)
        w = sim.world
        sim.manual[0] = MotorCommand(.8, .8)
        for _ in range(800):
            sim.step()
        obs = w.observation(0)
        self.assertGreater(obs.estimated_pose[0] - w.cars[0].body.position.x, 1200)
        self.assertLess(math.dist(obs.navigation_pose[:2], w.cars[0].body.position), 100)
        target = (2200, 800)
        for _ in range(2400):
            speeds, arrived = navigate(w.observation(0).navigation_pose, target, w.cars[0].c)
            sim.manual[0] = WheelSpeed(*speeds)
            sim.step()
            if arrived:
                break
        self.assertTrue(arrived)
        self.assertLess(math.dist(w.cars[0].body.position, target), w.c["arrival"] + 10)

    def test_real_scan_time_and_angle_adapter(self):
        scan = ScanSample.from_pavo([(90, 1.5), (180, 0)], [.09, .10], .12)
        self.assertAlmostEqual(scan.beams[0][0], -1.57079632679)
        self.assertEqual(scan.beams[0][1], 1500.)
        self.assertEqual(scan.beams[1], (0., None))
        self.assertEqual(scan.time, .10)
        with self.assertRaises(ValueError):
            ScanSample.from_pavo([(180, 1)], [.10], .08)

    def test_timer_wrap_and_reverse(self):
        a = EncoderSample(0., 0., 65534, 1)
        b = EncoderSample(.01, .01, 2, 65533)
        self.assertEqual(b.travel_delta(a), (4, 4))
        self.assertEqual(a.travel_delta(b), (-4, -4))
        self.assertEqual(encoder_delta(0, 65535), 1)

    def test_dense_delivery_and_legacy_readers(self):
        options = settings(SENSOR_REFERENCE)
        options["sensors"]["mode"] = "Nominal"
        options["sensors"]["lidar"].update(lidar_model="dense", lidar_latency_s=.08,
                                           lidar_mount_x_mm=30.)
        sim = Simulation(options)
        world = sim.world
        self.assertIsNone(world.observation(0).lidar_sample)
        for _ in range(80):
            sim.step()
        obs = world.observation(0)
        frame = obs.lidar_sample
        self.assertGreater(len(frame.beams), 3000)
        self.assertLess(frame.times[0], frame.time)
        self.assertAlmostEqual(frame.available_time-frame.time, .08)
        self.assertLessEqual(frame.available_time, obs.time)
        self.assertEqual(obs.wall_fix.time, frame.time)
        self.assertEqual(obs.lidar, frame.beams)
        self.assertFalse(hasattr(frame, "pose"))
        self.assertEqual(obs.read_imu(), obs.imu_sample.yaw_rate)
        self.assertEqual(obs.encoder_counts[1] % 65536, (-obs.encoder_sample.tim8) % 65536)
        self.assertLess(world.observation(1).imu_sample.time, obs.time)
        self.assertLessEqual(obs.imu_sample.available_time, obs.time + 1e-12)

    def test_stress_replay_and_reset(self):
        options = settings(SENSOR_REFERENCE)
        options["sensors"]["mode"] = "Stress"
        sim = Simulation(options)
        stream = io.StringIO()
        execute(sim, 90, stream=stream)
        stream.seek(0)
        meta = json.loads(next(stream))
        self.assertEqual(meta["software"], "2.2")
        replay = Simulation(meta["options"], meta["seed"], replay=True)
        execute(replay, 90, recorded=inputs(stream))
        self.assertEqual(sim.world.observation(0), replay.world.observation(0))
        sim.world.reset_cars()
        self.assertIsNone(sim.world.observation(0).imu_sample)
        self.assertEqual(sim.world.observation(0).encoder_counts, (0, 0))
        self.assertIsNone(sim.world.observation(0).lidar_sample)
