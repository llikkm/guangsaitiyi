import math
import unittest

from sil_core import Simulation, settings
from wall_localization import RectangleLocator, pavo_scan


class WallLocalizationTests(unittest.TestCase):
    def test_two_cars_motion_and_scan_timestamps(self):
        sim = Simulation(settings())
        checked = 0
        for _ in range(800):
            sim.step()
            for i in range(2):
                w = sim.world
                robot = w.observation(i)
                fix = robot.wall_fix
                self.assertEqual(fix.time, robot.lidar_time)
                if w.elapsed != w.scan_tick:
                    continue
                checked += 1
                x,y,h = w.scan_poses[i]
                self.assertEqual(fix.status, "ok")
                self.assertLess(math.dist(fix.pose[:2],(x,y)), 8)
                self.assertLess(abs(math.remainder(fix.pose[2]-h, math.tau)), .012)
                self.assertLess(math.dist(fix.wall_distances,(x,3000-x,y,2000-y)), 12)
        self.assertEqual(checked,96)

    def test_prior_offset_mount_occlusion_and_pavo_units(self):
        options = settings()
        options["scene"] = dict(poses=[[720,610,.41],[2400,1500,1]],
                                obstacles=[[1100,800,120,180,150]],pieces=[])
        robot = Simulation(options).world.observation(0)
        # The scan origin in this fixture is the simulated sensor at (720,610).
        mount = (80,-30,.21)
        h = .41-mount[2]
        x = 720-math.cos(h)*80-math.sin(h)*30
        y = 610-math.sin(h)*80+math.cos(h)*30
        raw = [(180+math.degrees(a),r/1000 if r else None) for a,r in robot.lidar]
        fix = RectangleLocator(3000,2000,mount).locate(pavo_scan(raw),0,(x+65,y-55,h+.045))
        self.assertEqual(fix.status, "ok")
        self.assertLess(math.dist(fix.pose[:2],(x,y)), 8)
        self.assertLess(abs(fix.pose[2]-h), .01)
        self.assertAlmostEqual(pavo_scan([(90, 1)])[0][0], -math.pi/2)

    def test_insufficient_geometry(self):
        locator = RectangleLocator(3000,2000)
        for beams in [[], [(0,None)]*40,
                      [(a/100, 1000/math.cos(a/100)) for a in range(-20,21)]]:
            fix = locator.locate(beams,1,(2000,1000,0))
            self.assertIsNone(fix.pose)
            self.assertIsNone(fix.wall_distances)
