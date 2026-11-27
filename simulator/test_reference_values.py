"""Acceptance values for the current manual and dynamics reference tables."""
import math
import unittest

from robot_api import WheelSpeed
from sil_view import manual_pwm
from test_dynamics import drive, experiment
from wall_localization import RectangleLocator


class ReferenceTests(unittest.TestCase):
    def test_push_table(self):
        for mass,expected in ((0.6,(410.42,336.27,15.90)),(1.2,(408.64,328.20,21.81)),
                              (2.4,(405.08,309.24,35.66))):
            w = experiment(vehicle=dict(mass=mass),scene={"pieces":[[0,6,400,200]]}).world
            drive(w,1,(.55,.55))
            b = w.cars[0].body
            x,v,travel = b.position.x,b.velocity.length,w.pieces[0][2].position.x-400
            drive(w,1,(0,0))
            for actual,reference in zip((v,travel,b.position.x-x),expected):
                self.assertAlmostEqual(actual,reference,delta=.01)

    def test_stop_and_refinement_tables(self):
        for mode,expected in (("brake",21.88),("coast",668.78)):
            w = experiment(vehicle=dict(zero_mode=mode)).world
            drive(w,1,(.55,.55))
            x = w.cars[0].body.position.x
            drive(w,2,(0,0))
            self.assertAlmostEqual(w.cars[0].body.position.x-x,expected,delta=.01)
        for hz,expected in ((100,322.041),(200,321.979),(400,321.946)):
            w = experiment(hz=hz).world
            drive(w,.3,(.55,.55))
            drive(w,.2,(0,0))
            self.assertAlmostEqual(w.cars[0].body.position.x,expected,delta=.01)

    def test_closed_loop_step(self):
        for target in (120,180,300,400,-180):
            sim = experiment(scene={"poses":[[1500,500,0],[2500,1600,1]],"pieces":[]})
            sim.manual[0] = WheelSpeed(target,target)
            counts = []
            for step in range(600):
                sim.step()
                if step%2: counts.append(sim.world.counts[0][0])
                for wheel in sim.world.cars[0].speed:
                    self.assertLessEqual(abs(wheel),1.05*abs(target))
                if step >= 399 and step%2:
                    # 100 ms encoder window resolves sub-5% error at 120 mm/s.
                    speed = (counts[-1]-counts[-11])*math.pi*65/1320/.1
                    self.assertLessEqual(abs(speed-target),abs(target)*.05)

    def test_central_mirror_is_ambiguous(self):
        for x in (1500,1450,1425):
            w = experiment(scene={"poses":[[x,1000,.3],[2500,1700,1]],"pieces":[]}).world
            for yaw in (.3,.3+math.pi):
                fix = RectangleLocator(3000,2000).locate(w.scans[0],0,(x,1000,yaw))
                self.assertEqual(fix.status,"ambiguous_heading")
                self.assertIsNone(fix.pose)

    def test_keyboard_and_continuous_gain(self):
        for gain in (0,.137,.5,1):
            self.assertEqual(manual_pwm(0,1,gain).left_pwm,-gain)
            self.assertEqual(manual_pwm(0,1,gain).right_pwm,gain)
            self.assertEqual(manual_pwm(0,-1,gain).right_pwm,-gain)
            self.assertEqual(manual_pwm(1,1,gain).left_pwm,1-gain)
