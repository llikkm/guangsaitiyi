import io
import json
import math
import unittest
from unittest.mock import patch

from experiment import sample
from sil import execute, inputs
from sil_core import DYNAMIC_REFERENCE, Simulation, settings
from units import ENGINE_FORCE_PER_N, ENGINE_MOMENT_PER_SI


def experiment(*, vehicle=None, right=None, scene=None, hz=200, config=None):
    o = settings(DYNAMIC_REFERENCE)
    o["config"].update(rate=hz,motor_hz=hz)
    o["config"].update(config or {})
    o["scene"] = {"pieces":[]} if scene is None else scene
    o["teams"][0].update(vehicle=vehicle or {},right=right or {})
    return Simulation(o)


def drive(w, seconds, pwm):
    for _ in range(round(w.c["rate"]*seconds)):
        w.step([pwm,(0,0)])


class AnalyticTests(unittest.TestCase):
    def test_force_mass_torque_and_units(self):
        self.assertEqual(ENGINE_FORCE_PER_N,1000)
        self.assertEqual(ENGINE_MOMENT_PER_SI,1_000_000)
        for mass in (0.6,1.2,2.4):
            w = experiment(vehicle=dict(mass=mass,length=260,breadth=220,tire_mu=0)).world
            b = w.cars[0].body
            inertia = mass*(0.26**2+0.22**2)/12
            self.assertAlmostEqual(b.moment/ENGINE_MOMENT_PER_SI,inertia)
            for _ in range(100):
                b.apply_force_at_world_point((2*ENGINE_FORCE_PER_N,0),b.position)
                b.torque = 0.02*ENGINE_MOMENT_PER_SI
                w.step([(0,0)]*2)
            self.assertAlmostEqual(b.velocity.x/1000,2*0.5/mass,places=8)
            self.assertAlmostEqual(b.angular_velocity,0.02*0.5/inertia,places=8)
            distance = 0.5*(2/mass)*0.5**2
            self.assertLessEqual(abs((b.position.x-200)/1000/distance-1),0.010000001)

    def test_parallel_axis_attachment_and_independent_geometry(self):
        w = experiment(vehicle=dict(mass=1.,bumper_mass=0.2,length=240,breadth=200,
                                    inertia_si=0.01,cg_x=-12,cg_y=6,counts=800,wheel=72)).world
        car = w.cars[0]
        center = (10,5)
        self.assertAlmostEqual(car.body.mass,1.2)
        self.assertEqual(tuple(round(x,8) for x in car.body.center_of_gravity),center)
        expected = 10000+22**2+1+0.2*(200**2/12+110**2+25)
        self.assertAlmostEqual(car.body.moment,expected)
        self.assertNotEqual(car.c["wheel"],w.cars[1].c["wheel"])
        drive(w,0.5,(0.5,0.5))
        self.assertEqual(w.counts[0],tuple(round(a.angle/math.tau*800) for a in car.wheels))
        self.assertEqual(w.match.c["mass"],car.body.mass)
        with self.assertRaises(ValueError): experiment(vehicle=dict(mass=2.4,bumper_mass=0.2))
        with self.assertRaises(ValueError): experiment(vehicle=dict(length=301))


class MotorTests(unittest.TestCase):
    def test_electrical_limits_deadzone_and_no_load_speed(self):
        w = experiment(vehicle=dict(tire_mu=0,rolling_mu=0)).world
        wheel = w.cars[0].wheels[0]
        drive(w,0.1,(0.04,0.04))
        self.assertEqual(wheel.angle,0)
        drive(w,0.005,(1,1))
        self.assertAlmostEqual(wheel.current,1.5)
        self.assertAlmostEqual(wheel.torque,0.016*1.5*20*0.82)
        drive(w,1,(1,1))
        self.assertAlmostEqual(wheel.omega,7.4/(0.016*20),places=6)
        self.assertAlmostEqual(wheel.current,0,places=6)
        drive(w,1,(-1,-1))
        self.assertLess(wheel.omega,-20)

    def test_asymmetry_and_blocked_slip(self):
        w = experiment(right={"resistance":8}).world
        drive(w,1,(0.55,0.55))
        self.assertLess(w.cars[0].body.angle,-0.1)
        scene = dict(pieces=[],obstacles=[[500,200,60,300,150]])
        w = experiment(scene=scene).world
        drive(w,1,(0.55,0.55))
        car = w.cars[0]
        self.assertLess(car.body.velocity.length,1)
        self.assertGreater(car.wheels[0].current,0.4)
        self.assertGreater(car.wheels[0].slip,0.1)
        self.assertGreater(w.estimates[0].pose[0]-car.body.position.x,100)

    def test_near_locked_rotor_and_rated_point(self):
        w = experiment(vehicle=dict(tire_mu=2.,long_stiffness=5000),
                       scene=dict(pieces=[],obstacles=[[500,200,60,300,150]]),
                       right={"spec_mode":"rated"}).world
        motor = w.cars[0].motors[1]
        self.assertAlmostEqual(motor["kt"]*motor["rated_a"],motor["rated_nm"])
        self.assertAlmostEqual(motor["ke"]*motor["rated_rpm"]*math.tau/60
                               +motor["rated_a"]*motor["resistance"],motor["rated_v"])
        drive(w,1,(0.55,0.55))
        wheel = w.cars[0].wheels[0]
        self.assertLess(wheel.omega,0.1)
        self.assertAlmostEqual(wheel.current,0.55*7.4/4,delta=0.01)

    def test_same_initial_speed_braking(self):
        stopping = []
        for mass in (0.6,1.2,2.4):
            w = experiment(vehicle={"mass":mass}).world
            car = w.cars[0]
            car.body.velocity = (400,0)
            for wheel in car.wheels: wheel.omega = 0.4/0.0325
            drive(w,1,(0,0))
            self.assertLess(car.body.velocity.length,0.1)
            stopping.append(car.body.position.x-200)
        self.assertLess(stopping[0],stopping[1])
        self.assertLess(stopping[1],stopping[2])

    def test_adhesion_and_lateral_force_circle(self):
        w = experiment(vehicle=dict(tire_mu=0.04)).world
        drive(w,1,(0.55,0.55))
        car = w.cars[0]
        self.assertGreater(w.estimates[0].pose[0]-car.body.position.x,150)
        car.body.velocity = (200,300)  # prescribed lateral perturbation
        w.step([(1,1),(0,0)])
        for wheel in car.wheels:
            self.assertLess(wheel.lateral_n,0)
            self.assertLessEqual(math.hypot(wheel.longitudinal_n,wheel.lateral_n),0.04*1.2*9.81/2+1e-10)

    def test_coast_energy_and_brake(self):
        distances = {}
        for mode in ("brake","coast"):
            w = experiment(vehicle={"zero_mode":mode}).world
            drive(w,1,(0.55,0.55))
            car = w.cars[0]
            x = car.body.position.x
            previous = math.inf
            for _ in range(400):
                w.step([(0,0)]*2)
                energy = car.body.kinetic_energy/(2*ENGINE_MOMENT_PER_SI)
                energy += sum(0.5*m["wheel_inertia"]*wheel.omega**2 for m,wheel in zip(car.motors,car.wheels))
                self.assertLessEqual(energy,previous+1e-12)
                previous = energy
                if mode == "coast": self.assertEqual(car.wheels[0].current,0)
            distances[mode] = car.body.position.x-x
        self.assertTrue(0 < distances["brake"] < distances["coast"])

    def test_step_convergence(self):
        estimates = []
        for hz in (100,200,400):
            w = experiment(hz=hz).world
            drive(w,0.3,(0.55,0.55))
            drive(w,0.2,(0,0))
            estimates.append(w.cars[0].body.position.x)
        self.assertLess(abs(estimates[2]-estimates[1]),abs(estimates[1]-estimates[0]))
        self.assertLess(abs(estimates[2]-estimates[1]),1)

    def test_mass_turn_braking_traction_and_pushing(self):
        rows = []
        for mass in (0.6,1.2,2.4):
            w = experiment(vehicle={"mass":mass},scene={"pieces":[[0,6,400,200]]}).world
            car = w.cars[0]
            drive(w,1,(0.55,0.55))
            piece_x = w.pieces[0][2].position.x
            self.assertGreater(piece_x,400)
            x = car.body.position.x
            drive(w,1,(0,0))
            stopping = car.body.position.x-x
            w = experiment(vehicle=dict(mass=mass,tire_mu=0.04)).world
            drive(w,0.05,(-1,1))
            traction = abs(w.cars[0].wheels[0].longitudinal_n)
            w = experiment(vehicle=dict(mass=mass)).world
            drive(w,0.1,(-0.4,0.4))
            rows.append((piece_x,stopping,traction,w.cars[0].body.angular_velocity))
        for light,heavy in zip(rows,rows[1:]):
            self.assertGreater(light[0],heavy[0])
            self.assertLess(light[1],heavy[1])
            self.assertLess(light[2],heavy[2])
            self.assertGreater(light[3],heavy[3])

    def test_chess_contact_momentum_and_floor_friction(self):
        for mu in (0,0.2):
            w = experiment(scene={"pieces":[[0,0,600,500],[1,0,670,500]]},config={"piece_mu":mu}).world
            a,b = [p[2] for p in w.pieces]
            a.velocity = (300,0)
            drive(w,0.15,(0,0))
            self.assertGreater(b.position.x,670)
            if mu == 0: self.assertAlmostEqual(a.velocity.x+b.velocity.x,300,places=6)
            else: self.assertLess(a.velocity.length+b.velocity.length,100)


class WorkbenchTests(unittest.TestCase):
    def test_panel_and_native_manual_replay(self):
        import tkinter as tk
        import pygame as pg
        from studio import Workbench
        root = tk.Tk()
        try:
            ui = Workbench(root)
            root.update()
            self.assertEqual(len(ui.book.tabs()),7)
            ui.surface()
            self.assertEqual(ui.collect()["config"]["piece_mu"],0.05)
            for path,_,var,_ in ui.fields:
                if path == ("teams",1,"vehicle","mass"): var.set("2.1")
            o = ui.collect()
            self.assertEqual(o["teams"][1]["vehicle"]["mass"],2.1)
            for key in range(4,10): self.assertTrue(root.bind(f"<F{key}>"))
        finally: root.destroy()
        sim = Simulation(o)
        log = io.StringIO()
        flip = pg.display.flip
        def present():
            flip()
            pg.event.post(pg.event.Event(pg.KEYDOWN,key=pg.K_END))
        with patch.object(pg.display,"flip",side_effect=present):
            execute(sim,80,stream=log,gui=True,speed=4)
        self.assertIn(0,sim.manual)
        log.seek(0)
        meta = json.loads(log.readline())
        replay = Simulation(meta["options"],meta["seed"],replay=True)
        execute(replay,0,recorded=inputs(log))
        self.assertEqual(sample(sim.world,[None]*2),sample(replay.world,[None]*2))
