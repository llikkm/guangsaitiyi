import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from experiment import sample
from robot_api import MotorCommand, WheelSpeed
from sil import execute, inputs
from sil_core import Schedule, Simulation, settings


class ClockAndIOTests(unittest.TestCase):
    def test_rate_phase_and_timestamps(self):
        options = settings()
        c = options["config"]
        self.assertEqual(c["rate"],200)
        schedule = Schedule(dict(c,scan_rate=12.5))
        self.assertEqual(sum(schedule.due("scan_rate",n) for n in range(1,12001)),750)
        sim = Simulation(options)
        scans = []
        for tick in range(1,601):
            sim.step()
            robot = sim.world.observation(0)
            self.assertAlmostEqual(robot.encoder_time,(tick//2)*0.01)
            self.assertEqual(robot.imu_time,robot.encoder_time)
            if sim.world.scan_tick == tick: scans.append(tick)
        self.assertEqual(len(scans),36)
        self.assertEqual(scans[:3],[17,34,50])
        self.assertEqual(scans[-1],600)

    def test_plugin_loading_and_observation_boundary(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            (folder/"custom.py").write_text("""
class Controller:
    def reset(self, config):
        assert set(config) == {"value"}
        self.value = config["value"]
        self.plans = []
        self.updates = []
    def plan(self, robot, dt):
        self.plans.append(dt)
    def update(self, robot, dt):
        assert not hasattr(robot, "world") and not hasattr(robot, "truth")
        assert isinstance(robot.read_lidar(), tuple)
        assert len(robot.read_encoders()) == 2
        assert isinstance(robot.read_imu(), float)
        self.updates.append(dt)
        return robot.set_pwm(self.value, self.value)
""",encoding="utf8")
            path = folder/"experiment.toml"
            path.write_text('[assumed]\ncontrol_hz=50\n[team_a]\ncontroller="custom.py"\n'
                            '[team_a.options]\nvalue=0.4\n',encoding="utf8")
            sim = Simulation(settings(path))
            for _ in range(200): sim.step()
            ctrl = sim.controllers[0]
            self.assertEqual(len(ctrl.plans),20)
            self.assertEqual(len(ctrl.updates),50)
            self.assertTrue(all(t == 0.02 for t in ctrl.updates))
            self.assertEqual(sim.pwm,[(0.4,0.4),(0,0)])
            self.assertNotEqual(sim.hashes[0],sim.hashes[1])
            with self.assertRaises(AttributeError):
                sim.world.observation(0).time = 1000

    def test_speed_loop_uses_encoder_feedback(self):
        nominal = Simulation(settings())
        changed = Simulation(settings())
        for sim in (nominal,changed): sim.step()
        # Inject an encoder-derived overspeed sample, with the same plant state.
        for sim in (nominal,changed): sim.step()
        changed.world.measured_speed[0] = (300,300)
        a,b = nominal.step(),changed.step()
        self.assertEqual(a["commands"],b["commands"])
        self.assertGreater(a["pwm"][0][0],b["pwm"][0][0])
        self.assertNotEqual(nominal.world.cars[0].speed[0],180)
        with self.assertRaises(ValueError): MotorCommand(1.01,0)
        with self.assertRaises(ValueError): WheelSpeed(math.nan,0)

    def test_scene_and_lidar(self):
        options = settings()
        options["scene"] = dict(poses=[[400,400,0],[2400,1500,1]],
                                pieces=[[1,6,650,400]],obstacles=[[900,400,100,160,180]])
        sim = Simulation(options)
        w = sim.world
        self.assertEqual(w.observation(0).estimated_pose,(400,400,0))
        self.assertEqual(len(w.pieces),1)
        self.assertAlmostEqual(w.observation(0).lidar[45][1],450)
        for _ in range(600):
            w.step([(1,1),(0,0)])
        self.assertLess(w.cars[0].body.position.x,850)
        self.assertGreater(w.pieces[0][2].position.x,650)
        options["scene"]["pieces"] = [[1,6,400,400]]
        with self.assertRaisesRegex(ValueError,"Overlapping"):
            Simulation(options)


class AcceptanceTests(unittest.TestCase):
    def test_straight_spin_stop(self):
        sim = Simulation(settings())
        b = sim.world.cars[0].body
        for _ in range(200): sim.step()
        self.assertGreater(b.position.x,350)
        self.assertAlmostEqual(b.position.y,200)
        self.assertAlmostEqual(b.angle,0)
        for _ in range(120): sim.step()
        center,angle = b.position,b.angle
        for _ in range(80): sim.step()
        self.assertGreater(b.angle-angle,0.75)
        self.assertLess((b.position-center).length,1)
        for _ in range(200): sim.step()
        self.assertLess(b.velocity.length,0.01)
        self.assertLess(abs(b.angular_velocity),0.001)
        self.assertEqual(tuple(sim.world.cars[1].body.position),(2800,1800))

    def test_native_and_replay_match_all_steps(self):
        options = settings()
        options["scene"] = dict(pieces=[[0,0,410,200]],obstacles=[[950,200,100,150,100]])
        trace = []
        def snapshot(w):
            return sample(w,[None,None]),[(tuple(c.body.velocity),c.body.angular_velocity) for c in w.cars]
        log = io.StringIO()
        execute(Simulation(options),650,stream=log,on_step=lambda w: trace.append(snapshot(w)))
        for fps in (30,60):
            options["fps"] = fps
            other = Simulation(options)
            def equal(w):
                self.assertEqual(snapshot(w),trace[w.elapsed-1])
            execute(other,650,gui=True,speed=16,on_step=equal)
            self.assertEqual(other.world.elapsed,len(trace))
        log.seek(0)
        meta = json.loads(log.readline())
        # Playback must work even if the original controller file is unavailable.
        meta["options"]["teams"][0]["controller"] = "missing.py"
        replay = Simulation(meta["options"],meta["seed"],replay=True)
        execute(replay,0,recorded=inputs(log),on_step=equal)
        self.assertEqual(replay.world.elapsed,len(trace))

    def test_truncated_log_rejected(self):
        with self.assertRaisesRegex(ValueError,"Incomplete"):
            execute(Simulation(settings(),replay=True),0,recorded=inputs(io.StringIO("")))
