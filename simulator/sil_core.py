"""Deterministic software-in-the-loop core. No GUI imports."""
import copy
from fractions import Fraction
import hashlib
import importlib.util
import math
from pathlib import Path
import tomllib

import pymunk as pm
from control import clamp
from physics import World
from wall_localization import RectangleLocator
from robot_api import MotorCommand, Observation, WheelSpeed
from rules import config
from units import counts_to_travel, turns_to_counts
from dynamics import DynamicCar, vehicle_config


REFERENCE = Path(__file__).with_name("v2.toml")
DYNAMIC_REFERENCE = REFERENCE.with_name("v21.toml")
TASKS = ("motor_hz", "pid_hz", "encoder_hz", "imu_hz", "control_hz",
         "navigation_hz", "scan_rate")


def settings(path=REFERENCE):
    path = Path(path).resolve()
    with path.open("rb") as stream:
        data = tomllib.load(stream)
    with REFERENCE.open("rb") as stream:
        reference = tomllib.load(stream)
    c = reference["assumed"] | config(path)
    # A partial V2 file inherits V2 task rates ahead of the V1 defaults.
    for key, default in reference["assumed"].items():
        if not any(key in data.get(section, {}) for section in ("theoretical","assumed","nominal","measured")):
            c[key] = default
    teams = []
    for name in ("team_a","team_b"):
        team = reference[name] | data.get(name, {})
        base = path.parent if "controller" in data.get(name,{}) else REFERENCE.parent
        team["controller"] = str((base/team["controller"]).resolve())
        teams.append(team)
    options = dict(config=c, teams=teams, scene=data.get("scene", {}),
                   fps=data.get("display", {}).get("fps", 60))
    if "dynamics" in data:
        with DYNAMIC_REFERENCE.open("rb") as stream:
            defaults = tomllib.load(stream)["dynamics"]
        dyn = data["dynamics"]
        options["dynamics"] = {k: defaults[k] | dyn.get(k,{}) for k in ("assumed","motor_assumed")}
        sources = dict(data.get("sources",{}))
        for section,label in (("theoretical","理论"),("nominal","厂商标称"),("measured","实测")):
            for key in data.get(section,{}):
                sources["config."+key] = label
                for i,team in enumerate(teams):
                    if key not in team.get("vehicle",{}):
                        sources[f"teams.{i}.vehicle.{key}"] = label
            for group,prefix in (("assumed",""),("motor_assumed","motor_")):
                params = dyn.get(prefix+section,{})
                options["dynamics"][group].update(params)
                for i,team in enumerate(teams):
                    for part in (("left","right") if prefix else ("vehicle",)):
                        for key in params:
                            if key not in team.get(part,{}): sources[f"teams.{i}.{part}.{key}"] = label
        options["sources"] = sources | data.get("sources",{})
    return options


class Schedule:
    """Integer phase arithmetic for rational frequencies on a fixed time grid."""
    def __init__(self, c):
        self.rate = c["rate"]
        if type(self.rate) is not int or self.rate <= 0:
            raise ValueError("Physics frequency must be a positive integer")
        self.ratios = {}
        for task in TASKS:
            ratio = Fraction(str(c[task])) / self.rate
            if not 0 < ratio <= 1:
                raise ValueError(f"{task} must be positive and <= physics frequency")
            self.ratios[task] = (ratio.numerator, ratio.denominator)

    def due(self, task, tick):
        a,b = self.ratios[task]
        return tick == 0 or tick*a//b != (tick-1)*a//b


class SpeedLoop:
    def __init__(self):
        self.integral = 0.
        self.previous = None

    def update(self, target, measured, dt, c):
        target = clamp(target, c["speed"])
        error = target - measured
        change = 0 if self.previous is None else (error-self.previous)/dt
        self.previous = error
        integral = self.integral + error*dt
        raw = (target/c["speed"] + c["speed_kp"]*error +
               c["speed_ki"]*integral + c["speed_kd"]*change)
        if abs(raw) <= 1 or raw*error < 0:
            self.integral = integral
        return clamp(raw, 1)


class Plant(World):
    def __init__(self, c, seed, scene, vehicles=None):
        self.vehicles = vehicles
        self.locator = RectangleLocator(c["width"], c["height"])
        self.scene = scene
        for name, size in (("poses",3),("pieces",4),("obstacles",5)):
            for row in scene.get(name, []):
                if len(row) != size or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in row):
                    raise ValueError(f"Invalid scene.{name} entry")
        if "poses" in scene and len(scene["poses"]) != 2:
            raise ValueError("scene.poses requires two vehicle poses")
        self.schedule = Schedule(c)
        super().__init__(c, seed)
        if vehicles:
            self.match.c = c | dict(mass=self.cars[0].body.mass,mass_b=self.cars[1].body.mass)

    def make_car(self,team):
        if self.vehicles:
            c,motors = self.vehicles[team]
            return DynamicCar(self.space,c,team,motors)
        return super().make_car(team)

    def initial_pieces(self):
        if "pieces" in self.scene and self.match.round == 0:
            for team, kind, x, y in self.scene["pieces"]:
                if type(team) is not int or team not in (0,1) or type(kind) is not int or not 0 <= kind < 7:
                    raise ValueError("Invalid scene piece team or kind")
                yield team, kind, (x,y)
        else:
            yield from super().initial_pieces()

    def setup(self):
        super().setup()
        self.obstacles = []
        for x,y,width,depth,height in self.scene.get("obstacles", []):
            if min(width,depth,height) <= 0:
                raise ValueError("Obstacle dimensions must be positive")
            body = pm.Body(body_type=pm.Body.STATIC)
            body.position = x,y
            shape = pm.Poly.create_box(body, (width,depth))
            shape.friction = self.c["contact_friction"]
            shape.elasticity = self.c["elasticity"]
            self.space.add(body,shape)
            self.heights[shape] = height
            self.obstacles.append(shape)
        self.refresh_scans()
        if self.scene:
            for shape in self.heights:
                if shape.body == self.space.static_body:
                    continue
                bb = shape.bb
                if bb.left < 0 or bb.bottom < 0 or bb.right > self.c["width"] or bb.top > self.c["height"]:
                    raise ValueError("Scene object outside field")
                for hit in self.space.shape_query(shape):
                    contact = hit.contact_point_set
                    # Chipmunk can report a zero normal for concentric shapes.
                    penetrates = contact.normal.length == 0 or any(p.distance < -1e-7 for p in contact.points)
                    if hit.shape.body != shape.body and penetrates:
                        raise ValueError("Overlapping objects in scene")

    def reset_cars(self):
        super().reset_cars()
        poses = self.scene.get("poses")
        if poses is not None:
            for car,estimate,(x,y,h) in zip(self.cars,self.estimates,poses):
                car.body.position = x,y
                car.body.angle = h
                self.space.reindex_shapes_for_body(car.body)
                estimate.pose = (x,y,h)
        self.counts = [(0,0),(0,0)]
        self.measured_speed = [(0.,0.),(0.,0.)]
        self.encoder_tick = self.imu_tick = self.motor_tick = self.elapsed
        self.gyros = [self.c["gyro_bias"]]*2
        self.refresh_scans()

    def advance_drive(self, commands, dt):
        due = self.schedule.due("motor_hz",self.elapsed)
        if due:
            motor_dt = (self.elapsed-self.motor_tick)/self.c["rate"] or 1/self.c["motor_hz"]
            self.motor_tick = self.elapsed
        for car, pwm in zip(self.cars,commands):
            if self.vehicles:
                if due: car.duty = tuple(pwm)
                car.integrate(dt)
                continue
            if due:
                car.update_wheels([value*self.c["speed"] for value in pwm],motor_dt)
            car.apply_traction(dt)

    def update_sensors(self, dt):
        c, tick = self.c, self.elapsed
        if self.schedule.due("encoder_hz", tick):
            period = (tick-self.encoder_tick)*dt
            self.encoder_tick = tick
            for i,car in enumerate(self.cars):
                counts = tuple(turns_to_counts(n,car.c["counts"]) for n in car.rotations)
                self.measured_speed[i] = tuple(counts_to_travel(n-old,car.c["wheel"],car.c["counts"])/period
                                               for n,old in zip(counts,self.counts[i]))
                self.counts[i] = counts
        if self.schedule.due("imu_hz", tick):
            self.imu_tick = tick
            self.gyros = [car.body.angular_velocity+c["gyro_bias"] for car in self.cars]
        for i,estimate in enumerate(self.estimates):
            estimate.update(self.counts[i],self.gyros[i],dt)
        if self.schedule.due("scan_rate", tick):
            self.refresh_scans()

    def refresh_scans(self):
        super().refresh_scans()
        self.wall_fixes = [self.locator.locate(beams, self.scan_tick/self.c["rate"], odom.pose)
                           for beams, odom in zip(self.scans, self.estimates)]

    def observation(self,i):
        rate = self.c["rate"]
        return Observation(self.elapsed/rate, tuple(self.estimates[i].pose),
                           tuple(self.scans[i]), self.scan_tick/rate, self.gyros[i],
                           self.imu_tick/rate, self.counts[i], self.encoder_tick/rate, self.wall_fixes[i])


def load_controller(team, index):
    path = Path(team["controller"])
    spec = importlib.util.spec_from_file_location(f"sil_team_{index}",path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Controller(), hashlib.sha256(path.read_bytes()).hexdigest()


def pack(cmd):
    if isinstance(cmd, MotorCommand):
        return ["pwm",cmd.left_pwm,cmd.right_pwm]
    if isinstance(cmd, WheelSpeed):
        return ["speed",cmd.left,cmd.right]
    raise TypeError("Controller.update must return MotorCommand or WheelSpeed")


class Simulation:
    def __init__(self, options, seed=42, replay=False):
        self.options = copy.deepcopy(options)
        vehicles = None
        if "dynamics" in options:
            vehicles = [vehicle_config(options["config"],team,options["dynamics"],i)
                        for i,team in enumerate(options["teams"])]
        self.world = Plant(self.options["config"],seed,self.options["scene"],vehicles)
        self.manual = {}
        self.controllers = []
        self.hashes = []
        if not replay:
            for i,team in enumerate(self.options["teams"]):
                controller,digest = load_controller(team,i)
                self.controllers.append(controller)
                self.hashes.append(digest)
        self.reset_control()

    def reset_control(self):
        self.commands = [MotorCommand(0,0)]*2
        self.pwm = [(0.,0.)]*2
        self.loops = [[SpeedLoop(),SpeedLoop()] for _ in range(2)]
        self.last = {}
        for controller,team in zip(self.controllers,self.options["teams"]):
            controller.reset(copy.deepcopy(team.get("options",{})))

    def step(self, recorded=None):
        w = self.world
        tick, c = w.elapsed,w.c
        if w.match.result:
            raise ValueError("Match has finished")
        if recorded is not None:
            if recorded["tick"] != tick or len(recorded["commands"]) != 2:
                raise ValueError("Invalid replay input sequence")
            types = {"pwm": MotorCommand,"speed": WheelSpeed}
            self.commands = [types[mode](left,right) for mode,left,right in recorded["commands"]]
        for task in ("navigation_hz","control_hz","pid_hz"):
            if not w.schedule.due(task,tick):
                continue
            dt = (tick-self.last[task])/c["rate"] if task in self.last else 1/c[task]
            self.last[task] = tick
            for i,controller in enumerate(self.controllers):
                if task == "navigation_hz" and hasattr(controller,"plan"):
                    controller.plan(w.observation(i),dt)
                if task == "control_hz":
                    self.commands[i] = (self.manual[i] if i in self.manual else
                                        controller.update(w.observation(i),dt))
            if task == "pid_hz":
                for i,cmd in enumerate(self.commands):
                    if isinstance(cmd,WheelSpeed):
                        self.pwm[i] = tuple(loop.update(target,speed,dt,c) for loop,target,speed in
                                            zip(self.loops[i],(cmd.left,cmd.right),w.measured_speed[i]))
        for i,cmd in enumerate(self.commands):
            if isinstance(cmd,MotorCommand):
                self.pwm[i] = (cmd.left_pwm,cmd.right_pwm)
                self.loops[i] = [SpeedLoop(),SpeedLoop()]
        if not w.match.round and w.match.tick >= round(w.match.duration*c["rate"]):
            self.pwm = [(0.,0.)]*2
        row = dict(tick=tick,time_s=tick/c["rate"],commands=list(map(pack,self.commands)),
                   pwm=[list(pair) for pair in self.pwm])
        if recorded is not None and row != recorded:
            raise ValueError(f"Replay diverged at tick {tick}")
        old_round = w.match.round
        w.step(self.pwm)
        if old_round != w.match.round:
            self.reset_control()
        return row
