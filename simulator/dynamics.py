"""Planar DC drivetrain. Wheel and tire calculations are in SI."""
from dataclasses import dataclass
import math

import pymunk as pm
from control import clamp
from vehicle import Car, bumper_points
from units import ENGINE_FORCE_PER_N, ENGINE_MOMENT_PER_SI, MM_PER_M


def properties(c):
    """Composite mass, local COM (mm), polar inertia about composite COM."""
    mass = c["mass"]
    center = pm.Vec2d(c["cg_x"],c["cg_y"])
    inertia = c["inertia_si"] * ENGINE_MOMENT_PER_SI or mass*(c["length"]**2+c["breadth"]**2)/12
    parts = [(mass,center,inertia)]
    points = bumper_points(c) or [(c["length"]/2,-c["breadth"]/2),(c["length"]/2,c["breadth"]/2)]
    segments = [(pm.Vec2d(*a),pm.Vec2d(*b)) for a,b in zip(points,points[1:])]
    length = sum((b-a).length for a,b in segments)
    for a,b in segments:
        span = (b-a).length
        m = c["bumper_mass"]*span/length
        parts.append((m,(a+b)/2,m*span**2/12))
    total = sum(m for m,_,_ in parts)
    center = sum((m*p for m,p,_ in parts),pm.Vec2d(0,0))/total
    moment = sum(j+m*(p-center).length_squared for m,p,j in parts)
    return total,center,moment


def vehicle_config(c,team,dynamics,i):
    v = c | {"mass":c["mass_b"] if i else c["mass"]} | dynamics["assumed"] | team.get("vehicle",{})
    motors = [dynamics["motor_assumed"] | team.get(side,{}) for side in ("left","right")]
    if not all(math.isfinite(value) for group in [v,*motors] for value in group.values() if isinstance(value,(float,int))):
        raise ValueError("Physical parameters must be finite")
    if min(v[k] for k in ("mass","track","wheel","counts","length","breadth","car_height")) <= 0:
        raise ValueError("Mass, dimensions and encoder resolution must be positive")
    if max(v["length"],v["breadth"],v["car_height"]) > 300 or max(v["length"],v["breadth"]) > v["start"]:
        raise ValueError("Vehicle exceeds competition dimensions")
    if v["track"] > v["breadth"] or abs(v["cg_x"]) > v["length"]/2 or abs(v["cg_y"]) >= v["track"]/2:
        raise ValueError("Invalid wheel track or chassis centre of gravity")
    if min(v[k] for k in ("inertia_si","bumper_mass","tire_mu","rolling_mu","long_stiffness","side_stiffness")) < 0:
        raise ValueError("Inertia, masses and tire coefficients must be nonnegative")
    if v["battery_v"] <= 0 or v["zero_mode"] not in ("brake","coast"):
        raise ValueError("Invalid battery voltage or zero mode")
    for motor in motors:
        if motor["spec_mode"] == "rated":
            if min(motor[k] for k in ("rated_v","rated_a","rated_rpm","rated_nm")) <= 0:
                raise ValueError("Rated motor point must be positive")
            motor["kt"] = motor["rated_nm"]/motor["rated_a"]
            motor["ke"] = (motor["rated_v"]-motor["rated_a"]*motor["resistance"])/(motor["rated_rpm"]*math.tau/60)
        elif motor["spec_mode"] != "constants":
            raise ValueError("Unknown motor specification mode")
        if min(motor[k] for k in ("resistance","ke","kt","gear","efficiency","current_limit","stall_nm","wheel_inertia")) <= 0:
            raise ValueError("Motor constants must be positive")
        if motor["efficiency"] > 1 or not 0 <= motor["deadband"] < 1:
            raise ValueError("Invalid efficiency or PWM deadband")
    total,center,_ = properties(v)
    if total >= 2.5 or abs(center.y) >= v["track"]/2:
        raise ValueError("Loaded mass or centre of gravity outside competition envelope")
    return v,motors


@dataclass
class Wheel:
    omega: float = 0.
    angle: float = 0.
    current: float = 0.
    motor_rpm: float = 0.
    shaft_nm: float = 0.
    torque: float = 0.
    slip: float = 0.
    longitudinal_n: float = 0.
    lateral_n: float = 0.


class DynamicCar(Car):
    def __init__(self,space,c,team,motors):
        super().__init__(space,c | {"mass_b":c["mass"]},team)
        total,center,moment = properties(c)
        origin = self.body.position
        self.body.mass,self.body.moment = total,moment
        self.body.center_of_gravity = center
        self.body.position = origin
        space.reindex_shapes_for_body(self.body)
        self.motors = motors
        self.free_speeds = [c["battery_v"]*c["wheel"]/(2*m["ke"]*m["gear"]) for m in motors]
        self.previous_contact_speed = [None,None]
        self.wheels = [Wheel(),Wheel()]
        self.duty = (0.,0.)

    def integrate(self,dt):
        c,b = self.c,self.body
        radius = c["wheel"]/(2*MM_PER_M)
        forward = b.rotation_vector
        side = forward.perpendicular()
        for i,(wheel,motor,u) in enumerate(zip(self.wheels,self.motors,self.duty)):
            sign = 1 if i == 0 else -1
            contact = b.local_to_world((0,sign*c["track"]/2))
            velocity = b.velocity_at_world_point(contact)/MM_PER_M
            vx,vy = velocity.dot(forward),velocity.dot(side)
            share = 0.5+sign*b.center_of_gravity.y/c["track"]
            supported = share*b.mass
            normal = supported*c["gravity_mm_s2"]/MM_PER_M
            gear,j = motor["gear"],motor["wheel_inertia"]
            active = abs(u) > motor["deadband"]
            voltage = u*c["battery_v"] if active else 0.
            current = 0.
            if active or c["zero_mode"] == "brake":
                current = clamp((voltage-motor["ke"]*gear*wheel.omega)/motor["resistance"],motor["current_limit"])
            shaft = clamp(motor["kt"]*current,motor["stall_nm"])
            torque = shaft*gear*motor["efficiency"]
            # Limit dissipative impulses to the distance to electrical equilibrium.
            target = voltage/(motor["ke"]*gear)
            delta = clamp(torque*dt/j,abs(target-wheel.omega))
            free = wheel.omega+delta
            roll = c["rolling_mu"]*normal*radius*dt/j
            free -= clamp(free,roll)
            # Implicit linear creep at contact, capped by the common friction circle.
            k = c["long_stiffness"]
            # Implicit wheel reaction against measured contact speed. The second
            # bound limits the single-step impulse for an unusually light chassis.
            fx = k*(free*radius-vx)/(1+k*dt*max(radius**2/j,1/supported))
            if c.get("reviewed_dynamics"):
                # Estimate acceleration from contacts other than this tire. At a wall,
                # its reaction cancels the previous traction even at zero body speed.
                previous = self.previous_contact_speed[i]
                other_dv = 0 if previous is None else vx-previous-dt*wheel.longitudinal_n/supported
                fx = k*(free*radius-vx-other_dv)/(1+k*dt*(radius**2/j+1/supported))
                self.previous_contact_speed[i] = vx
            ky = c["side_stiffness"]
            fy = -ky*vy/(1+ky*dt/supported)
            magnitude = math.hypot(fx,fy)
            factor = min(1.,c["tire_mu"]*normal/magnitude) if magnitude else 1.
            fx,fy = fx*factor,fy*factor
            wheel.current,wheel.torque = current,delta*j/dt
            wheel.omega = free-fx*radius*dt/j
            wheel.motor_rpm = wheel.omega*gear*60/math.tau
            wheel.shaft_nm = shaft
            wheel.angle += wheel.omega*dt
            wheel.slip = wheel.omega*radius-vx
            wheel.longitudinal_n,wheel.lateral_n = fx,fy
            self.rotations[i] = wheel.angle/math.tau
            self.speed[i] = wheel.omega*radius*MM_PER_M
            b.apply_force_at_world_point((forward*fx+side*fy)*ENGINE_FORCE_PER_N,contact)
