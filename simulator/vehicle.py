import math
import pymunk as pm
from control import clamp
from units import travel_to_turns


def bumper_points(c):
    mode = c["bumper"]
    if mode == "flat":
        return []
    depth, half = c["bumper_depth"], c["bumper_width"]/2
    if mode == "u":
        outline = [(depth, half), (0, half), (0, -half), (depth, -half)]
    elif mode == "custom":
        outline = c["custom_outline"]
    else:
        raise ValueError("Unknown bumper profile")
    radius = c["bumper_radius"]
    if len(outline) < 2 or radius <= 0 or not 0 < c["bumper_height"] <= 300:
        raise ValueError("Invalid bumper dimensions")
    points = [(c["length"]/2+x, y) for x, y in outline]
    limit = min(c["start"], 300)
    if any(not (0 <= x <= limit-c["length"]/2-radius and abs(y)+radius <= limit/2)
           for x, y in points) or any(a == b for a,b in zip(points,points[1:])):
        raise ValueError("Bumper must fit the competition envelope and have distinct vertices")
    return points


class Car:
    def __init__(self, space, c, team):
        self.c = c
        mass = c["mass_b" if team else "mass"]
        dims = (c["length"],c["breadth"])
        self.body = pm.Body(mass,pm.moment_for_box(mass,dims))
        offset = c["start"]/2+c["start_inset"]
        points = bumper_points(c)
        nose = max((x+c["bumper_radius"] for x,y in points), default=0)
        start_x = offset - max(0, nose-c["start"]/2)
        self.body.position = (c["width"]-start_x,c["height"]-offset) if team else (start_x,offset)
        self.body.angle = math.pi if team else 0
        self.shape = pm.Poly.create_box(self.body,dims)
        self.shape.friction = c["contact_friction"]
        self.shape.elasticity = c["elasticity"]
        self.shape.filter = pm.ShapeFilter(group=team+1)
        space.add(self.body,self.shape)
        self.rails = []
        for a,b in zip(points,points[1:]):
            rail = pm.Segment(self.body,a,b,c["bumper_radius"])
            rail.friction = self.shape.friction
            rail.elasticity = self.shape.elasticity
            rail.filter = self.shape.filter
            space.add(rail)
            self.rails.append(rail)
        self.speed = [0.,0.]
        self.rotations = [0.,0.]

    def drive(self, commands, dt):
        c,b = self.c,self.body
        forward = b.rotation_vector
        side = forward.perpendicular()
        for i,y in enumerate((c["track"]/2,-c["track"]/2)):
            desired = clamp(commands[i],c["speed"])
            self.speed[i] += clamp(desired-self.speed[i],c["accel"]*dt)
            self.rotations[i] += travel_to_turns(self.speed[i]*dt, c["wheel"])
            p = b.local_to_world((0,y))
            v = b.velocity_at_world_point(p)
            force = clamp((self.speed[i]-v.dot(forward))*c["drive_gain"],c["drive_force"])
            lateral = clamp(-v.dot(side)*c["lateral_grip"],c["drive_force"])
            b.apply_force_at_world_point(forward*force + side*lateral,p)
