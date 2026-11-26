import math
import pymunk as pm
from control import clamp


class Car:
    def __init__(self, space, c, team):
        self.c = c
        mass = c["mass_b" if team else "mass"]
        dims = (c["length"],c["breadth"])
        self.body = pm.Body(mass,pm.moment_for_box(mass,dims))
        offset = c["start"]/2+c["start_inset"]
        self.body.position = (c["width"]-offset,c["height"]-offset) if team else (offset,offset)
        self.body.angle = math.pi if team else 0
        self.shape = pm.Poly.create_box(self.body,dims)
        self.shape.friction = c["contact_friction"]
        self.shape.elasticity = c["elasticity"]
        self.shape.filter = pm.ShapeFilter(group=team+1)
        space.add(self.body,self.shape)
        self.speed = [0.,0.]
        self.rotations = [0.,0.]

    def drive(self, commands, dt):
        c,b = self.c,self.body
        forward = b.rotation_vector
        side = forward.perpendicular()
        for i,y in enumerate((c["track"]/2,-c["track"]/2)):
            desired = clamp(commands[i],c["speed"])
            self.speed[i] += clamp(desired-self.speed[i],c["accel"]*dt)
            self.rotations[i] += self.speed[i]*dt/(math.pi*c["wheel"])
            p = b.local_to_world((0,y))
            v = b.velocity_at_world_point(p)
            force = clamp((self.speed[i]-v.dot(forward))*c["drive_gain"],c["drive_force"])
            lateral = clamp(-v.dot(side)*c["lateral_grip"],c["drive_force"])
            b.apply_force_at_world_point(forward*force + side*lateral,p)
