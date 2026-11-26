import math
import pymunk as pm
from match import Match
from rules import layout
from sensors import Odometry, scan
from vehicle import Car


class World:
    def __init__(self,c,seed):
        self.c,self.seed = c,seed
        self.match = Match(c)
        self.elapsed = 0
        self.setup()

    def setup(self):
        c = self.c
        self.space = pm.Space()
        self.heights = {}
        w,h = c["width"],c["height"]
        corners = [(0,0),(w,0),(w,h),(0,h)]
        for i,p in enumerate(corners):
            wall = pm.Segment(self.space.static_body,p,corners[(i+1)%4],0)
            wall.friction = c["contact_friction"]
            wall.elasticity = c["elasticity"]
            self.space.add(wall)
            self.heights[wall] = c["wall_height"]
        self.cars = []
        self.pieces = []
        for team,kind,pos in layout(c,self.seed+self.match.round):
            r,m = c["diameter"]/2,c["piece_mass"]
            body = pm.Body(m,pm.moment_for_circle(m,0,r))
            body.position = pos
            shape = pm.Circle(body,r)
            shape.friction,shape.elasticity = c["contact_friction"],c["elasticity"]
            self.space.add(body,shape)
            self.heights[shape] = c["piece_height"]
            self.pieces.append((team,kind,body))
        self.reset_cars()

    def reset_cars(self):
        for car in self.cars:
            self.space.remove(car.shape, car.body)
            del self.heights[car.shape]
        self.cars = [Car(self.space, self.c, team) for team in range(2)]
        self.estimates = []
        for car in self.cars:
            self.heights[car.shape] = self.c["car_height"]
            self.estimates.append(Odometry((*car.body.position, car.body.angle), self.c))
        self.gyros = [0., 0.]
        self.refresh_scans()

    def refresh_scans(self):
        self.scan_tick = self.elapsed
        self.scans = [scan(self, car) for car in self.cars]
        self.scan_poses = [(*car.body.position, car.body.angle) for car in self.cars]

    def step(self, commands):
        if self.match.result:
            return
        c = self.c
        dt = 1/c["rate"]
        settling = not self.match.round and self.match.tick >= round(self.match.duration*c["rate"])
        if settling:
            commands = [(0,0),(0,0)]
            for car in self.cars:
                car.speed = [0.,0.]
                car.body.body_type = pm.Body.KINEMATIC
                car.body.velocity = (0,0)
                car.body.angular_velocity = 0
        for car,cmd in zip(self.cars,commands):
            if not settling: car.drive(cmd,dt)
        for _,_,body in self.pieces:
            decay = math.exp(-c["floor_drag"]*dt)
            body.velocity *= decay
            body.angular_velocity *= decay
        self.space.step(dt)
        self.elapsed += 1
        for i,car in enumerate(self.cars):
            counts = tuple(round(n*c["counts"]) for n in car.rotations)
            self.gyros[i] = car.body.angular_velocity+c["gyro_bias"]
            self.estimates[i].update(counts,self.gyros[i],dt)
        if self.elapsed - self.scan_tick >= max(1, round(c["rate"]/c["scan_rate"])):
            self.refresh_scans()
        old_round = self.match.round
        settled = all(b.velocity.length < c["rest_speed"] for _,_,b in self.pieces) and all(car.body.velocity.length < c["rest_speed"] for car in self.cars)
        self.match.update(((t,k,*b.position) for t,k,b in self.pieces), settled)
        if self.match.round != old_round:
            self.setup()
