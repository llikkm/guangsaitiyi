import math


class Odometry:
    def __init__(self, initial_pose, c):
        self.pose = initial_pose
        self.c = c
        self.counts = (0,0)

    def update(self, counts, gyro, dt):
        x,y,h = self.pose
        distance = sum(n-old for n,old in zip(counts,self.counts))*math.pi*self.c["wheel"]/(2*self.c["counts"])
        turn = gyro*dt
        self.pose = (x+distance*math.cos(h+turn/2),y+distance*math.sin(h+turn/2),math.remainder(h+turn,math.tau))
        self.counts = counts


def scan(world, car):
    import pymunk as pm
    c = world.c
    start = car.body.position
    query = pm.ShapeFilter(group=car.shape.filter.group)
    readings = []
    for i in range(c["rays"]):
        angle = math.radians(c["fov"]*(i/(c["rays"]-1)-0.5))
        direction = pm.Vec2d(1,0).rotated(car.body.angle+angle)
        end = start + direction*c["range_mm"]
        hits = world.space.segment_query(start,end,0,query)
        fraction = min((h.alpha for h in hits if world.heights[h.shape]>=c["laser_height"]),default=None)
        readings.append((angle,None if fraction is None else fraction*c["range_mm"]))
    return readings
