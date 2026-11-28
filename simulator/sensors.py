from units import radians
from odometry import Odometry  # existing imports


def scan(world, car):
    import pymunk as pm
    c = world.c
    start = car.body.position
    query = pm.ShapeFilter(group=car.shape.filter.group)
    readings = []
    for i in range(c["rays"]):
        angle = radians(c["fov"]*(i/(c["rays"]-1)-0.5))
        direction = pm.Vec2d(1,0).rotated(car.body.angle+angle)
        end = start + direction*c["range_mm"]
        hits = world.space.segment_query(start,end,0,query)
        fraction = min((h.alpha for h in hits if world.heights[h.shape]>=c["laser_height"]),default=None)
        readings.append((angle,None if fraction is None else fraction*c["range_mm"]))
    return readings
