import math
from units import counts_to_travel


class Odometry:
    def __init__(self, initial_pose, c):
        self.pose = initial_pose
        self.c = c
        self.counts = (0,0)

    def update(self, counts, gyro, dt):
        x,y,h = self.pose
        mean_counts = sum(n-old for n,old in zip(counts,self.counts))/2
        distance = counts_to_travel(mean_counts, self.c["wheel"], self.c["counts"])
        turn = gyro*dt
        self.pose = (x+distance*math.cos(h+turn/2),y+distance*math.sin(h+turn/2),math.remainder(h+turn,math.tau))
        self.counts = counts
