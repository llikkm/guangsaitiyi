import math


def clamp(value, bound):
    return min(bound, max(-bound, value))


def navigate(pose, target, c):
    x,y,h = pose
    dx,dy = target[0]-x,target[1]-y
    distance = math.hypot(dx,dy)
    if distance <= c["arrival"]:
        return (0,0), True
    error = math.remainder(math.atan2(dy,dx)-h, math.tau)
    forward = min(c["speed"], distance*c["approach_gain"])*max(0,math.cos(error))
    turn = c["track"]/2*c["turn_gain"]*error
    return (clamp(forward-turn,c["speed"]), clamp(forward+turn,c["speed"])), False
