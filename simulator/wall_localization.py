"""Local rectangle-map scan matching. Lengths in mm, angles in radians."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WallFix:
    time: float
    pose: tuple | None
    wall_distances: tuple | None  # west, east, south, north, from body origin
    support: float
    rms_mm: float | None
    status: str


def pavo_scan(raw_beams):
    """PAVO packet angles in degrees, decoded ranges in metres, to body convention.

    Angles here are before the optional sensor-to-body mount transform.
    """
    return tuple((math.remainder(math.radians(angle - 180), math.tau),
                  distance * 1000 if distance is not None and distance > 0 else None)
                 for angle, distance in raw_beams)


class RectangleLocator:
    def __init__(self, width_mm, height_mm, mount=(0., 0., 0.)):
        if not all(math.isfinite(v) and v > 0 for v in (width_mm, height_mm)):
            raise ValueError("Positive finite field dimensions required")
        self.width, self.height = width_mm, height_mm
        self.mount = mount  # forward, left, yaw relative to body

    def locate(self, beams, time, prior):
        """Solve one scan with a known approximate body pose at acquisition."""
        forward, left, yaw = self.mount
        points = [(forward + r * math.cos(a + yaw), left + r * math.sin(a + yaw))
                  for a, r in beams if r is not None and r > 0
                  and math.isfinite(r) and math.isfinite(a)]
        stride = max(1, math.ceil(len(points) / 360))
        points = points[::stride]
        x, y, h = prior
        support, rms = 0., None

        def reject(reason):
            return WallFix(time, None, None, support, rms, reason)

        if len(points) < 12:
            return reject("too_few_returns")
        for iteration in range(12):
            c, s = math.cos(h), math.sin(h)
            axes = [[], []]
            errors = []
            gate = 160 if iteration < 4 else 65
            for px, py in points:
                rx, ry = c*px-s*py, s*px+c*py
                wx, wy = x+rx, y+ry
                if not (-gate <= wx <= self.width+gate and -gate <= wy <= self.height+gate):
                    continue
                dx = wx if wx < self.width/2 else wx-self.width
                dy = wy if wy < self.height/2 else wy-self.height
                axis, error, slope = (0,dx,-ry) if abs(dx)<abs(dy) else (1,dy,rx)
                if abs(error) <= gate:
                    weight = min(1., 15/max(abs(error), 1.))
                    axes[axis].append((error, slope, weight))
                    errors.append(error)
            support = len(errors)/len(points)
            rms = math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None
            if min(map(len,axes)) < 5:
                return reject("missing_wall_direction")
            # Eliminate x/y translations separately before solving the yaw increment.
            means = []
            numerator = denominator = 0.
            for group in axes:
                total = sum(w for _,_,w in group)
                e0 = sum(e*w for e,_,w in group)/total
                j0 = sum(j*w for _,j,w in group)/total
                means.append((e0,j0))
                for e,j,w in group:
                    numerator += w*(j-j0)*(e-e0)
                    denominator += w*(j-j0)**2
            if denominator < len(errors)*80**2:
                return reject("short_wall_segments")
            turn = -numerator/denominator
            dx,dy = (-e-j*turn for e,j in means)
            if iteration >= 4 and max(abs(dx),abs(dy)) < .1 and abs(turn)<.0001:
                if support < .55 or rms > 35 or not (0<=x<=self.width and 0<=y<=self.height):
                    return reject("poor_match")
                if math.dist((x,y),prior[:2]) > 300 or abs(math.remainder(h-prior[2],math.tau)) > .35:
                    return reject("poor_match")
                # Both headings fit the same four walls. Near centre even the
                # independently supplied position prior cannot select a branch.
                mirror = (self.width-x,self.height-y)
                if math.dist(mirror,prior[:2]) <= 300:
                    return reject("ambiguous_heading")
                return WallFix(time,(x,y,math.remainder(h,math.tau)),
                               (x,self.width-x,y,self.height-y),support,rms,"ok")
            x,y,h = x+dx,y+dy,h+turn
        return reject("no_convergence")


class WallTracker:
    """Per-robot history; odometry and scans must share an acquisition time."""
    def __init__(self, locator):
        self.locator = locator
        self.anchor = self.pending = None

    @staticmethod
    def predict(anchor, odom):
        fix, old = anchor
        x,y,h = fix.pose
        rot = h-old[2]
        dx,dy = odom[0]-old[0],odom[1]-old[1]
        return (x+dx*math.cos(rot)-dy*math.sin(rot),
                y+dx*math.sin(rot)+dy*math.cos(rot),h+odom[2]-old[2])

    def update(self, beams, time, odom):
        prior = self.predict(self.anchor,odom) if self.anchor else odom
        fix = self.locator.locate(beams,time,prior)
        if fix.pose is None and self.anchor and fix.status not in ("ambiguous_heading","too_few_returns"):
            # With a recently tracked heading, extrema along map axes give wall
            # translation hypotheses. Interior obstacles can only shorten a ray.
            if 0 <= time-self.anchor[0].time <= 10:
                candidate = self.reacquire(beams,time,prior[2])
                if candidate:
                    if self.pending and 0 < time-self.pending[0].time <= .5:
                        predicted = self.predict(self.pending,odom)
                        if (math.dist(candidate.pose[:2],predicted[:2]) < 100 and
                            abs(math.remainder(candidate.pose[2]-predicted[2],math.tau)) < .12):
                            fix = candidate
                    self.pending = candidate,odom
                else:
                    self.pending = None
        else:
            self.pending = None
        if fix.pose:
            self.anchor = fix,odom
            self.pending = None
        return fix

    def reacquire(self, beams, time, heading):
        loc = self.locator
        mx,my,yaw = loc.mount
        c,s = math.cos(heading),math.sin(heading)
        points = [(mx*c-my*s+r*math.cos(a+yaw+heading),
                   mx*s+my*c+r*math.sin(a+yaw+heading))
                  for a,r in beams if r and r>0 and math.isfinite(r) and math.isfinite(a)]
        if len(points)<12: return None
        xs,ys = zip(*points)
        if min(abs(max(xs)-min(xs)-loc.width),abs(max(ys)-min(ys)-loc.height)) > 100:
            return None
        best = None
        for x in (-min(xs),loc.width-max(xs)):
            for y in (-min(ys),loc.height-max(ys)):
                fix = loc.locate(beams,time,(x,y,heading))
                if fix.pose and fix.support >= .8 and fix.rms_mm <= 20:
                    if best and math.dist(best.pose[:2],fix.pose[:2]) > 100:
                        return None
                    if best is None or fix.support > best.support: best = fix
        return best
