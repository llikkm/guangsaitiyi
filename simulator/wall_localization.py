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
                return WallFix(time,(x,y,math.remainder(h,math.tau)),
                               (x,self.width-x,y,self.height-y),support,rms,"ok")
            x,y,h = x+dx,y+dy,h+turn
        return reject("no_convergence")
