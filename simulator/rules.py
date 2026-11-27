import math
import random
import tomllib
from pathlib import Path


def config(path=None):
    with open(path or Path(__file__).with_name("config.toml"), "rb") as f:
        data = tomllib.load(f)
    with open(Path(__file__).with_name("config.toml"), "rb") as f:
        defaults = tomllib.load(f)
    c = {k: v for section in ("theoretical", "assumed", "nominal", "measured")
         for k, v in (defaults.get(section, {}) | data.get(section, {})).items()}
    if c["piece_mu"] < 0 or c["gravity_mm_s2"] <= 0:
        raise ValueError("Invalid floor friction or gravity")
    if not 0 < min(c["mass"], c["mass_b"]) or max(c["mass"], c["mass_b"]) >= 2.5:
        raise ValueError("Competition car mass must be positive and below 2.5 kg")
    dims = (c["length"], c["breadth"], c["car_height"])
    if min(dims) <= 0 or max(dims) > 300 or max(dims[:2]) > c["start"]:
        raise ValueError("Car must fit the start area and the 300 mm size limit")
    if min(c[k] for k in ("track", "wheel", "counts", "range_mm", "scan_rate", "rate")) <= 0:
        raise ValueError("Wheel geometry, encoder resolution, range and rates must be positive")
    if c["rays"] < 2 or not 0 < c["fov"] <= 360 or c["laser_height"] < 0:
        raise ValueError("Invalid lidar scan geometry")
    return c


def layout(c, seed):
    rng = random.Random(seed)
    side, size = c["scatter"], c["diameter"]
    grid = math.floor(side / size)
    if grid ** 2 < 32 or side > min(c["width"], c["height"]):
        raise ValueError("Invalid central placement area")
    cells = rng.sample([(i, j) for i in range(grid) for j in range(grid)], 32)
    step = side / grid
    kinds = [k for k, n in enumerate((1, 2, 2, 2, 2, 2, 5)) for _ in range(n)]
    for index, (i, j) in enumerate(cells):
        def coord(cell, axis):
            return ((c[axis]-side)/2 + cell*step + size/2 + rng.random()*(step-size))
        yield index // 16, kinds[index % 16], (coord(i,"width"), coord(j,"height"))
