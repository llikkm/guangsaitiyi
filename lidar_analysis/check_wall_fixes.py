"""Reproduce the rectangular map negative control from captures or shared scans."""
import argparse
from collections import Counter
import hashlib
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"simulator"))
from wall_localization import RectangleLocator, pavo_scan


def extract(paths):
    from analyze_static import read_capture
    scans, names, stamps, indices = [], [], [], []
    for path in paths:
        ranges,times = read_capture(path)
        print(path.name,hashlib.sha256(path.read_bytes()).hexdigest())
        for i in range(1,len(ranges)-1,30):
            # Original packet resolution is 2 mm. Zero denotes a missing return.
            scans.append(np.rint(np.nan_to_num(ranges[i])*500).astype(np.uint16))
            stamps.append(times[i])
            indices.append(i)
            names.append(path.name)
    return dict(raw=scans,time=stamps,frame=indices,capture=names)


def check(data):
    locator = RectangleLocator(3000,2000)
    angles = np.arange(4500)*0.08
    counts = Counter()
    for raw,time in zip(data["raw"],data["time"]):
        beams = pavo_scan(zip(angles,raw.astype(float)*.002))
        fix = locator.locate(beams,float(time),(1500,1000,0))
        counts[fix.status] += 1
    return counts


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures",nargs="*",type=Path)
    parser.add_argument("--export",type=Path)
    args = parser.parse_args()
    if args.captures:
        data = extract(args.captures)
        if args.export: np.savez_compressed(args.export,**data)
    else:
        data = np.load(HERE/"samples"/"room_scans.npz",allow_pickle=False)
    counts = check(data)
    print(sum(counts.values()),dict(counts))
    if counts["ok"]: raise SystemExit("Room scan unexpectedly accepted")
