"""Estimate motion against one fixed scan using trimmed point-to-line ICP."""
from pathlib import Path
import argparse

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import median_filter

from analyze_static import read_capture, DIRECTIONS


def points(scan):
    xy = scan[:, None] * DIRECTIONS
    xy[(scan < .45) | (scan > 10)] = np.nan
    return xy


def reference(scan):
    p = points(scan)
    edge = p[4:] - p[:-4]
    length = np.linalg.norm(edge, axis=1)
    keep = (length > .012) & (length < .22) & np.isfinite(p[2:-2]).all(axis=1)
    n = edge[keep][:, ::-1] * [1, -1] / length[keep, None]
    q = p[2:-2][keep]
    return cKDTree(q), q, n


def align(scan, pose, target):
    tree, q, normals = target
    p = points(scan)[::5]
    p = p[np.isfinite(p).all(axis=1)]
    pose = pose.copy()
    for _ in range(24):
        c, s = np.cos(pose[2]), np.sin(pose[2])
        rot = p @ np.array([[c,s],[-s,c]])
        xy = rot + pose[:2]
        dist, idx = tree.query(xy)
        n = normals[idx]
        err = ((xy-q[idx])*n).sum(axis=1)
        mask = (dist < .30) & (abs(err) < .16)
        if mask.sum() < 40:
            raise ValueError("Insufficient scan overlap")
        # Downweight moving objects and depth discontinuities.
        w = np.sqrt(np.minimum(1, .018 / np.maximum(abs(err[mask]), 1e-8)))
        deriv = rot[:,0]*n[:,1] - rot[:,1]*n[:,0]
        J = np.column_stack((n, deriv))[mask]
        delta = np.linalg.lstsq(J*w[:,None], -err[mask]*w, rcond=None)[0]
        pose += delta
        if np.linalg.norm(delta) < .00005:
            break
    return pose, mask.mean(), np.median(abs(err[mask]))


def run(path,out):
    scans,t = read_capture(path)
    scans,t = scans[1:-1], t[1:-1]
    t = t-t[0]
    target = reference(scans[0])
    pose = np.zeros(3)
    rows = []
    for scan,ts in zip(scans,t):
        pose, share, residual = align(scan, pose, target)
        rows.append([ts,*pose[:2],np.rad2deg(pose[2]),share,residual])
    a = np.array(rows)
    out.mkdir(parents=True,exist_ok=True)
    np.savetxt(out/(path.stem+'.csv'),a,delimiter=',',fmt='%.6f',comments='',
               header='time_s,right_m,forward_m,ccw_deg,inlier_fraction,median_abs_line_residual_m')
    print(path.stem, 'sampled heading',np.round(a[::10,3],1),'end',np.median(a[-10:],axis=0),'worst overlap',a[:,4].min())
    return a


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files',nargs='+',type=Path)
    parser.add_argument('--out',type=Path,default=Path('lidar_analysis/motion_results'))
    args = parser.parse_args()
    fig, axes = plt.subplots(len(args.files),2,figsize=(11,3*len(args.files)),squeeze=False)
    for path,(left,right) in zip(args.files,axes):
        a = run(path,args.out)
        left.plot(a[:,0],a[:,3]); left.set(ylabel='CCW angle (deg)',title=path.stem)
        right.plot(a[:,0],100*a[:,1],label='Right')
        right.plot(a[:,0],100*a[:,2],label='Forward')
        right.set(ylabel='Displacement (cm)'); right.legend()
        for ax in (left,right):
            ax.grid(alpha=.3); ax.set_xlabel('Time (s)')
    fig.tight_layout(); fig.savefig(args.out/'motion.png',dpi=150)
