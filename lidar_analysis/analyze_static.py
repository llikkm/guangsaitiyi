"""Offline replay of the observed PAVO 10H static_test recording layout."""
import argparse
import struct
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ANGLES = np.arange(4500) * 0.08
RAD = np.deg2rad(180 - ANGLES)  # raw 90 degrees is screen right
DIRECTIONS = np.column_stack((np.sin(RAD), np.cos(RAD)))
# Conservative interior windows selected from the observed forward range trace.
WINDOWS = [(1, 60, 1), (121, 360, 2), (401, 541, 3)]


def read_capture(path):
    data = path.read_bytes()
    if data[:4] != bytes.fromhex("d4c3b2a1") or struct.unpack_from("<I", data, 20)[0] != 1:
        raise ValueError("Expected little-endian microsecond Ethernet PCAP")
    records = np.frombuffer(data, np.uint8, offset=24).reshape(-1, 184)
    if not np.all(records[:, 8:16] == [168,0,0,0,168,0,0,0]):
        raise ValueError("Unexpected capture record size")
    if not (np.all(records[:, 28:31] == [8,0,69]) and np.all(records[:,39] == 17)
            and np.all(records[:,54:56] == [0,134])):
        raise ValueError("Expected Ethernet/IPv4/UDP layout")
    blocks = records[:, 58:178].reshape(-1, 12, 10).astype(int)
    if not np.all(blocks[:,:,:2] == [255,1]):
        raise ValueError("Unexpected block signature")
    angles = blocks[:,:,2] + 256 * blocks[:,:,3]
    if not np.all(np.diff(angles.ravel()) % 36000 == 16) or not np.all(angles % 8 == 0):
        raise ValueError("Angular gap or unsupported sampling configuration")
    # First block starts at 0.08 degrees after wrap. Its final pair wraps to 0.
    frames = np.cumsum(np.r_[0, np.diff(angles.ravel()) < 0]).reshape(angles.shape)
    ranges = np.full((frames.max()+1, 4500), np.nan)
    for beam, offset in enumerate([4, 7]):
        raw = blocks[:,:,offset] + 256 * blocks[:,:,offset+1]
        ranges[frames, (angles//8 + beam) % 4500] = np.where(raw, raw*.002, np.nan)
    stamps = records[:,178:182].copy().view("<u4").ravel().astype(np.int64)
    times = np.cumsum(np.r_[0, np.diff(stamps) % 2**32]) * 1e-6
    _, starts = np.unique(frames.ravel(), return_index=True)
    return ranges, times[starts//12]


def line(points, lo, hi):
    p = points[(ANGLES >= lo) & (ANGLES <= hi)]
    p = p[np.isfinite(p[:,0])]
    for _ in range(3):
        if len(p) < 8:
            return np.full(2, np.nan), np.nan
        center = p.mean(axis=0)
        normal = np.linalg.svd(p-center, full_matrices=False)[2][-1]
        offset = center @ normal
        errors = np.abs(p @ normal - offset)
        p = p[errors <= max(.012, 3*np.median(errors))]
    return normal, offset


def analyze(path, out):
    ranges, times = read_capture(path)
    if len(ranges) != 543:
        raise ValueError("Window annotations apply only to static_test")
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # blind bins have no returns
        background = np.nanmedian(ranges[1:-1], axis=0)
    rows = []
    for frame in range(1, len(ranges)-1):
        pts = ranges[frame,:,None] * DIRECTIONS
        static = pts.copy()
        static[np.abs(ranges[frame] - background) > .06] = np.nan
        ns, ds = zip(line(static,207.1,211.1), line(static,211.8,213.5))
        corner = np.linalg.solve(ns, ds) if np.isfinite(ns).all() and abs(np.linalg.det(ns)) > .5 else np.full(2,np.nan)
        normal, offset = line(static,93,113)
        board = np.nanmedian(pts[(ANGLES >=180) & (ANGLES <=180.56),1])
        ref = next((v for lo,hi,v in WINDOWS if lo<=frame<=hi), np.nan)
        rows.append([frame,times[frame],ref,board,offset/normal[0],*corner,
                     np.linalg.norm(corner),180+np.degrees(np.arctan2(*corner)),
                     *np.abs(ds), np.degrees(np.arctan2(-ns[0][0],ns[0][1]))%180, abs(offset)])
    a = np.asarray(rows)
    out.mkdir(exist_ok=True, parents=True)
    np.savetxt(out/'frames.csv',a,delimiter=',',fmt='%.6f',comments='',
               header='frame,time_s,board_reference_m,board_forward_m,cabinet_right_intercept_m,corner_x_right_m,corner_y_forward_m,corner_range_m,corner_display_deg,front_wall_perp_m,side_wall_perp_m,front_wall_axis_deg_mod180,cabinet_perp_m')
    fig, ax = plt.subplots(figsize=(7,7))
    pts = ranges[201,:,None]*DIRECTIONS
    ax.scatter(*pts.T,s=2,color='0.65')
    for lo,hi,label in [(207.1,211.1,'Corner wall A'),(211.8,213.5,'Corner wall B'),(93,113,'Cabinet fit'),(177,182,'Front sector')]:
        sub=pts[(ANGLES>=lo)&(ANGLES<=hi)]
        ax.scatter(*sub.T,s=8,label=label)
    ax.scatter(*a[200,5:7],marker='x',color='red',label='Wall intersection')
    ax.scatter(0,0,marker='^',color='black',label='Lidar')
    ax.set(xlabel='Right (m)',ylabel='Forward (m)',aspect='equal',xlim=(-7,7),ylim=(-2,10))
    ax.grid(alpha=.25); ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(out/'scan.png',dpi=160); plt.close(fig)
    fig,axes=plt.subplots(4,1,figsize=(9,8),sharex=True)
    for ax,col,label in zip(axes,[3,4,7,8],['Board (m)','Cabinet (m)','Corner (m)','Corner bearing (deg)']):
        ax.plot(a[:,1],a[:,col],lw=.9); ax.set_ylabel(label); ax.grid(alpha=.25)
    for lo,hi,v in WINDOWS:
        axes[0].axvspan(times[lo],times[hi],color='green',alpha=.13)
        vals=a[a[:,2]==v,3]
        print(v,len(vals),np.nanmedian(vals),np.nanpercentile(vals,[5,95]))
    axes[-1].set_xlabel('Device time since capture start (s)')
    fig.tight_layout();fig.savefig(out/'traces.png',dpi=160);plt.close(fig)
    print('Median / 5th and 95th percentiles for CSV columns 4 onward:')
    print(np.nanpercentile(a[:,4:],[50,5,95],axis=0))
    return a


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--out',type=Path,default=Path('lidar_analysis/results'))
    args=parser.parse_args()
    analyze(args.capture,args.out)
