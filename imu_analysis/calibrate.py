"""Reproduce V2.2 IMU calibration and independent checks. Requires NumPy."""
import argparse
import json
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulator"))
from imu_sensor import ImuSensor, imu_defaults
FIT = ("static10", "face_px", "face_nx", "face_py", "face_ny", "face_pz", "face_nz",
       "cw1", "cw2", "ccw1", "ccw2")
HOLDOUT = ("static2", "cw3", "ccw3")
G = 9.80665


def load(root, name):
    complete = []
    for path in sorted((root / name).glob("*/samples.csv")):
        meta = json.loads(path.with_name("settings.json").read_text(encoding="utf-8-sig"))
        if meta.get("complete") is True and meta["run"] == name:
            complete.append((path, meta))
    if len(complete) != 1:
        raise ValueError(f"{name}: expected one completed samples.csv, found {len(complete)}")
    path, meta = complete[0]
    if (meta["sensor"], meta["odr_hz"], meta["dps_per_count"], meta["g_per_count"]) != (
            "LSM6DS3TR-C", 104, 0.00875, 0.000061):
        raise ValueError(f"{name}: unexpected chip or register scale")
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=np.int64)
    t = (data["t_sample_us"] - data["t_sample_us"][0]) / 1e6
    if np.any(np.diff(t) <= 0) or np.any(np.diff(data["seq"]) != 1):
        raise ValueError(f"{name}: timestamp or sequence gap")
    keep = t >= 2.0  # Same startup guard for all recordings; turns begin later.
    axes = np.column_stack([data[c] for c in ("gx_raw", "gy_raw", "gz_raw",
                                               "ax_raw", "ay_raw", "az_raw")])
    axes = axes.astype(float)
    axes[:, :3] *= meta["dps_per_count"]
    axes[:, 3:] *= meta["g_per_count"]
    return t[keep], axes[keep], len(t)


def integrate(rate, t):
    return float(np.sum((rate[1:] + rate[:-1]) * np.diff(t) * 0.5))


def calibrate(dataset):
    static = dataset["static10"][1]
    gyro_bias = static[:, :3].mean(axis=0)
    noise = static.std(axis=0, ddof=1)
    accel_bias, accel_scale = [], []
    for i, axis in enumerate("xyz"):
        hi = np.median(dataset["face_p" + axis][1][:, 3+i])
        lo = np.median(dataset["face_n" + axis][1][:, 3+i])
        accel_bias.append((hi + lo) / 2)
        accel_scale.append((hi - lo) / 2)
    vertical = (static[:, 3:].mean(axis=0) - accel_bias) / accel_scale
    vertical /= np.linalg.norm(vertical)
    roll = math.degrees(math.atan2(vertical[1], vertical[2]))
    pitch = math.degrees(-math.asin(vertical[0]))
    z_turns = []
    for name in ("cw1", "cw2", "ccw1", "ccw2"):
        t, axes, _ = dataset[name]
        z_turns.append(abs(integrate(axes[:, 2] - gyro_bias[2], t)))
    # Effective Z gain for a hand-commanded 360° rotation on a level table.
    z_gain = float(np.mean(z_turns)) / (360 * vertical[2])
    return dict(gyro_bias_dps=gyro_bias.tolist(), accel_bias_g=accel_bias,
                gyro_noise_dps=noise[:3].tolist(), accel_noise_g=noise[3:].tolist(),
                gyro_scale=[1., 1., z_gain], accel_scale=accel_scale,
                mount_rpy_deg=[roll, pitch, 0.])


def summarize(dataset, fit):
    roll, pitch, _ = np.radians(fit["mount_rpy_deg"])
    vertical = np.array([-np.sin(pitch), np.sin(roll)*np.cos(pitch), np.cos(roll)*np.cos(pitch)])
    summary = {}
    for name, (t, a, count) in dataset.items():
        result = dict(count=count, used=len(t), rate_hz=(len(t)-1)/(t[-1]-t[0]))
        if name.startswith(("cw", "ccw")):
            corrected = (a[:, :3] - fit["gyro_bias_dps"]) / fit["gyro_scale"]
            angle = integrate(corrected @ vertical, t)
            result.update(angle_deg=angle, error_deg=angle-(360 if name.startswith("ccw") else -360))
        elif name.startswith("static"):
            mean = a.mean(axis=0)
            result.update(mean_dps_g=mean.tolist(), std_dps_g=a.std(axis=0, ddof=1).tolist(),
                          corrected_yaw_drift_deg=integrate(
                              ((a[:, :3] - fit["gyro_bias_dps"]) / fit["gyro_scale"]) @ vertical, t),
                          accel_residual_m_s2=((mean[3:] - fit["accel_bias_g"] -
                                               vertical * fit["accel_scale"])*G).tolist())
        summary[name] = result
    return summary


def compare_static(validation, fit):
    for key, inferred in fit.items():
        if not np.allclose(imu_defaults()[key], inferred, rtol=0., atol=1e-8):
            raise ValueError(f"Fitted {key} differs from the simulator default")
    t, observed, _ = validation["static2"]
    sensor = ImuSensor({}, "Nominal", random.Random(20261128))
    vehicle = SimpleNamespace(body=SimpleNamespace(angle=0., angular_velocity=0., velocity=(0., 0.)))
    simulated = []
    for now, dt in zip(t, np.diff(t, prepend=t[0])):
        sample = sensor.sample(vehicle, now, dt)
        simulated.append([*np.degrees(sample.gyro_rad_s), *(np.asarray(sample.accel_m_s2) / G)])
    return dict(seed=20261128, mean_simulated_dps_g=np.mean(simulated, axis=0).tolist(),
                std_simulated_dps_g=np.std(simulated, axis=0, ddof=1).tolist(),
                mean_sim_minus_real_dps_g=(np.mean(simulated, axis=0)-observed.mean(axis=0)).tolist())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=ROOT / "imu_data")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    training = {name: load(args.data, name) for name in FIT}
    fit = calibrate(training)
    validation = {name: load(args.data, name) for name in HOLDOUT}
    report = dict(parameters=fit, development=summarize(training, fit),
                  validation=summarize(validation, fit), simulated_static2=compare_static(validation, fit))
    output = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.out:
        args.out.write_text(output, encoding="utf-8")
    else:
        print(output, end="")


if __name__ == "__main__":
    main()
