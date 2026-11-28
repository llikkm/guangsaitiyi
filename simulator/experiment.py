"""JSON Lines experiment format and dependency-free offline analysis."""
from dataclasses import asdict
import argparse
import json
import math
from pathlib import Path


def sample(w, commands):
    cars = []
    for i,car in enumerate(w.cars):
        cars.append(dict(truth=[*car.body.position,car.body.angle],
                         estimate=w.estimates[i].pose, counts=w.estimates[i].counts,
                         wheel_mm_s=car.speed.copy(), gyro_rad_s=w.gyros[i],
                         command=commands[i], scan=w.scans[i],
                         scan_tick=w.scan_tick, scan_pose=w.scan_poses[i]))
        if hasattr(w, "wall_fixes"):
            fix = w.wall_fixes[i]
            cars[-1]["wall_fix"] = asdict(fix) if fix else None
        if getattr(w, "sensor_config", None) is not None:
            obs = w.observation(i)
            scan = obs.lidar_sample
            cars[-1].update(scan_tick=w.suites[i].delivery_tick,
                            imu_sample=asdict(obs.imu_sample) if obs.imu_sample else None,
                            encoder_sample=asdict(obs.encoder_sample) if obs.encoder_sample else None,
                            beam_times=scan.times if scan else (),
                            scan_time=obs.lidar_time if scan else None,
                            scan_available_time=scan.available_time if scan else None)
        if hasattr(car,"wheels"):
            cars[-1]["latched_pwm"] = car.duty
            cars[-1]["wheels"] = [vars(wheel).copy() for wheel in car.wheels]
            cars[-1]["body_velocity_mm_s"] = list(car.body.velocity)
            cars[-1]["angular_velocity_rad_s"] = car.body.angular_velocity
    return dict(tick=w.elapsed, round=w.match.round, clock=w.match.tick,
                score=w.match.scores.copy(), result=w.match.result, cars=cars,
                pieces=[[t,k,*b.position] for t,k,b in w.pieces])


def save(path, config, seed, samples, timeline=None):
    with Path(path).open("x",encoding="utf8") as out:
        print(json.dumps(dict(format=2,software="1.1",config=config,seed=seed)),file=out)
        if timeline is not None:
            for event in timeline.events:
                print(json.dumps(event,separators=(",",":")),file=out)
        for row in samples:
            print(json.dumps(row,separators=(",",":")),file=out)


def analyze(path):
    errors = [[],[]]
    with open(path,encoding="utf8") as stream:
        header = json.loads(next(stream))
        for line in stream:
            row = json.loads(line)
            if "action" in row or "end_tick" in row:
                continue
            for i,car in enumerate(row["cars"]):
                errors[i].append(math.dist(car["truth"][:2],car["estimate"][:2]))
    return dict(seed=header["seed"], samples=len(errors[0]),
                rmse_mm=[math.sqrt(sum(x*x for x in e)/len(e)) if e else None for e in errors])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    print(json.dumps(analyze(parser.parse_args().path),indent=2))
