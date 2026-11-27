"""V2 algorithm runner. Use --gui for the live view; default runs unpaced."""
import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path

import pymunk
from experiment import sample
from sil_core import REFERENCE, Simulation, settings


def write_line(stream, value):
    stream.write(json.dumps(value,allow_nan=False,separators=(",",":"))+"\n")


def header(sim):
    return dict(format=3,software="2.0",pymunk=pymunk.version,seed=sim.world.seed,
                options=sim.options,controller_sha256=sim.hashes)


def inputs(stream):
    for line in stream:
        row = json.loads(line)
        if row.get("action") == "drive":
            yield row["input"]
        elif "end_tick" in row:
            return row["end_tick"]
        elif "cars" not in row:
            raise ValueError("Unknown experiment record")
    raise ValueError("Incomplete experiment: missing end_tick")


def execute(sim, count, stream=None, recorded=None, gui=False, speed=1, on_step=None):
    if stream:
        write_line(stream,header(sim))
    w = sim.world

    def advance():
        if recorded is not None:
            try:
                row = next(recorded)
            except StopIteration as end:
                if end.value != w.elapsed:
                    raise ValueError("Replay length mismatch")
                return False
        else:
            row = None
            if w.match.result or w.elapsed >= count:
                return False
        applied = sim.step(row)
        if stream:
            write_line(stream,dict(action="drive",input=applied))
            if w.scan_tick == w.elapsed or w.match.result or w.elapsed == count:
                write_line(stream,sample(w,applied["commands"]))
        if on_step:
            on_step(w)
        return True

    if gui:
        from sil_view import watch
        watch(sim,advance,speed)
    else:
        while advance():
            pass
    if stream:
        write_line(stream,dict(end_tick=w.elapsed))
    return dict(ticks=w.elapsed,seconds=w.elapsed/w.c["rate"],score=w.match.scores,
                poses=[[*car.body.position,car.body.angle] for car in w.cars])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=REFERENCE)
    parser.add_argument("--seed",type=int,default=42)
    parser.add_argument("--seconds",type=float,default=4)
    parser.add_argument("--gui",action="store_true")
    parser.add_argument("--speed",type=float,default=1,help="GUI simulation speed multiplier")
    parser.add_argument("--record",type=Path)
    parser.add_argument("--replay",type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("Duration and speed must be positive and finite")
    with ExitStack() as stack:
        recorded = None
        if args.replay:
            if args.record:
                parser.error("Choose either --record or --replay")
            source = stack.enter_context(args.replay.open(encoding="utf8"))
            meta = json.loads(source.readline())
            if (meta.get("format"),meta.get("software"),meta.get("pymunk")) != (3,"2.0",pymunk.version):
                parser.error("Expected a compatible V2.0 experiment")
            sim = Simulation(meta["options"],meta["seed"],replay=True)
            recorded = inputs(source)
        else:
            sim = Simulation(settings(args.config),args.seed)
        out = None
        if args.record:
            args.record.parent.mkdir(parents=True,exist_ok=True)
            out = stack.enter_context(args.record.open("x",encoding="utf8"))
        result = execute(sim,round(args.seconds*sim.world.c["rate"]),out,recorded,args.gui,args.speed)
        print(json.dumps(result))


if __name__ == "__main__":
    main()
