import argparse
from collections import deque
from datetime import datetime
from pathlib import Path

import pygame as pg
from field_view import field_view
from control import navigate, clamp
from experiment import sample, save
from physics import World
from rules import config
from units import degrees
from playback import DIVISORS, FRAME_RATES, StepClock, Timeline, load_replay


def run(c, seed, replay=None, fps=60):
    pg.init()
    screen = pg.display.set_mode((1180,680))
    pg.key.stop_text_input()
    pg.display.set_caption("Optical Chess | Differential Drive Lab")
    font = pg.font.Font(None, 23)
    small = pg.font.Font(None, 16)
    timer = pg.time.Clock()
    w = World(c,seed)
    timeline = Timeline()
    clock = StepClock(c["rate"])
    divisor = 1
    team = 0
    paused = True
    braking = False
    target = None
    paths = [deque(maxlen=2000),deque(maxlen=2000)]
    log = []
    message = ""
    px,draw = field_view(screen,c,small)
    scale = min(840/c["width"],580/c["height"])
    left,base = 24,630

    def label(text,x,y,color=(223,232,235)):
        screen.blit(font.render(str(text),True,color),(x,y))

    controls = [
        ("Play/Pause", pg.K_p), ("Brake", pg.K_SPACE), ("Team", pg.K_TAB),
        ("Speed", pg.K_t), ("FPS", pg.K_f), ("Bumper", pg.K_b),
        ("Reset", pg.K_r), ("Save", pg.K_s), ("Replay", pg.K_v),
        ("Unjam", pg.K_j), ("-5", pg.K_MINUS)]
    buttons = [(pg.Rect(882+(i%3)*97,435+(i//3)*34,93,30), name, key)
               for i,(name,key) in enumerate(controls)]
    physical = {getattr(pg, "KSCAN_"+letter.upper()): ord(letter) for letter in "pjrsbtfv"}
    running = True
    while running:
        delta = timer.tick(fps)
        for e in pg.event.get():
            if e.type == pg.QUIT:
                running = False
            elif e.type == pg.MOUSEBUTTONDOWN and e.button == 1:
                clicked = next((key for rect,_,key in buttons if rect.collidepoint(e.pos)),None)
                if clicked is not None:
                    e = pg.event.Event(pg.KEYDOWN, key=clicked)
                elif replay is None:
                    x,y = (e.pos[0]-left)/scale,(base-e.pos[1])/scale
                    if 0 <= x <= c["width"] and 0 <= y <= c["height"]:
                        target = (x,y)
            if e.type == pg.KEYDOWN:
                e.key = physical.get(getattr(e,"scancode",None), e.key)
                if e.key == pg.K_t:
                    divisor = DIVISORS[(DIVISORS.index(divisor)+1)%len(DIVISORS)]
                if e.key == pg.K_f:
                    fps = FRAME_RATES[(FRAME_RATES.index(fps)+1)%len(FRAME_RATES)]
                if e.key == pg.K_v:
                    source = timeline if replay is None else replay
                    if source.events:
                        replay = Timeline(source.events)
                        w = World(c,seed)
                        target,paused = None,False
                        clock = StepClock(c["rate"])
                        delta = 0
                        for trail in paths: trail.clear()
                if e.key == pg.K_p:
                    paused = not paused
                    delta = 0
                    if replay is None: timeline.record("pause",paused)
                if e.key == pg.K_ESCAPE: running = False
                if e.key == pg.K_TAB:
                    team = 1-team
                    target = None
                if replay is not None and e.key == pg.K_r:
                    replay = None
                    timeline = Timeline()
                    log = []
                if e.key == pg.K_s:
                    if replay is not None and not timeline.events:
                        message = "Playing a saved run"
                    else:
                        path = Path("runs")/datetime.now().strftime("%Y%m%d-%H%M%S-%f.jsonl")
                        path.parent.mkdir(exist_ok=True)
                        save(path,c,seed,log,timeline)
                        message = path.name
                if replay is not None:
                    continue
                if e.key == pg.K_b:
                    if paused and w.elapsed == 0 and not timeline.events:
                        modes = ("flat","u","custom")
                        mode = modes[(modes.index(c["bumper"])+1)%len(modes)]
                        candidate = dict(c,bumper=mode)
                        try:
                            w = World(candidate,seed)
                            c = candidate
                        except ValueError as error:
                            message = str(error)
                    else:
                        message = "Select bumper before starting a new run"
                if e.key == pg.K_SPACE:
                    target = None
                    braking = True
                if e.key == pg.K_j and not w.match.result:
                    paused = True
                    target = None
                    timeline.record("unjam")
                    clock = StepClock(c["rate"])
                    w.reset_cars()
                    for trail in paths: trail.clear()
                if e.key == pg.K_r:
                    timeline = Timeline()
                    w = World(c,seed)
                    paths = [deque(maxlen=2000),deque(maxlen=2000)]
                    target,paused,log,clock = None,True,[],StepClock(c["rate"])
                    message = "Reset"
                if e.key == pg.K_MINUS and not w.match.result:
                    timeline.record("penalty",team)
                    w.match.penalize(team)
        keys = pg.key.get_pressed()
        forward = int(keys[pg.K_UP])-int(keys[pg.K_DOWN])
        turn = int(keys[pg.K_LEFT])-int(keys[pg.K_RIGHT])
        if braking:
            braking = bool(forward or turn)
            forward = turn = 0
        steps = clock.advance(delta, divisor) if not paused else 0
        for _ in range(steps):
            if replay is not None:
                replay.advance(w)
                if replay.index == len(replay.events):
                    paused = True
                    message = "Replay finished"
                if w.scan_tick == w.elapsed:
                    for i,car in enumerate(w.cars): paths[i].append(px(*car.body.position))
                if paused: break
                continue
            if w.match.result: break
            commands = [(0.,0.),(0.,0.)]
            if forward or turn:
                target = None
                commands[team] = tuple(c["speed"]*clamp(v,1) for v in (forward-turn,forward+turn))
            elif target:
                commands[team],arrived = navigate(w.estimates[team].pose,target,c)
                if arrived: target = None
            previous_round = w.match.round
            timeline.record("drive",commands)
            w.step(commands)
            if w.match.round != previous_round:
                target = None
                for trail in paths: trail.clear()
            if w.scan_tick == w.elapsed or w.match.result:
                log.append(sample(w,commands))
                for i,car in enumerate(w.cars):
                    paths[i].append(px(*car.body.position))
        screen.fill((20,31,40))
        label("OPTICAL CHESS    /    MOTION LAB",left,16)
        draw(w,team,paths,target)
        car = w.cars[team]
        m = w.match
        x,y = car.body.position
        ex,ey,eh = w.estimates[team].pose
        rows = [f"Team {'AB'[team]}    Seed {seed}",m.result or ("PAUSED" if paused else "RUNNING"),
                f"{'Golden '+str(m.round) if m.round else 'Regular'}   {m.duration-m.tick/c['rate']:.1f} s",
                f"A {m.scores[0]}    B {m.scores[1]}","",
                f"Truth mm  {x:.0f}, {y:.0f}",f"Yaw       {degrees(car.body.angle)%360:.1f} deg",
                f"Est mm    {ex:.0f}, {ey:.0f}",f"Est yaw   {degrees(eh)%360:.1f} deg",
                f"Wheels    {car.speed[0]:.0f}, {car.speed[1]:.0f} mm/s",
                f"Encoder   {w.estimates[team].counts}",f"Gyro      {w.gyros[team]:.3f} rad/s","",
                f"{'Replay' if replay is not None else 'Live'}  1/{divisor}x   {fps} FPS",
                f"Bumper: {c['bumper']}",
                f"Body speed: {car.body.velocity.length:.1f} mm/s"]
        for i,row in enumerate(rows): label(row,885,47+24*i)
        for rect,name,_ in buttons:
            pg.draw.rect(screen,(57,82,96),rect,border_radius=4)
            label(name,rect.x+6,rect.y+6)
        label("Arrows: drive   Click field: navigate",882,585)
        label(message,882,616)
        pg.display.flip()
    pg.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed",type=int,default=42)
    parser.add_argument("--config")
    parser.add_argument("--replay")
    parser.add_argument("--fps",type=int,choices=FRAME_RATES,default=60)
    args = parser.parse_args()
    if args.replay:
        header, replay = load_replay(args.replay)
        run(header["config"],header["seed"],replay,args.fps)
    else:
        run(config(args.config),args.seed,fps=args.fps)
