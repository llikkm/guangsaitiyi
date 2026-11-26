import argparse
import math
from collections import deque
from datetime import datetime
from pathlib import Path

import pygame as pg
from control import navigate, clamp
from experiment import sample, save
from physics import World
from rules import config


def run(c, seed):
    pg.init()
    screen = pg.display.set_mode((1180,680))
    pg.display.set_caption("Optical Chess | Differential Drive Lab")
    font = pg.font.Font(None, 23)
    small = pg.font.Font(None, 16)
    timer = pg.time.Clock()
    w = World(c,seed)
    team = 0
    paused = True
    braking = False
    target = None
    paths = [deque(maxlen=2000),deque(maxlen=2000)]
    log = []
    message = ""
    scale = min(840/c["width"],580/c["height"])
    left,base = 24,630

    def px(x,y):
        return round(left+x*scale),round(base-y*scale)

    def area(x,y,width,height,color):
        pg.draw.rect(screen,color,(*px(x,y+height),round(width*scale),round(height*scale)))

    def label(text,x,y,color=(223,232,235)):
        screen.blit(font.render(str(text),True,color),(x,y))

    colors = [(236,112,96),(223,231,242)]
    accumulator = 0.
    running = True
    while running:
        delta = timer.tick(60)/1000
        for e in pg.event.get():
            if e.type == pg.QUIT:
                running = False
            elif e.type == pg.MOUSEBUTTONDOWN and e.button == 1:
                x,y = (e.pos[0]-left)/scale,(base-e.pos[1])/scale
                if 0 <= x <= c["width"] and 0 <= y <= c["height"]:
                    target = (x,y)
            elif e.type == pg.KEYDOWN:
                if e.key == pg.K_ESCAPE: running = False
                if e.key == pg.K_TAB:
                    team = 1-team
                    target = None
                if e.key == pg.K_p: paused = not paused
                if e.key == pg.K_SPACE:
                    target = None
                    braking = True
                if e.key == pg.K_j and not w.match.result:
                    paused = True
                    target = None
                    accumulator = 0
                    w.reset_cars()
                    for trail in paths: trail.clear()
                if e.key == pg.K_r:
                    w = World(c,seed)
                    paths = [deque(maxlen=2000),deque(maxlen=2000)]
                    target,paused,log,accumulator = None,True,[],0.
                    message = "Reset"
                if e.key == pg.K_s:
                    path = Path("runs")/datetime.now().strftime("%Y%m%d-%H%M%S-%f.jsonl")
                    path.parent.mkdir(exist_ok=True)
                    save(path,c,seed,log)
                    message = path.name
                if e.key == pg.K_MINUS and not w.match.result:
                    w.match.penalize(team)
        keys = pg.key.get_pressed()
        forward = int(keys[pg.K_UP])-int(keys[pg.K_DOWN])
        turn = int(keys[pg.K_LEFT])-int(keys[pg.K_RIGHT])
        if braking:
            braking = bool(forward or turn)
            forward = turn = 0
        accumulator += delta if not paused and not w.match.result else 0
        while accumulator >= 1/c["rate"] and not paused and not w.match.result:
            commands = [(0.,0.),(0.,0.)]
            if forward or turn:
                target = None
                commands[team] = tuple(c["speed"]*clamp(v,1) for v in (forward-turn,forward+turn))
            elif target:
                commands[team],arrived = navigate(w.estimates[team].pose,target,c)
                if arrived: target = None
            previous_round = w.match.round
            w.step(commands)
            accumulator -= 1/c["rate"]
            if w.match.round != previous_round:
                target = None
                for trail in paths: trail.clear()
            if w.scan_tick == w.elapsed or w.match.result:
                log.append(sample(w,commands))
                for i,car in enumerate(w.cars):
                    paths[i].append(px(*car.body.position))
        screen.fill((20,31,40))
        label("OPTICAL CHESS    /    MOTION LAB",left,16)
        area(0,0,c["width"],c["height"],(73,92,92))
        s = c["scatter"]
        area((c["width"]-s)/2,(c["height"]-s)/2,s,s,(90,107,102))
        for i in range(2):
            x = c["width"]-c["goal_depth"] if i else 0
            area(x,0,c["goal_depth"],c["height"],(66,85,106) if i else (101,78,76))
            area(x,(c["height"]-c["goal_width"])/2,c["goal_depth"],c["goal_width"],
                 (71,93,131) if i else (127,85,81))
            inset = c["start_inset"]
            area(c["width"]-inset-c["start"] if i else inset,
                 c["height"]-inset-c["start"] if i else inset,c["start"],c["start"],(100,108,106))
            if len(paths[i])>1: pg.draw.lines(screen,colors[i],False,list(paths[i]),2)
        car = w.cars[team]
        for angle,distance in w.scans[team]:
            if distance is not None:
                x,y,yaw = w.scan_poses[team]
                a = angle+yaw
                pg.draw.circle(screen,(96,225,175),px(x+distance*math.cos(a),y+distance*math.sin(a)),2)
        for t,k,b in w.pieces:
            p = px(*b.position)
            pg.draw.circle(screen,(25,30,33),p,round(c["diameter"]/2*scale))
            glyph = small.render("GAEHRCP"[k],True,colors[t])
            screen.blit(glyph,glyph.get_rect(center=p))
        for i,bot in enumerate(w.cars):
            pg.draw.polygon(screen,colors[i],[px(*bot.body.local_to_world(v)) for v in bot.shape.get_vertices()])
            pg.draw.line(screen,(22,34,35),px(*bot.body.position),
                         px(*bot.body.local_to_world((c["length"]/2,0))),3)
        if target: pg.draw.circle(screen,(254,213,103),px(*target),6,2)
        m = w.match
        x,y = car.body.position
        ex,ey,eh = w.estimates[team].pose
        rows = [f"Team {'AB'[team]}    Seed {seed}",m.result or ("PAUSED" if paused else "RUNNING"),
                f"{'Golden '+str(m.round) if m.round else 'Regular'}   {m.duration-m.tick/c['rate']:.1f} s",
                f"A {m.scores[0]}    B {m.scores[1]}","",
                f"Truth mm  {x:.0f}, {y:.0f}",f"Yaw       {math.degrees(car.body.angle)%360:.1f} deg",
                f"Est mm    {ex:.0f}, {ey:.0f}",f"Est yaw   {math.degrees(eh)%360:.1f} deg",
                f"Wheels    {car.speed[0]:.0f}, {car.speed[1]:.0f} mm/s",
                f"Encoder   {w.estimates[team].counts}",f"Gyro      {w.gyros[team]:.3f} rad/s","",
                "P  start / pause","Arrows  drive","Click field  navigate","Tab  select car",
                "Space  brake    J  unjam","R  reset    S  save","Minus  referee -5",message]
        for i,row in enumerate(rows): label(row,885,47+25*i)
        pg.display.flip()
    pg.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed",type=int,default=42)
    parser.add_argument("--config")
    args = parser.parse_args()
    run(config(args.config),args.seed)
