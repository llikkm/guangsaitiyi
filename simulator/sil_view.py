"""Observer for the V2 experiment runner."""
from collections import deque
import pygame as pg
from field_view import field_view


def watch(sim,advance,speed):
    w = sim.world
    fps = sim.options["fps"]
    if type(fps) is not int or fps <= 0:
        raise ValueError("display.fps must be a positive integer")
    pg.init()
    try:
        screen = pg.display.set_mode((1180,680))
        pg.display.set_caption("Optical Chess | V2.0 controller experiment")
        pg.key.stop_text_input()
        font = pg.font.Font(None,24)
        px,draw = field_view(screen,w.c,pg.font.Font(None,16))
        trails = [deque(maxlen=2000),deque(maxlen=2000)]
        timer = pg.time.Clock()
        team, credit, paused = 0, 0., False
        running = True
        while running:
            ms = timer.tick(fps)
            for event in pg.event.get():
                if event.type == pg.QUIT: return
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_ESCAPE: return
                    if event.key == pg.K_TAB: team = 1-team
                    if event.key == pg.K_SPACE: paused = not paused
                    if event.key == pg.K_DOWN: speed = max(1/60,speed/2)
                    if event.key == pg.K_UP: speed = min(64,speed*2)
            if not paused:
                credit += ms*w.c["rate"]*speed/1000
            while credit >= 1:
                credit -= 1
                if not advance():
                    running = False
                    break
                if w.elapsed == w.scan_tick:
                    for car,path in zip(w.cars,trails):
                        path.append(px(*car.body.position))
            screen.fill((20,31,40))
            draw(w,team,trails)
            obs = w.observation(team)
            x,y,h = obs.estimated_pose
            rows = ["V2.0  Controller experiment",f"Team {'AB'[team]}    {obs.time:.3f} s",
                    f"Estimated xy  {x:.1f}, {y:.1f}",f"Estimated yaw  {h:.3f} rad",
                    f"PWM   {sim.pwm[team][0]:.3f}, {sim.pwm[team][1]:.3f}",
                    f"Encoders   {obs.encoder_counts}",f"Gyro   {obs.gyro_rad_s:.3f} rad/s",
                    f"Score   {w.match.scores}",f"{speed:g}x    {fps} FPS",
                    "Space  pause     Tab  team","Up / Down  speed","Esc  close"]
            for i,row in enumerate(rows):
                screen.blit(font.render(row,True,(223,232,235)),(880,48+30*i))
            pg.display.flip()
    finally:
        pg.quit()
