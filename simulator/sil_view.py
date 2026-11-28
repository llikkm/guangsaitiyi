"""Observer for the V2 experiment runner."""
from collections import deque
import pygame as pg
from field_view import field_view
from control import navigate, clamp
from robot_api import MotorCommand, WheelSpeed


def manual_pwm(forward,turn,steering):
    return MotorCommand(clamp(forward-turn*steering,1),clamp(forward+turn*steering,1))


def watch(sim,advance,speed):
    w = sim.world
    fps = sim.options["fps"]
    if type(fps) is not int or fps <= 0:
        raise ValueError("display.fps must be a positive integer")
    pg.init()
    try:
        screen = pg.display.set_mode((1180,680))
        version = "2.1" if w.vehicles else "2.0"
        if w.sensor_config is not None:
            version = "2.2 " + w.sensor_config["mode"]
        pg.display.set_caption(f"Optical Chess | V{version} dynamics")
        pg.key.stop_text_input()
        font = pg.font.Font(None,24)
        px,draw = field_view(screen,w.c,pg.font.Font(None,16))
        trails = [deque(maxlen=2000),deque(maxlen=2000)]
        timer = pg.time.Clock()
        team, credit, paused = 0, 0., False
        running = True
        target = None
        brake_latched = False
        steering = 1.
        slider = pg.Rect(883,640,278,16)
        buttons = [("Pause [Space]",pg.K_SPACE),("Team [Tab]",pg.K_TAB),
                   ("Manual [F2]",pg.K_F2),("Stop [End]",pg.K_END),
                   ("Slower [F3]",pg.K_F3),("Faster [F4]",pg.K_F4),
                   ("FPS [F5]",pg.K_F5),("Close [Esc]",pg.K_ESCAPE)]
        controls = [(pg.Rect(875+i%2*149,466+i//2*38,145,33),name,key)
                    for i,(name,key) in enumerate(buttons)]
        while running:
            ms = timer.tick(fps)
            for event in pg.event.get():
                if event.type == pg.QUIT: return
                if ((event.type == pg.MOUSEBUTTONDOWN and event.button == 1) or
                    (event.type == pg.MOUSEMOTION and event.buttons[0])) and slider.inflate(0,12).collidepoint(event.pos):
                    steering = max(0.,min(1.,(event.pos[0]-slider.left)/(slider.width-1)))
                    continue
                if event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
                    key = next((key for rect,_,key in controls if rect.collidepoint(event.pos)),None)
                    if key is not None:
                        event = pg.event.Event(pg.KEYDOWN,key=key)
                    elif team in sim.manual:
                        scale = min(840/w.c["width"],580/w.c["height"])
                        x,y = (event.pos[0]-24)/scale,(630-event.pos[1])/scale
                        if 0 <= x <= w.c["width"] and 0 <= y <= w.c["height"]:
                            target = (x,y)
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_ESCAPE: return
                    if event.key == pg.K_LEFTBRACKET: steering = max(0.,steering-.05)
                    if event.key == pg.K_RIGHTBRACKET: steering = min(1.,steering+.05)
                    if event.key == pg.K_TAB:
                        if team in sim.manual: sim.manual[team] = MotorCommand(0,0)
                        team = 1-team
                        target = None
                    if event.key == pg.K_SPACE: paused = not paused
                    if event.key == pg.K_F3: speed = max(1/60,speed/2)
                    if event.key == pg.K_F4: speed = min(64,speed*2)
                    if event.key == pg.K_F5:
                        fps = next((f for f in (15,30,60,120) if f > fps),15)
                    if event.key == pg.K_F2 and sim.controllers:
                        target = None
                        if team in sim.manual: del sim.manual[team]
                        else: sim.manual[team] = MotorCommand(0,0)
                    if event.key == pg.K_END and sim.controllers:
                        sim.manual[team] = MotorCommand(0,0)
                        brake_latched = True
                        target = None
            if not paused:
                credit += ms*w.c["rate"]*speed/1000
            while credit >= 1:
                credit -= 1
                if team in sim.manual:
                    keys = pg.key.get_pressed()
                    f = int(keys[pg.K_UP])-int(keys[pg.K_DOWN])
                    t = int(keys[pg.K_LEFT])-int(keys[pg.K_RIGHT])
                    if brake_latched:
                        brake_latched = bool(f or t)
                        f = t = 0
                    if f or t:
                        target = None
                    if target:
                        cmd,arrived = navigate(w.observation(team).navigation_pose,target,w.cars[team].c)
                        sim.manual[team] = WheelSpeed(*cmd)
                        if arrived: target = None
                    else:
                        sim.manual[team] = manual_pwm(f,t,steering)
                if not advance():
                    running = False
                    break
                if w.elapsed == w.scan_tick:
                    for car,path in zip(w.cars,trails):
                        path.append(px(*car.body.position))
            screen.fill((20,31,40))
            draw(w,team,trails,target)
            obs = w.observation(team)
            x,y,h = obs.navigation_pose
            mode = "Manual" if team in sim.manual else ("Algorithm" if sim.controllers else "Replay")
            rows = [f"V{version}  {mode}",f"Team {'AB'[team]}    {obs.time:.3f} s",
                    f"Nav xy mm  {x:.1f}, {y:.1f}",f"Nav yaw  {h:.3f} rad",
                    f"PWM   {sim.pwm[team][0]:.3f}, {sim.pwm[team][1]:.3f}",
                    f"Encoders   {obs.encoder_counts}",f"Gyro   {obs.gyro_rad_s:.3f} rad/s",
                    f"Score   {w.match.scores}",f"{speed:g}x    {fps} FPS",
                    "Arrows: drive   Click: target"]
            fix = obs.wall_fix
            if fix is None:
                rows.append("Lidar: no sample")
            elif fix.pose is None:
                rows.append(f"Lidar: {fix.status}")
            else:
                lx, ly, yaw = fix.pose
                west, east, south, north = fix.wall_distances
                rows.extend([f"Lidar xy mm   {lx:.0f}, {ly:.0f}", f"Lidar yaw   {yaw:.3f} rad",
                             f"Wall W/E mm   {west:.0f} / {east:.0f}",
                             f"Wall S/N mm   {south:.0f} / {north:.0f}"])
            car = w.cars[team]
            if hasattr(car,"wheels"):
                rows += [f"Mass {car.body.mass:.3f} kg   Body {car.body.velocity.length:.1f} mm/s"]
                for label,attr in (("Current A","current"),("Wheel rad/s","omega"),("Slip m/s","slip"),("Traction N","longitudinal_n")):
                    rows.append(label+"   "+" / ".join(f"{getattr(wheel,attr):.2f}" for wheel in car.wheels))
            for i,row in enumerate(rows):
                screen.blit(font.render(row,True,(223,232,235)),(880,22+22*i))
            for rect,name,_ in controls:
                pg.draw.rect(screen,(57,82,96),rect,border_radius=4)
                screen.blit(font.render(name,True,(223,232,235)),(rect.x+5,rect.y+6))
            screen.blit(font.render(f"Steering {steering:.1%}   [ / ]",True,(223,232,235)),(882,616))
            pg.draw.rect(screen,(57,82,96),slider)
            pg.draw.circle(screen,(110,220,180),(slider.left+round(steering*(slider.width-1)),slider.centery),7)
            pg.display.flip()
    finally:
        pg.quit()
