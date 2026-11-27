import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import pygame as pg
from experiment import analyze, sample, save
from main import run
from physics import World
from playback import DIVISORS, FRAME_RATES, StepClock, Timeline, load_replay
from rules import config
from units import counts_to_travel, degrees, radians, travel_to_turns, turns_to_counts


class MotionTests(unittest.TestCase):
    def test_release_stops_and_stays_stopped(self):
        for mu in (0.105, 0.35):
            w = World(dict(config(),piece_mu=mu),42)
            b = w.pieces[0][2]
            b.position = (600,500)
            b.velocity = (200,0)
            for _ in range(120):
                w.step([(0,0)]*2)
            distance = b.position.x-600
            self.assertAlmostEqual(distance,200**2/(2*mu*9810),delta=200/120)
            self.assertEqual(b.velocity.length,0)
            self.assertGreater(distance,0)

    def test_car_acceleration_and_braking(self):
        w = World(config(),42)
        car = w.cars[0]
        w.step([(300,300),(0,0)])
        self.assertLess(car.body.velocity.length,300)
        for _ in range(90): w.step([(300,300),(0,0)])
        x = car.body.position.x
        self.assertGreater(car.body.velocity.x,290)
        for _ in range(120): w.step([(0,0)]*2)
        self.assertLess(car.body.velocity.length,1)
        self.assertTrue(0 < car.body.position.x-x < 100)

    def test_retention_on_same_turn(self):
        counts = {}
        for mode in ("flat","u","custom"):
            w = World(dict(config(),bumper=mode),42)
            car = w.cars[0]
            for (_,_,b),y in zip(w.pieces,(-36,36)):
                b.position = car.body.local_to_world((146,y))
            for tick in range(180):
                w.step([((220,220) if tick<60 else (40,260)),(0,0)])
            points = [car.body.world_to_local(b.position) for _,_,b in w.pieces[:2]]
            counts[mode] = sum(110 <= x <= 180 and abs(y)<=100 for x,y in points)
        self.assertEqual(counts,{"flat":0,"u":2,"custom":2})

    def test_custom_envelope(self):
        with self.assertRaises(ValueError):
            World(dict(config(),bumper="custom",custom_outline=[[0,0],[300,0]]),1)


class ReplayTests(unittest.TestCase):
    def test_units(self):
        distance = -math.pi*65
        counts = turns_to_counts(travel_to_turns(distance,65),1320)
        self.assertEqual(counts,-1320)
        self.assertAlmostEqual(counts_to_travel(counts,65,1320),distance)
        self.assertEqual(degrees(radians(135)),135)

    def test_clock_all_speeds(self):
        for fps in FRAME_RATES:
            for divisor in DIVISORS:
                clock = StepClock(120)
                ticks = sum(clock.advance(round((i+1)*1000/fps)-round(i*1000/fps),divisor)
                            for i in range(fps))
                self.assertEqual(ticks,120//divisor)

    def test_disk_replay_every_step(self):
        c = dict(config(),bumper="u")
        w = World(c,42)
        tape = Timeline()
        states = []
        for i in range(180):
            if i == 60:
                tape.record("penalty",1)
                w.match.penalize(1)
            if i == 120:
                tape.record("unjam")
                w.reset_cars()
            cmd = [(200,i*2),(0,0)]
            tape.record("drive",cmd)
            w.step(cmd)
            states.append(sample(w,cmd))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"run.jsonl"
            save(path,c,42,states[-1:],tape)
            self.assertEqual(analyze(path)["samples"],1)
            for fps in (30,60):
                header,replay = load_replay(path)
                other = World(header["config"],header["seed"])
                clock = StepClock(120)
                frame = 0
                while replay.tick < len(states):
                    ms = round((frame+1)*1000/fps)-round(frame*1000/fps)
                    frame += 1
                    for _ in range(clock.advance(ms,10)):
                        if replay.tick == len(states): break
                        replay.advance(other)
                        i = replay.tick-1
                        self.assertEqual(sample(other,[(200,i*2),(0,0)]),states[i])

    def test_ime_events_and_mouse_buttons(self):
        # Inject a physical P with no Latin keycode, followed by an IME event.
        from main import World as RealWorld
        worlds = []
        def build(*args):
            w = RealWorld(*args)
            worlds.append(w)
            return w
        frame = 0
        flip = pg.display.flip
        def present():
            nonlocal frame
            flip()
            if frame == 0:
                pg.event.post(pg.event.Event(pg.MOUSEBUTTONDOWN,button=1,pos=(1110,480)))
                pg.event.post(pg.event.Event(pg.TEXTEDITING,text="中",start=0,length=1))
                pg.event.post(pg.event.Event(pg.KEYDOWN,key=pg.K_UNKNOWN,scancode=pg.KSCAN_P))
            if frame == 5: pg.event.post(pg.event.Event(pg.QUIT))
            frame += 1
        with patch("main.World",side_effect=build), patch.object(pg.display,"flip",side_effect=present):
            run(config(),42)
        self.assertEqual(worlds[-1].c["bumper"],"u")
        self.assertGreater(worlds[-1].elapsed,0)

    def test_native_window_replay_at_two_frame_rates(self):
        c = config()
        original = World(c,7)
        tape = Timeline()
        for i in range(24):
            commands = [(120+i,220),(0,0)]
            tape.record("drive",commands)
            original.step(commands)
        expected = sample(original,commands)
        flip = pg.display.flip
        for fps in (30,60):
            actual = World(c,7)
            replay = Timeline(tape.events)
            frames = 0
            def present():
                nonlocal frames
                flip()
                frames += 1
                if frames == 1:
                    pg.event.post(pg.event.Event(pg.KEYDOWN,key=pg.K_p))
                if replay.index == len(tape.events) or frames >= 2*fps:
                    pg.event.post(pg.event.Event(pg.QUIT))
            with patch("main.World",return_value=actual), patch.object(pg.display,"flip",side_effect=present):
                run(c,7,replay,fps)
            self.assertEqual(replay.tick,24)
            self.assertEqual(sample(actual,commands),expected)
