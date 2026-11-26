import os
import unittest
from unittest.mock import patch
from rules import config


class EngineTest(unittest.TestCase):
    def setUp(self):
        from physics import World
        self.w = World(config(),42)

    def test_push_and_wall(self):
        w = self.w
        car = w.cars[0]
        piece = w.pieces[0][2]
        piece.position = car.body.position + (210,0)
        for _ in range(1000):
            w.step([(500,500),(0,0)])
        self.assertGreater(piece.position.x,800)
        self.assertLessEqual(piece.position.x,w.c["width"]-w.c["diameter"]/2+2)
        self.assertLess(car.body.position.x,w.c["width"])
        self.assertGreater(w.estimates[0].pose[0],car.body.position.x+300)

    def test_lidar_plane(self):
        from sensors import scan
        w = self.w
        car = w.cars[0]
        car.body.position = (500,500)
        other = w.cars[1]
        other.body.position = (1500,500)
        piece = w.pieces[0][2]
        piece.position = (750,500)
        for b in (car.body,other.body,piece):
            w.space.reindex_shapes_for_body(b)
        beam = w.c["rays"]//2
        readings = scan(w,car)
        import math
        self.assertEqual(len(readings),91)
        self.assertAlmostEqual(math.degrees(readings[0][0]),-135)
        self.assertAlmostEqual(math.degrees(readings[-1][0]),135)
        self.assertAlmostEqual(readings[beam][1],1000-w.c["length"]/2)
        w.c["laser_height"] = w.c["piece_height"]
        self.assertAlmostEqual(scan(w,car)[beam][1],250-w.c["diameter"]/2)
        w.c["laser_height"] = w.c["car_height"]+1
        self.assertAlmostEqual(scan(w,car)[beam][1],2500)
        w.c["laser_height"] = w.c["wall_height"]+1
        self.assertIsNone(scan(w,car)[beam][1])

    def test_turn(self):
        for _ in range(60):
            self.w.step([(-150,150),(0,0)])
        self.assertGreater(self.w.cars[0].body.angle,0)
        l,r = self.w.estimates[0].counts
        self.assertLess(l,0)
        self.assertGreater(r,0)

    def test_unjam(self):
        w = self.w
        for _ in range(30):
            w.step([(200, 200), (0, 0)])
        positions = [b.position for _, _, b in w.pieces]
        tick = w.match.tick
        w.reset_cars()
        self.assertEqual(positions, [b.position for _, _, b in w.pieces])
        self.assertEqual(w.match.tick, tick)
        self.assertEqual(tuple(w.cars[0].body.position), (200, 200))
        self.assertEqual(w.estimates[0].counts, (0, 0))

    def test_piece_contact_and_drag(self):
        w = self.w
        a, b = [p[2] for p in w.pieces[:2]]
        a.position, b.position = (500, 500), (600, 500)
        a.velocity = (400, 0)
        for _ in range(60):
            w.step([(0, 0), (0, 0)])
        self.assertGreater(b.position.x, 610)
        self.assertGreaterEqual(b.position.x-a.position.x, w.c["diameter"]-1)
        self.assertLess(a.velocity.length+b.velocity.length, 250)

    def test_full_match_clock(self):
        w = self.w
        # No wheel input: play the full regular time and golden goal period.
        w.c["mass_b"] = 1.1
        for _ in range(120 * w.c["rate"] - 1):
            w.step([(0, 0), (0, 0)])
        self.assertEqual(w.match.round, 0)
        old_positions = [b.position for _, _, b in w.pieces]
        w.step([(0, 0), (0, 0)])
        self.assertEqual((w.match.round, w.match.tick), (1, 0))
        self.assertNotEqual(old_positions, [b.position for _, _, b in w.pieces])
        for _ in range(60 * w.c["rate"]):
            w.step([(0, 0), (0, 0)])
        self.assertEqual(w.match.result, "B wins")

    def test_display(self):
        import pygame
        from main import run
        flip = pygame.display.flip
        frames = []
        def present():
            surface = pygame.display.get_surface()
            self.assertEqual(surface.get_size(), (1180, 680))
            self.assertNotEqual(surface.get_at((450, 350)), surface.get_at((0, 0)))
            flip()
            frames.append(pygame.display.get_driver())
            pygame.event.post(pygame.event.Event(pygame.QUIT))
        with patch.object(pygame.display, "flip", side_effect=present):
            run(config(), 42)
        self.assertTrue(frames)
        if "SDL_VIDEODRIVER" in os.environ:
            self.assertEqual(frames[0], os.environ["SDL_VIDEODRIVER"])
        print("SDL window driver:", frames[0])
