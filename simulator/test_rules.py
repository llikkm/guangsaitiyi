import math
import unittest
from match import Match
from rules import config, layout


class RulesTest(unittest.TestCase):
    def test_layout(self):
        c = config()
        for seed in range(20):
            pts = list(layout(c,seed))
            self.assertEqual(pts,list(layout(c,seed)))
            self.assertEqual(len(pts),32)
            for team in range(2):
                self.assertEqual(sum(c["points"][k] for t,k,_ in pts if t == team),45)
            for i,(team,kind,p) in enumerate(pts):
                self.assertEqual(team,i//16)
                for value, axis in zip(p, ("width", "height")):
                    self.assertLessEqual(abs(value-c[axis]/2)+c["diameter"]/2,c["scatter"]/2)
                self.assertTrue(all(math.dist(p,q[2]) >= c["diameter"] for q in pts[:i]))

    def test_whole_disk_and_once(self):
        m = Match(config())
        m.update([(0,0,380,1000)])
        self.assertEqual(m.scores[0],0)
        for _ in range(2):
            m.update([(0,0,350,1000)])
        self.assertEqual(m.scores[0],10)

    def test_golden(self):
        m = Match(dict(config(),rate=1,duration=1))
        m.update([])
        self.assertEqual(m.round,1)
        m.update([(1,6,2800,1000)])
        self.assertEqual(m.result,"B wins")

    def test_terminal_settling_and_penalty(self):
        m = Match(dict(config(), rate=1, duration=1))
        m.penalize(0)
        self.assertEqual(m.scores, [-5, 0])
        m.update([], settled=False)
        self.assertFalse(m.result)
        m.update([(0, 0, 200, 1000)], settled=True)
        self.assertEqual(m.scores, [5, 0])
        self.assertEqual(m.result, "A wins")

    def test_equal_mass_repeats_overtime(self):
        m = Match(dict(config(), rate=1, duration=1, overtime=1))
        m.update([])
        m.update([])
        self.assertEqual((m.round, m.tick, m.result), (2, 0, ""))
