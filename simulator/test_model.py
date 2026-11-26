import unittest
from control import navigate
from rules import config
from sensors import Odometry
from match import Match


class ModelTest(unittest.TestCase):
    def test_odometry(self):
        c = config()
        od = Odometry((0,0,0),c)
        od.update((1320,1320),0,1)
        self.assertAlmostEqual(od.pose[0],204.20352248)
        self.assertEqual(od.pose[1],0)

    def test_nav(self):
        c = config()
        self.assertTrue(navigate((0,0,0),(1,1),c)[1])
        (l,r),done = navigate((0,0,0),(0,500),c)
        self.assertLess(l,r)
        self.assertFalse(done)

    def test_reversible_score(self):
        m = Match(config())
        m.update([(0,4,200,1000),(1,0,200,1000)])
        self.assertEqual(m.scores,[5,0])
        m.update([(0,4,366.5,1000)])
        self.assertEqual(m.scores,[0,0])

    def test_simultaneous(self):
        m = Match(dict(config(),mass_b=1.1))
        m.new_round()
        m.update([(0,0,200,1000),(1,6,2800,1000)])
        self.assertEqual(m.result,"B wins")
