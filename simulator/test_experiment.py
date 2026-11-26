from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from experiment import save, analyze


class LogTest(unittest.TestCase):
    def test_known_error(self):
        car = dict(truth=[0,0,0],estimate=[3,4,0])
        with TemporaryDirectory() as temp:
            path = Path(temp)/"run.jsonl"
            save(path,{},42,[dict(cars=[car,car])])
            result = analyze(path)
            self.assertEqual(result["rmse_mm"],[5,5])
            self.assertEqual(result["seed"],42)
