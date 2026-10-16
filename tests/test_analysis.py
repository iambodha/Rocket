import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from analyze_flight import read_samples, summarize

HEADER = 'Time(ms),AccelX(g),AccelY(g),AccelZ(g),Pressure(Pa)\n'


class FlightAnalysisTests(unittest.TestCase):
    def read(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'input.csv'
            path.write_text(HEADER + rows)
            return read_samples(path)

    def test_uses_elapsed_time_and_all_axes(self):
        result = summarize(self.read('100,3,4,0,101000\n120,0,0,1,100900\n160,0,0,1,100800\n'))
        self.assertAlmostEqual(result['duration_s'], .06)
        self.assertAlmostEqual(result['median_sample_rate_hz'], 1/.03)
        self.assertEqual(result['peak_measured_acceleration_g'], 5)
        self.assertEqual(result['minimum_pressure_pa'], 100800)

    def test_rejects_invalid_exports(self):
        for rows in ('', '0,0,0,1,1\n', '0,0,0,1,1\n0,0,0,1,1\n',
                     '1,0,0,1,1\n0,0,0,1,1\n', '0,nan,0,1,1\n20,0,0,1,1\n',
                     '0,0,0,1,0\n20,0,0,1,1\n', '0,no,0,1,1\n20,0,0,1,1\n'):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.read(rows)

    def test_example_cli_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            result = subprocess.run([sys.executable, 'analyze_flight.py', 'examples/synthetic-flight.csv', '--json', str(path/'summary.json'), '--svg', str(path/'plot.svg')], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertGreater(json.loads((path/'summary.json').read_text())['samples'], 2)
            self.assertIn('<svg', (path/'plot.svg').read_text())

    def test_missing_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bad.csv'
            path.write_text('time,value\n0,1\n')
            with self.assertRaisesRegex(ValueError, 'Missing CSV columns'):
                read_samples(path)
