"""Tests for the metrics and the gain calculator (REQ-G05, G06).
Run: python tools/tests/test_analyze.py"""
import csv
import math
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import analyze as A  # noqa: E402
from host import CSV_HEADER  # noqa: E402


def write_run(rows):
    f = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="")
    w = csv.writer(f)
    w.writerow(CSV_HEADER)
    w.writerows(rows)
    f.close()
    return f.name


def first_order_step(tau, target=50.0, n_pre=50, n_hold=300, n_post=50):
    """Velocity step 0 -> target at 100 Hz, exact first-order response."""
    rows = []
    for k in range(n_pre + n_hold + n_post):
        t = k * 0.01
        if n_pre <= k < n_pre + n_hold:
            ref = target
            v = target * (1 - math.exp(-(t - n_pre * 0.01) / tau))
        else:
            ref, v = 0.0, 0.0
        rows.append([k * 10, 3, 0, 0, v, ref, 0, 0, 0])
    return rows


class TestGainCalculator(unittest.TestCase):
    def test_pole_cancellation(self):
        g = A.pi_from_model(k=0.24, tau=0.05, tau_cl=0.1)
        self.assertAlmostEqual(g["kp_v"], 0.05 / (0.24 * 0.1))
        self.assertAlmostEqual(g["ki_v"], g["kp_v"] / 0.05)
        self.assertAlmostEqual(g["kp_p_max"], 2.0)
        self.assertAlmostEqual(g["t_s"], 0.4)
        self.assertTrue(g["ts_ok"])

    def test_sample_time_warning(self):
        self.assertFalse(A.pi_from_model(0.24, 0.05, 0.05)["ts_ok"])  # Ts = 10 ms > tau_cl / 10


class TestMetrics(unittest.TestCase):
    def test_first_order_step(self):
        tau = 0.1
        path = write_run(first_order_step(tau))
        m = A.metrics(A.load(path))
        os.remove(path)
        self.assertEqual(m["target_mms"], 50.0)
        self.assertAlmostEqual(m["t_r_s"], math.log(9) * tau, delta=0.011)  # 10-90 %: 2.197 tau
        self.assertAlmostEqual(m["t_s_s"], math.log(50) * tau, delta=0.011)  # 2 %: 3.912 tau
        self.assertEqual(m["POT_pct"], 0.0)
        self.assertAlmostEqual(m["e_ss_mms"], 0.0, places=3)

    def test_overshoot(self):
        rows = first_order_step(0.05)
        for r in rows[60:70]:
            r[4] = 55.0  # 10 % above the target for 0.1 s
        path = write_run(rows)
        m = A.metrics(A.load(path))
        os.remove(path)
        self.assertAlmostEqual(m["POT_pct"], 10.0, places=3)
        self.assertFalse(A.passes("POT_pct", m["POT_pct"]))

    def test_move(self):
        rows = []
        for k in range(200):
            running = k < 150
            ref = min(k, 100) * 2.0
            pos = ref - (1.5 if 20 < k < 80 else 0.0)
            rows.append([k * 10, 3 if running else 2, 0, pos, 0, 0, ref if running else pos, 0, 0])
        rows[-1][3] = 199.6  # final position 0.4 mm short of 200 mm
        path = write_run(rows)
        m = A.metrics(A.load(path))
        os.remove(path)
        self.assertAlmostEqual(m["track_max_mm"], 1.5)
        self.assertAlmostEqual(m["final_err_mm"], 0.4, places=6)
        self.assertTrue(A.passes("track_max_mm", m["track_max_mm"]))
        self.assertTrue(A.passes("final_err_mm", m["final_err_mm"]))


if __name__ == "__main__":
    unittest.main()
