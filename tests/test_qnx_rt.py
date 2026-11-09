"""
Automated Verification Suite for QNX SDP 8.0 Deterministic Edge Watchdog & Sensor Fusion Engine.
"""

from __future__ import annotations

import json
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BIN_PATH = ROOT / "bin" / "qnx_rt_watchdog"


class TestQNXRealtimeEngine(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        subprocess.run(["make"], cwd=str(ROOT), check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def test_binary_exists_and_executes_fault_injection_cycle(self) -> None:
        self.assertTrue(BIN_PATH.exists(), "Compiled binary bin/qnx_rt_watchdog must exist")
        proc = subprocess.run(
            [str(BIN_PATH), "--json", "--cycles", "1000"],
            cwd=str(ROOT),
            check=True,
            capture_output=True,
            text=True,
        )
        data = json.loads(proc.stdout)

        self.assertEqual(data["total_cycles"], 1000)
        self.assertTrue(data["prio_inherit_enabled"])
        self.assertEqual(data["glitches_rejected"], 2)
        self.assertGreaterEqual(data["watchdog_recoveries"], 1)
        self.assertEqual(data["unrecovered_deadline_misses"], 0)
        self.assertLess(data["p99_latency_us"], data["deadline_budget_us"])
        self.assertAlmostEqual(data["final_position_m"], 12.04, delta=0.25)
        self.assertAlmostEqual(data["final_velocity_mps"], 12.08, delta=0.25)

    def test_fault_free_nominal_trajectory(self) -> None:
        proc = subprocess.run(
            [str(BIN_PATH), "--json", "--no-faults", "--cycles", "500"],
            cwd=str(ROOT),
            check=True,
            capture_output=True,
            text=True,
        )
        data = json.loads(proc.stdout)
        self.assertEqual(data["total_cycles"], 500)
        self.assertEqual(data["valid_fusions"], 500)
        self.assertEqual(data["glitches_rejected"], 0)
        self.assertEqual(data["unrecovered_deadline_misses"], 0)
        self.assertLess(data["avg_latency_us"], 250.0)

    def test_codelab_and_tutorial_artifacts_exist(self) -> None:
        codelab_md = ROOT / "CODELAB_TUTORIAL.md"
        codelab_html = ROOT / "codelab" / "index.html"
        self.assertTrue(codelab_md.exists(), "CODELAB_TUTORIAL.md must exist for the $3,500 Best Codelab prize")
        self.assertTrue(codelab_html.exists(), "codelab/index.html interactive tutorial must exist")


if __name__ == "__main__":
    unittest.main()
