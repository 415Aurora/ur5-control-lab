from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src/ur5_control_lab/src"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from ur5_control_lab.core.experiments import (  # noqa: E402
    ExperimentConfig,
    ExperimentRecorder,
    export_experiment_record,
    format_metrics_summary,
)


def test_experiment_config_requires_six_joint_targets() -> None:
    with pytest.raises(ValueError, match="6 target joint values"):
        ExperimentConfig(name="bad", target_joints_deg=[0.0, 1.0])


def test_experiment_recorder_computes_metrics_from_samples() -> None:
    recorder = ExperimentRecorder()
    config = ExperimentConfig(
        name="baseline-run",
        target_joints_deg=[0.0, 5.0, 10.0, 15.0, 20.0, 25.0],
        duration_seconds=2.5,
        sample_rate_hz=10.0,
    )

    recorder.start(config, started_at=100.0)
    recorder.add_sample(
        timestamp=100.0,
        joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joint_velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ee_position=[0.0, 0.0, 0.0],
    )
    recorder.add_sample(
        timestamp=101.0,
        joint_positions=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
        joint_velocities=[0.0, 0.3, 0.4, 0.5, 0.6, 0.7],
        ee_position=[0.1, 0.0, 0.0],
    )
    recorder.add_sample(
        timestamp=102.0,
        joint_positions=[0.0, 0.15, 0.25, 0.35, 0.45, 0.55],
        joint_velocities=[0.0, 0.2, 0.1, 0.5, 0.3, 0.2],
        ee_position=[0.1, 0.1, 0.0],
    )

    record = recorder.stop(completed_at=102.0)

    assert record.metrics["sample_count"] == 3
    assert record.metrics["actual_duration_seconds"] == pytest.approx(2.0)
    assert record.metrics["joint_path_length_rad"] == pytest.approx(1.75)
    assert record.metrics["ee_path_length_m"] == pytest.approx(0.2)
    assert record.metrics["max_joint_speed_rad_s"] == pytest.approx(0.7)
    assert record.metrics["mean_joint_speed_rad_s"] == pytest.approx(3.8 / 18.0)
    assert record.metrics["final_joint_error_rad"] == pytest.approx(
        math.sqrt(sum((actual - target) ** 2 for actual, target in zip(
            [0.0, 0.15, 0.25, 0.35, 0.45, 0.55],
            [math.radians(v) for v in [0.0, 5.0, 10.0, 15.0, 20.0, 25.0]],
        )))
    )


def test_export_experiment_record_writes_json_and_csv(tmpdir) -> None:
    recorder = ExperimentRecorder()
    config = ExperimentConfig(
        name="export-run",
        target_joints_deg=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        duration_seconds=1.0,
    )

    recorder.start(config, started_at=10.0)
    recorder.add_sample(
        timestamp=10.0,
        joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joint_velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ee_position=[0.0, 0.0, 0.0],
    )
    record = recorder.stop(completed_at=10.5)

    exported = export_experiment_record(record, Path(str(tmpdir)))

    json_path = exported["json"]
    csv_path = exported["csv"]

    payload = json.loads(json_path.read_text())
    assert payload["config"]["name"] == "export-run"
    assert payload["metrics"]["sample_count"] == 1
    assert payload["samples"][0]["joint_positions"][0] == 0.0

    with csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))

    assert len(rows) == 1
    assert rows[0]["elapsed_seconds"] == "0.000000"
    assert rows[0]["joint_1_position_rad"] == "0.000000"


def test_format_metrics_summary_is_ready_for_gui_display() -> None:
    summary = format_metrics_summary(
        {
            "sample_count": 12.0,
            "actual_duration_seconds": 3.2,
            "joint_path_length_rad": 1.75,
            "ee_path_length_m": 0.42,
            "max_joint_speed_rad_s": 0.7,
            "mean_joint_speed_rad_s": 0.2,
            "final_joint_error_rad": 0.05,
        }
    )

    assert "Samples: 12" in summary
    assert "Duration: 3.20 s" in summary
    assert "Joint path: 1.750 rad" in summary
    assert "EE path: 0.420 m" in summary
    assert "Final error: 0.050 rad" in summary
