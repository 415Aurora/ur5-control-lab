from __future__ import annotations

import csv
import json
import math
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


JOINT_COUNT = 6


def _ensure_joint_vector(values: list[float], field_name: str) -> list[float]:
    if len(values) != JOINT_COUNT:
        raise ValueError(f"{field_name} must contain 6 target joint values")
    return [float(value) for value in values]


def _slugify(value: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "-", value.strip().lower()).strip("-")
    return slug or "experiment"


def _distance(first: list[float], second: list[float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(first, second)))


@dataclass
class ExperimentConfig:
    name: str
    target_joints_deg: list[float]
    duration_seconds: float = 4.0
    planner_id: str = "joint_space_quintic"
    sample_rate_hz: float = 10.0
    notes: str = ""

    def __post_init__(self) -> None:
        self.target_joints_deg = _ensure_joint_vector(
            self.target_joints_deg, "target_joints_deg"
        )
        self.duration_seconds = float(self.duration_seconds)
        self.sample_rate_hz = float(self.sample_rate_hz)
        if self.duration_seconds <= 0.0:
            raise ValueError("duration_seconds must be positive")
        if self.sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")

    @property
    def target_joints_rad(self) -> list[float]:
        return [math.radians(value) for value in self.target_joints_deg]


@dataclass
class ExperimentSample:
    timestamp: float
    joint_positions: list[float]
    joint_velocities: list[float]
    ee_position: list[float]

    def __post_init__(self) -> None:
        self.joint_positions = _ensure_joint_vector(
            self.joint_positions, "joint_positions"
        )
        self.joint_velocities = _ensure_joint_vector(
            self.joint_velocities, "joint_velocities"
        )
        if len(self.ee_position) != 3:
            raise ValueError("ee_position must contain 3 values")
        self.ee_position = [float(value) for value in self.ee_position]
        self.timestamp = float(self.timestamp)


@dataclass
class ExperimentRecord:
    config: ExperimentConfig
    started_at: float
    completed_at: float
    samples: list[ExperimentSample] = field(default_factory=list)
    metrics: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "config": asdict(self.config),
            "started_at": self.started_at,
            "completed_at": self.completed_at,
            "metrics": self.metrics,
            "samples": [asdict(sample) for sample in self.samples],
        }


def compute_experiment_metrics(record: ExperimentRecord) -> dict[str, float]:
    sample_count = len(record.samples)
    actual_duration_seconds = max(0.0, record.completed_at - record.started_at)

    joint_path_length_rad = 0.0
    ee_path_length_m = 0.0

    for previous, current in zip(record.samples, record.samples[1:]):
        joint_path_length_rad += sum(
            abs(curr - prev)
            for curr, prev in zip(current.joint_positions, previous.joint_positions)
        )
        ee_path_length_m += _distance(previous.ee_position, current.ee_position)

    max_joint_speed_rad_s = 0.0
    mean_joint_speed_rad_s = 0.0
    if sample_count:
        all_joint_speeds = [
            abs(speed)
            for sample in record.samples
            for speed in sample.joint_velocities
        ]
        max_joint_speed_rad_s = max(all_joint_speeds, default=0.0)
        mean_joint_speed_rad_s = sum(all_joint_speeds) / len(all_joint_speeds)

    final_joint_error_rad = 0.0
    if record.samples:
        final_positions = record.samples[-1].joint_positions
        final_joint_error_rad = math.sqrt(
            sum(
                (actual - target) ** 2
                for actual, target in zip(
                    final_positions, record.config.target_joints_rad
                )
            )
        )

    return {
        "sample_count": float(sample_count),
        "actual_duration_seconds": actual_duration_seconds,
        "joint_path_length_rad": joint_path_length_rad,
        "ee_path_length_m": ee_path_length_m,
        "max_joint_speed_rad_s": max_joint_speed_rad_s,
        "mean_joint_speed_rad_s": mean_joint_speed_rad_s,
        "final_joint_error_rad": final_joint_error_rad,
    }


class ExperimentRecorder:
    def __init__(self) -> None:
        self._active_config: ExperimentConfig | None = None
        self._started_at: float | None = None
        self._samples: list[ExperimentSample] = []

    @property
    def is_recording(self) -> bool:
        return self._active_config is not None

    def start(self, config: ExperimentConfig, started_at: float) -> None:
        self._active_config = config
        self._started_at = float(started_at)
        self._samples = []

    def add_sample(
        self,
        *,
        timestamp: float,
        joint_positions: list[float],
        joint_velocities: list[float],
        ee_position: list[float],
    ) -> None:
        if not self.is_recording:
            return
        self._samples.append(
            ExperimentSample(
                timestamp=timestamp,
                joint_positions=joint_positions,
                joint_velocities=joint_velocities,
                ee_position=ee_position,
            )
        )

    def stop(self, completed_at: float) -> ExperimentRecord:
        if self._active_config is None or self._started_at is None:
            raise RuntimeError("No active experiment recording")

        record = ExperimentRecord(
            config=self._active_config,
            started_at=self._started_at,
            completed_at=float(completed_at),
            samples=list(self._samples),
        )
        record.metrics = compute_experiment_metrics(record)

        self._active_config = None
        self._started_at = None
        self._samples = []
        return record


def export_experiment_record(
    record: ExperimentRecord, output_dir: str | Path
) -> dict[str, Path]:
    target_dir = Path(output_dir)
    target_dir.mkdir(parents=True, exist_ok=True)

    slug = _slugify(record.config.name)
    base_name = f"{slug}-{int(record.started_at)}"

    json_path = target_dir / f"{base_name}.json"
    csv_path = target_dir / f"{base_name}.csv"

    json_path.write_text(json.dumps(record.to_dict(), indent=2))

    with csv_path.open("w", newline="") as handle:
        fieldnames = [
            "elapsed_seconds",
            *[f"joint_{index}_position_rad" for index in range(1, JOINT_COUNT + 1)],
            *[f"joint_{index}_velocity_rad_s" for index in range(1, JOINT_COUNT + 1)],
            "ee_x_m",
            "ee_y_m",
            "ee_z_m",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()

        for sample in record.samples:
            row = {
                "elapsed_seconds": f"{sample.timestamp - record.started_at:.6f}",
                "ee_x_m": f"{sample.ee_position[0]:.6f}",
                "ee_y_m": f"{sample.ee_position[1]:.6f}",
                "ee_z_m": f"{sample.ee_position[2]:.6f}",
            }
            row.update(
                {
                    f"joint_{index}_position_rad": f"{sample.joint_positions[index - 1]:.6f}"
                    for index in range(1, JOINT_COUNT + 1)
                }
            )
            row.update(
                {
                    f"joint_{index}_velocity_rad_s": f"{sample.joint_velocities[index - 1]:.6f}"
                    for index in range(1, JOINT_COUNT + 1)
                }
            )
            writer.writerow(row)

    return {"json": json_path, "csv": csv_path}


def format_metrics_summary(metrics: dict[str, float]) -> str:
    return "\n".join(
        [
            "Latest experiment",
            f"Samples: {int(metrics.get('sample_count', 0.0))}",
            f"Duration: {metrics.get('actual_duration_seconds', 0.0):.2f} s",
            f"Joint path: {metrics.get('joint_path_length_rad', 0.0):.3f} rad",
            f"EE path: {metrics.get('ee_path_length_m', 0.0):.3f} m",
            f"Max speed: {metrics.get('max_joint_speed_rad_s', 0.0):.3f} rad/s",
            f"Mean speed: {metrics.get('mean_joint_speed_rad_s', 0.0):.3f} rad/s",
            f"Final error: {metrics.get('final_joint_error_rad', 0.0):.3f} rad",
        ]
    )
