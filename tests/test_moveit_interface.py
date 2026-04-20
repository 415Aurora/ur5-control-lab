from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src/ur5_control_lab/src"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from ur5_control_lab.core.moveit_interface import MoveItPositionPlanner  # noqa: E402


def _make_pose(x: float, y: float, z: float, ox: float, oy: float, oz: float, ow: float):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=ox, y=oy, z=oz, w=ow),
    )


def _make_plan(*joint_positions: float):
    return SimpleNamespace(
        joint_trajectory=SimpleNamespace(
            points=[SimpleNamespace(positions=list(joint_positions))]
        )
    )


class FakeMoveGroup:
    def __init__(self, current_pose, plan_result, execute_result=True):
        self._current_pose = current_pose
        self._plan_result = plan_result
        self._execute_result = execute_result
        self.target_pose = None
        self.stop_called = False
        self.clear_called = False

    def get_current_pose(self):
        return SimpleNamespace(pose=self._current_pose)

    def set_pose_target(self, pose):
        self.target_pose = pose

    def plan(self):
        return self._plan_result

    def execute(self, plan, wait=True):
        return self._execute_result

    def stop(self):
        self.stop_called = True

    def clear_pose_targets(self):
        self.clear_called = True


def test_plan_position_target_preserves_current_orientation() -> None:
    current_pose = _make_pose(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)
    planner = MoveItPositionPlanner(
        move_group=FakeMoveGroup(current_pose, (True, _make_plan(0.1, 0.2, 0.3, 0.4, 0.5, 0.6), 0.0, None))
    )

    result = planner.plan_position_target(0.9, 0.8, 0.7)

    assert result["success"] is True
    assert planner.group.target_pose.position.x == 0.9
    assert planner.group.target_pose.position.y == 0.8
    assert planner.group.target_pose.position.z == 0.7
    assert planner.group.target_pose.orientation.x == 0.4
    assert planner.group.target_pose.orientation.y == 0.5
    assert planner.group.target_pose.orientation.z == 0.6
    assert planner.group.target_pose.orientation.w == 0.7
    assert result["target_joint_positions"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


def test_plan_and_execute_position_target_returns_failure_when_plan_has_no_points() -> None:
    current_pose = _make_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    empty_plan = SimpleNamespace(joint_trajectory=SimpleNamespace(points=[]))
    planner = MoveItPositionPlanner(
        move_group=FakeMoveGroup(current_pose, (False, empty_plan, 0.0, None))
    )

    result = planner.plan_and_execute_position_target(0.1, 0.2, 0.3)

    assert result["success"] is False
    assert result["message"] == "MoveIt planning failed"
    assert planner.group.clear_called is True


def test_execute_plan_stops_and_clears_targets_after_execution() -> None:
    current_pose = _make_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    plan = _make_plan(1.0, 1.1, 1.2, 1.3, 1.4, 1.5)
    planner = MoveItPositionPlanner(
        move_group=FakeMoveGroup(current_pose, (True, plan, 0.0, None))
    )

    result = planner.execute_plan(plan)

    assert result is True
    assert planner.group.stop_called is True
    assert planner.group.clear_called is True
