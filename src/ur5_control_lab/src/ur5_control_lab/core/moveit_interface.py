from __future__ import annotations

from copy import deepcopy
from typing import Any


class MoveItPositionPlanner:
    def __init__(self, move_group: Any | None = None, group_name: str = "manipulator"):
        if move_group is not None:
            self.group = move_group
            return

        import moveit_commander

        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(5)

    def _copy_pose_with_position(self, current_pose: Any, x: float, y: float, z: float) -> Any:
        target_pose = deepcopy(current_pose)
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        return target_pose

    def _copy_pose_with_pose(
        self,
        current_pose: Any,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> Any:
        target_pose = self._copy_pose_with_position(current_pose, x, y, z)
        target_pose.orientation.x = float(qx)
        target_pose.orientation.y = float(qy)
        target_pose.orientation.z = float(qz)
        target_pose.orientation.w = float(qw)
        return target_pose

    def _normalize_plan_result(self, plan_result: Any) -> tuple[bool, Any]:
        if isinstance(plan_result, tuple):
            if len(plan_result) >= 2 and isinstance(plan_result[0], bool):
                return plan_result[0], plan_result[1]
            if len(plan_result) == 2:
                return True, plan_result[1]
        return self._plan_has_points(plan_result), plan_result

    def _plan_has_points(self, plan: Any) -> bool:
        trajectory = getattr(plan, "joint_trajectory", None)
        points = getattr(trajectory, "points", None)
        return bool(points)

    def _extract_joint_positions(self, plan: Any) -> list[float]:
        if not self._plan_has_points(plan):
            return []
        return list(plan.joint_trajectory.points[-1].positions)

    def plan_position_target(self, x: float, y: float, z: float) -> dict[str, Any]:
        current_pose = self.group.get_current_pose().pose
        target_pose = self._copy_pose_with_position(current_pose, x, y, z)
        return self._plan_target_pose(target_pose, "moveit_position")

    def plan_pose_target(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> dict[str, Any]:
        current_pose = self.group.get_current_pose().pose
        target_pose = self._copy_pose_with_pose(current_pose, x, y, z, qx, qy, qz, qw)
        return self._plan_target_pose(target_pose, "moveit_pose")

    def _plan_target_pose(self, target_pose: Any, planner_id: str) -> dict[str, Any]:
        self.group.set_pose_target(target_pose)

        success, plan = self._normalize_plan_result(self.group.plan())
        if not success or not self._plan_has_points(plan):
            self.group.clear_pose_targets()
            return {
                "success": False,
                "planner_id": planner_id,
                "message": "MoveIt planning failed",
                "plan": None,
                "target_pose": target_pose,
                "target_joint_positions": [],
            }

        return {
            "success": True,
            "planner_id": planner_id,
            "message": "MoveIt plan ready",
            "plan": plan,
            "target_pose": target_pose,
            "target_joint_positions": self._extract_joint_positions(plan),
        }

    def execute_plan(self, plan: Any) -> bool:
        executed = bool(self.group.execute(plan, wait=True))
        self.group.stop()
        self.group.clear_pose_targets()
        return executed

    def plan_and_execute_position_target(self, x: float, y: float, z: float) -> dict[str, Any]:
        result = self.plan_position_target(x, y, z)
        if not result["success"]:
            return result

        if not self.execute_plan(result["plan"]):
            result["success"] = False
            result["message"] = "MoveIt execution failed"
            return result

        result["message"] = "MoveIt execution succeeded"
        return result

    def plan_and_execute_pose_target(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> dict[str, Any]:
        result = self.plan_pose_target(x, y, z, qx, qy, qz, qw)
        if not result["success"]:
            return result

        if not self.execute_plan(result["plan"]):
            result["success"] = False
            result["message"] = "MoveIt execution failed"
            return result

        result["message"] = "MoveIt execution succeeded"
        return result
