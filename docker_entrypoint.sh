#!/bin/bash
set -euo pipefail

ROOT_DIR="/root/ur5_gazebo"
SIM_PID=""
MOVEIT_PID=""

log() {
    echo "[docker_entrypoint] $*"
}

source_with_nounset_guard() {
    local target="$1"
    local status=0
    local had_nounset=0

    case $- in
        *u*) had_nounset=1 ;;
    esac

    set +u
    # shellcheck disable=SC1090
    source "$target" || status=$?
    if [[ "$had_nounset" -eq 1 ]]; then
        set -u
    fi

    return "$status"
}

cleanup() {
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        log "Stopping Gazebo launch (PID $SIM_PID)..."
        kill "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
    if [[ -n "$MOVEIT_PID" ]] && kill -0 "$MOVEIT_PID" 2>/dev/null; then
        log "Stopping MoveIt launch (PID $MOVEIT_PID)..."
        kill "$MOVEIT_PID" 2>/dev/null || true
        wait "$MOVEIT_PID" 2>/dev/null || true
    fi
}

wait_for_topic() {
    local topic="$1"
    local timeout_seconds="${2:-60}"
    local elapsed=0

    log "Waiting for topic $topic ..."
    while (( elapsed < timeout_seconds )); do
        if rostopic list 2>/dev/null | grep -Fxq "$topic"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "Timed out waiting for ROS topic $topic" >&2
    exit 1
}

wait_for_service() {
    local service="$1"
    local timeout_seconds="${2:-60}"
    local elapsed=0

    log "Waiting for service $service ..."
    while (( elapsed < timeout_seconds )); do
        if rosservice list 2>/dev/null | grep -Fxq "$service"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "Timed out waiting for ROS service $service" >&2
    exit 1
}

trap cleanup EXIT INT TERM

cd "$ROOT_DIR"

log "Sourcing ROS and workspace"
source_with_nounset_guard "/opt/ros/noetic/setup.bash"
catkin_make
source_with_nounset_guard "$ROOT_DIR/devel/setup.bash"

log "Launching Gazebo bringup"
roslaunch ur_gazebo ur5_bringup.launch &
SIM_PID=$!

wait_for_topic "/joint_states" 60
wait_for_topic "/eff_joint_traj_controller/follow_joint_trajectory/goal" 60

log "Launching MoveIt planning execution"
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true &
MOVEIT_PID=$!

wait_for_topic "/move_group/status" 60
wait_for_service "/get_planning_scene" 60

log "Starting GUI"
rosrun ur5_control_lab main.py
