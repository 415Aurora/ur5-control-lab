#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_SETUP="$ROOT_DIR/devel/setup.bash"
VENV_SETUP="$ROOT_DIR/ur5_env/bin/activate"
APP_ONLY=false
SIM_PID=""

log() {
    echo "[run_project] $*"
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
}

source_ros() {
    local ros_setup=""
    for candidate in \
        /opt/ros/noetic/setup.bash \
        /opt/ros/melodic/setup.bash \
        "$HOME/catkin_ws/devel/setup.bash"
    do
        if [[ -f "$candidate" ]]; then
            ros_setup="$candidate"
            break
        fi
    done

    if [[ -z "$ros_setup" ]]; then
        echo "ROS setup.bash not found. Install ROS Noetic and source it before launching." >&2
        exit 1
    fi

    log "Sourcing ROS from $ros_setup"
    source_with_nounset_guard "$ros_setup"
}

source_local_env() {
    if [[ -f "$VENV_SETUP" ]]; then
        log "Activating virtualenv at $VENV_SETUP"
        # shellcheck disable=SC1090
        source "$VENV_SETUP"
    else
        log "Virtualenv not found. Using current Python environment."
    fi

    if [[ ! -f "$WORKSPACE_SETUP" ]]; then
        echo "Workspace setup missing at $WORKSPACE_SETUP. Run catkin_make first." >&2
        exit 1
    fi

    log "Sourcing workspace from $WORKSPACE_SETUP"
    source_with_nounset_guard "$WORKSPACE_SETUP"
}

ensure_python_module() {
    local module="$1"
    if ! python3 -c "import $module" >/dev/null 2>&1; then
        echo "Missing Python module '$module'. Install dependencies with: python3 -m pip install -r $ROOT_DIR/requirements.txt" >&2
        exit 1
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

launch_simulation() {
    log "Launching Gazebo bringup..."
    roslaunch ur_gazebo ur5_bringup.launch &
    SIM_PID=$!
    wait_for_topic "/joint_states" 60
    wait_for_topic "/eff_joint_traj_controller/follow_joint_trajectory/goal" 60
}

run_gui() {
    log "Starting GUI via rosrun ur5_control_lab main.py"
    rosrun ur5_control_lab main.py
}

for arg in "$@"; do
    case "$arg" in
        --app-only)
            APP_ONLY=true
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            echo "Usage: $0 [--app-only]" >&2
            exit 1
            ;;
    esac
done

trap cleanup EXIT INT TERM

source_ros
source_local_env
ensure_python_module customtkinter

if [[ "$APP_ONLY" == false ]]; then
    launch_simulation
else
    log "App-only mode enabled. Assuming ROS master and Gazebo are already running."
fi

run_gui
