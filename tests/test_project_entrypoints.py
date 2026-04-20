from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (REPO_ROOT / path).read_text()


def test_legacy_pd_entrypoint_removed_from_repo() -> None:
    assert not (REPO_ROOT / "src/ur5_control_lab/scripts/pd.py").exists()
    assert not (REPO_ROOT / "src/ur5_control_lab/scripts/pd.py.bak").exists()


def test_legacy_script_core_removed_from_repo() -> None:
    assert not (REPO_ROOT / "src/ur5_control_lab/scripts/core").exists()


def test_scripts_and_docs_only_reference_main_entrypoint() -> None:
    files = [
        "README.md",
        "run_docker.sh",
        "run_project.sh",
        "docker_entrypoint.sh",
        "start_docker.sh",
    ]
    for path in files:
        content = read(path)
        assert "pd.py" not in content
        assert "rosrun ur5_control_lab main.py" in content or path == "start_docker.sh"


def test_run_project_supports_default_and_app_only_modes() -> None:
    content = read("run_project.sh")
    assert "--app-only" in content
    assert "roslaunch ur_gazebo ur5_bringup.launch" in content
    assert "rosrun ur5_control_lab main.py" in content
    assert "joint_states" in content


def test_docker_uses_shared_requirements_and_entrypoint() -> None:
    requirements = REPO_ROOT / "requirements.txt"
    assert requirements.exists()
    assert "customtkinter" in requirements.read_text()

    dockerfile = read("Dockerfile")
    assert "pip install -r requirements.txt" in dockerfile
    assert 'ENTRYPOINT ["/root/ur5_gazebo/docker_entrypoint.sh"]' in dockerfile


def test_display_layer_uses_ur5_control_lab_project_name() -> None:
    run_docker = read("run_docker.sh")
    assert 'IMAGE_NAME="ur5-control-lab"' in run_docker

    readme = read("README.md")
    assert readme.startswith("# UR5 Control Lab")
    assert "ur5-control-lab" in readme.lower()
    assert "rosrun ur5_control_lab main.py" in readme


def test_run_docker_persists_exports_to_host_workspace() -> None:
    content = read("run_docker.sh")
    assert 'EXPORTS_DIR="$ROOT_DIR/exports"' in content
    assert 'mkdir -p "$EXPORTS_DIR/experiments"' in content
    assert 'CONTAINER_EXPORTS_DIR="/root/ur5_gazebo/exports"' in content
    assert '-v "$EXPORTS_DIR:$CONTAINER_EXPORTS_DIR"' in content


def test_gui_export_directory_matches_docker_exports_mount() -> None:
    app = read("src/ur5_control_lab/src/ur5_control_lab/gui/app.py")
    assert '/ "exports" / "experiments"' in app


def test_package_manifest_covers_runtime_dependencies() -> None:
    manifest = read("src/ur5_control_lab/package.xml")
    for token in [
        "<depend>actionlib</depend>",
        "<depend>kdl_parser_py</depend>",
        "<exec_depend>python3-pykdl</exec_depend>",
    ]:
        assert token in manifest


def test_docker_entrypoint_uses_guarded_source_for_ros_setup() -> None:
    content = read("docker_entrypoint.sh")
    assert "source_with_nounset_guard()" in content
    assert 'source_with_nounset_guard "/opt/ros/noetic/setup.bash"' in content
    assert 'source "$ROOT_DIR/devel/setup.bash"' not in content
    assert 'source /opt/ros/noetic/setup.bash' not in content


def test_run_project_uses_guarded_source_for_ros_and_workspace_setup() -> None:
    content = read("run_project.sh")
    assert "source_with_nounset_guard()" in content
    assert 'source_with_nounset_guard "$ros_setup"' in content
    assert 'source_with_nounset_guard "$WORKSPACE_SETUP"' in content
    assert 'source "$ros_setup"' not in content
    assert 'source "$WORKSPACE_SETUP"' not in content


def test_default_gazebo_world_focuses_camera_on_robot() -> None:
    launch = read("src/universal_robot/ur_gazebo/launch/inc/ur_control.launch.xml")
    assert 'default="$(find ur_gazebo)/worlds/ur5_visible.world"' in launch

    world_path = REPO_ROOT / "src/universal_robot/ur_gazebo/worlds/ur5_visible.world"
    assert world_path.exists()

    world = world_path.read_text()
    assert "<uri>model://sun</uri>" in world
    assert "<uri>model://ground_plane</uri>" in world
    assert "<gui" in world
    assert "<camera" in world
    assert "<pose>2.2 -2.2 1.6 0 0.35 2.35</pose>" in world


def test_readme_mentions_default_gazebo_camera_focus() -> None:
    content = read("README.md")
    assert "Gazebo" in content
    assert "default" in content.lower()
    assert "UR5" in content
    assert "camera" in content.lower()


def test_readme_mentions_docker_export_persistence() -> None:
    content = read("README.md")
    assert "./run_docker.sh" in content
    assert "exports/experiments" in content
    assert "host" in content.lower() or "宿主" in content
