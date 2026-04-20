import customtkinter as ctk
import rospy
import actionlib
import threading
import numpy as np
import time
import sys
import math
from pathlib import Path

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib import style

# Import Core
from ur5_control_lab.core.kinematics import KinematicsModule
from ur5_control_lab.core.dynamics import DynamicsModule
from ur5_control_lab.core.planning import PlanningModule
from ur5_control_lab.core.experiments import (
    ExperimentConfig,
    ExperimentRecorder,
    export_experiment_record,
    format_metrics_summary,
)
from ur5_control_lab.core.moveit_interface import MoveItPositionPlanner

# Import GUI
from ur5_control_lab.gui.widgets import Matrix4x4View, DHTableView, JointInput

# Config
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("dark-blue")
style.use('dark_background')
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class UR5LabApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        # ROS Init
        rospy.init_node("ur5_lab_main", anonymous=True)
        
        # Modules
        self.kin_mod = KinematicsModule()
        self.dyn_mod = DynamicsModule()
        self.plan_mod = PlanningModule(self.kin_mod)
        
        # ROS Interface
        self.connect_ros()
        
        # State
        self.current_joints = [0.0]*6
        self.current_vels = [0.0]*6
        self.is_recording = False
        self.recorded_traj = []
        self.experiment_recorder = ExperimentRecorder()
        self.latest_experiment = None
        self.last_export_paths = None
        self.experiment_output_dir = self._resolve_project_root() / "exports" / "experiments"
        self.moveit_planner = None
        self.moveit_error = None
        self.running = True

        self._initialize_moveit()
        
        # UI
        self.setup_ui()
        
        # Workers
        threading.Thread(target=self.ros_loop, daemon=True).start()
        threading.Thread(target=self.update_analysis_ui, daemon=True).start()

    def connect_ros(self):
        self.client = actionlib.SimpleActionClient(
            '/eff_joint_traj_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        self.effort_pub = rospy.Publisher(
            '/joint_group_eff_controller/command', 
            Float64MultiArray, 
            queue_size=10
        )

    def _resolve_project_root(self):
        return Path(__file__).resolve().parents[5]

    def _initialize_moveit(self):
        try:
            self.moveit_planner = MoveItPositionPlanner()
        except Exception as exc:
            self.moveit_error = str(exc)

    def _quaternion_from_euler_degrees(self, roll_deg, pitch_deg, yaw_deg):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def setup_ui(self):
        self.title("UR5 Control Lab - Professional Edition")
        self.geometry("1600x900")
        
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        
        # Sidebar
        self.sidebar = ctk.CTkFrame(self, width=250, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar_content()
        
        # Tabs
        self.tabs = ctk.CTkTabview(self, anchor="nw")
        self.tabs.grid(row=0, column=1, sticky="nsew", padx=20, pady=20)
        
        self.tab_dash = self.tabs.add("Dashboard")
        self.tab_kin = self.tabs.add("Kinematics & Dynamics")
        self.tab_plan = self.tabs.add("Path Planning")
        self.tab_rec = self.tabs.add("Recorder")
        
        self.setup_dashboard(self.tab_dash)
        self.setup_kinematics(self.tab_kin)
        self.setup_planning(self.tab_plan)
        self.setup_recorder(self.tab_rec)

    def sidebar_content(self):
        lbl = ctk.CTkLabel(self.sidebar, text="UR5 Control Lab", font=ctk.CTkFont(size=24, weight="bold"))
        lbl.grid(row=0, column=0, padx=20, pady=20)
        
        self.lbl_status = ctk.CTkLabel(self.sidebar, text="System: Initializing...", text_color="yellow", font=ctk.CTkFont(size=14))
        self.lbl_status.grid(row=1, column=0, padx=20, pady=10)
        
        frm = ctk.CTkFrame(self.sidebar)
        frm.grid(row=2, column=0, padx=20, pady=20, sticky="ew")
        ctk.CTkLabel(frm, text="Quick Poses", font=ctk.CTkFont(weight="bold")).pack(pady=10)
        ctk.CTkButton(frm, text="Home", command=lambda: self.move_to_preset("home")).pack(pady=5, padx=10, fill="x")
        ctk.CTkButton(frm, text="Up (Zero)", command=lambda: self.move_to_preset("up")).pack(pady=5, padx=10, fill="x")
        ctk.CTkButton(frm, text="Ready", command=lambda: self.move_to_preset("ready")).pack(pady=5, padx=10, fill="x")
        
        ctk.CTkButton(self.sidebar, text="EXIT APP", fg_color="gray", hover_color="darkgray", command=self.close_app).grid(row=8, column=0, padx=20, pady=10, sticky="ew")
        ctk.CTkButton(self.sidebar, text="EMERGENCY STOP", fg_color="red", hover_color="darkred", command=self.emergency_stop).grid(row=9, column=0, padx=20, pady=20, sticky="ew")

    def setup_dashboard(self, parent):
        info = ctk.CTkFrame(parent, height=100)
        info.pack(fill="x", pady=10, padx=10)
        
        self.lbl_ee = ctk.CTkLabel(info, text="EE Pos: --", font=ctk.CTkFont(family="Monospace", size=16))
        self.lbl_ee.pack(side="left", padx=40, pady=20)
        
        self.lbl_j = ctk.CTkLabel(info, text="Joints: --", font=ctk.CTkFont(family="Monospace", size=16))
        self.lbl_j.pack(side="right", padx=40, pady=20)
        
        plot_frame = ctk.CTkFrame(parent)
        plot_frame.pack(fill="both", expand=True, pady=10, padx=10)
        
        self.fig = Figure(figsize=(5,4), dpi=100)
        self.fig.patch.set_facecolor('#2b2b2b')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#2b2b2b')
        self.ax.tick_params(colors='white')
        self.ax.grid(color='#444444')
        self.ax.set_title("Joint Angles", color='white')
        
        self.lines = {}
        colors = ['#FF5555', '#50FA7B', '#8BE9FD', '#BD93F9', '#FFB86C', '#F1FA8C']
        for i, name in enumerate(JOINT_NAMES):
            self.lines[name], = self.ax.plot([], [], label=f"J{i+1}", color=colors[i])
        self.ax.legend(facecolor='#2b2b2b', edgecolor='white', loc='upper right')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def setup_kinematics(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=1)
        parent.grid_rowconfigure(0, weight=1)
        
        # Left: DH Table
        f1 = ctk.CTkFrame(parent)
        f1.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        ctk.CTkLabel(f1, text="Standard D-H Parameters", font=ctk.CTkFont(size=18, weight="bold")).pack(pady=15)
        
        self.dh_table = DHTableView(f1)
        self.dh_table.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Right: Matrices
        f2 = ctk.CTkFrame(parent)
        f2.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        
        # Transformation Matrix
        self.mat_view = Matrix4x4View(f2, title="Base -> EE Transform")
        self.mat_view.pack(fill="x", padx=10, pady=10)
        
        # Dynamics Section
        ctk.CTkLabel(f2, text="Dynamics", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(20, 5))
        
        dyn_grid = ctk.CTkFrame(f2, fg_color="transparent")
        dyn_grid.pack(fill="x", padx=10)
        
        # Gravity
        g_frame = ctk.CTkFrame(dyn_grid)
        g_frame.pack(side="left", fill="both", expand=True, padx=5)
        ctk.CTkLabel(g_frame, text="Gravity (G)", font=("Arial", 12, "bold")).pack(pady=5)
        self.lbl_grav = ctk.CTkLabel(g_frame, text="[0, 0, 0...]", font=("Monospace", 12), justify="left")
        self.lbl_grav.pack(pady=10)

        # Coriolis
        c_frame = ctk.CTkFrame(dyn_grid)
        c_frame.pack(side="left", fill="both", expand=True, padx=5)
        ctk.CTkLabel(c_frame, text="Coriolis (C)", font=("Arial", 12, "bold")).pack(pady=5)
        self.lbl_cor = ctk.CTkLabel(c_frame, text="[0, 0, 0...]", font=("Monospace", 12), justify="left")
        self.lbl_cor.pack(pady=10)

    def setup_planning(self, parent):
        ctrl = ctk.CTkFrame(parent)
        ctrl.pack(fill="x", pady=20, padx=20)

        settings = ctk.CTkFrame(ctrl)
        settings.pack(fill="x", padx=10, pady=(10, 0))
        for column in range(3):
            settings.grid_columnconfigure(column, weight=1)

        ctk.CTkLabel(settings, text="Experiment Name").grid(row=0, column=0, padx=10, pady=(10, 5), sticky="w")
        self.entry_experiment_name = ctk.CTkEntry(settings)
        self.entry_experiment_name.grid(row=1, column=0, padx=10, pady=(0, 10), sticky="ew")
        self.entry_experiment_name.insert(0, self._default_experiment_name())

        ctk.CTkLabel(settings, text="Duration (s)").grid(row=0, column=1, padx=10, pady=(10, 5), sticky="w")
        self.entry_duration = ctk.CTkEntry(settings)
        self.entry_duration.grid(row=1, column=1, padx=10, pady=(0, 10), sticky="ew")
        self.entry_duration.insert(0, "4.0")

        ctk.CTkLabel(settings, text="Planner").grid(row=0, column=2, padx=10, pady=(10, 5), sticky="w")
        self.planner_menu = ctk.CTkOptionMenu(settings, values=["joint_space_quintic"])
        self.planner_menu.grid(row=1, column=2, padx=10, pady=(0, 10), sticky="ew")

        ctk.CTkLabel(ctrl, text="Target Joints (Degrees)", font=ctk.CTkFont(size=18, weight="bold")).pack(pady=10)
        
        # Vertical container for the horizontal widgets
        inputs_frame = ctk.CTkScrollableFrame(ctrl, height=350, fg_color="transparent") # Scrollable in case height is an issue
        inputs_frame.pack(pady=10, fill="x", expand=True)
        
        self.joint_inputs = []
        for i, name in enumerate(JOINT_NAMES):
            ji = JointInput(inputs_frame, joint_idx=i, joint_name=name, initial_val=0, command=None)
            ji.pack(fill="x", pady=5, padx=10)
            self.joint_inputs.append(ji)

        ctk.CTkButton(ctrl, text="Plan & Execute", command=self.plan_joint, height=40, font=ctk.CTkFont(size=14, weight="bold")).pack(pady=20)

        cartesian_frame = ctk.CTkFrame(parent)
        cartesian_frame.pack(fill="x", pady=(0, 20), padx=20)
        ctk.CTkLabel(
            cartesian_frame,
            text="Cartesian Pose Target (X/Y/Z in meters, Roll/Pitch/Yaw in degrees)",
            font=ctk.CTkFont(size=16, weight="bold"),
        ).pack(pady=(15, 10))

        cartesian_inputs = ctk.CTkFrame(cartesian_frame, fg_color="transparent")
        cartesian_inputs.pack(fill="x", padx=10, pady=(0, 10))
        for column in range(6):
            cartesian_inputs.grid_columnconfigure(column, weight=1)

        self.entry_target_x = ctk.CTkEntry(cartesian_inputs)
        self.entry_target_y = ctk.CTkEntry(cartesian_inputs)
        self.entry_target_z = ctk.CTkEntry(cartesian_inputs)
        self.entry_target_roll = ctk.CTkEntry(cartesian_inputs)
        self.entry_target_pitch = ctk.CTkEntry(cartesian_inputs)
        self.entry_target_yaw = ctk.CTkEntry(cartesian_inputs)
        for idx, (label, entry) in enumerate(
            [
                ("X", self.entry_target_x),
                ("Y", self.entry_target_y),
                ("Z", self.entry_target_z),
                ("Roll", self.entry_target_roll),
                ("Pitch", self.entry_target_pitch),
                ("Yaw", self.entry_target_yaw),
            ]
        ):
            ctk.CTkLabel(cartesian_inputs, text=label).grid(row=0, column=idx, padx=10, pady=(0, 5), sticky="w")
            entry.grid(row=1, column=idx, padx=10, pady=(0, 10), sticky="ew")

        self.btn_cartesian_plan = ctk.CTkButton(
            cartesian_frame,
            text="Plan & Execute Cartesian Pose",
            command=self.plan_cartesian_pose,
            height=40,
            font=ctk.CTkFont(size=14, weight="bold"),
        )
        self.btn_cartesian_plan.pack(pady=(0, 10))
        if self.moveit_error is not None:
            self.btn_cartesian_plan.configure(state="disabled")

        default_cartesian_status = "Cartesian planner ready" if self.moveit_error is None else f"Cartesian planner unavailable: {self.moveit_error}"
        default_cartesian_color = "silver" if self.moveit_error is None else "orange"
        self.cartesian_status = ctk.CTkLabel(parent, text=default_cartesian_status, text_color=default_cartesian_color, font=ctk.CTkFont(size=15))
        self.cartesian_status.pack(pady=(0, 10))
        
        self.plan_status = ctk.CTkLabel(parent, text="Status: Ready", font=ctk.CTkFont(size=16))
        self.plan_status.pack(pady=20)

    def setup_recorder(self, parent):
        manual_frame = ctk.CTkFrame(parent)
        manual_frame.pack(fill="x", padx=20, pady=20)

        ctk.CTkButton(manual_frame, text="Start Recording", command=self.toggle_rec, fg_color="orange", height=50).pack(pady=20)
        self.btn_replay = ctk.CTkButton(manual_frame, text="Replay Recording", command=self.replay_rec, state="disabled", height=50)
        self.btn_replay.pack(pady=10)
        self.lbl_rec_count = ctk.CTkLabel(manual_frame, text="0 Points Saved", font=ctk.CTkFont(size=16))
        self.lbl_rec_count.pack(pady=(0, 10))

        experiment_frame = ctk.CTkFrame(parent)
        experiment_frame.pack(fill="both", expand=True, padx=20, pady=(0, 20))

        ctk.CTkLabel(experiment_frame, text="Experiment Tracking", font=ctk.CTkFont(size=18, weight="bold")).pack(pady=(20, 10))
        self.lbl_last_experiment = ctk.CTkLabel(
            experiment_frame,
            text="No completed experiment yet",
            font=ctk.CTkFont(size=15),
        )
        self.lbl_last_experiment.pack(pady=5)

        self.lbl_metric_summary = ctk.CTkLabel(
            experiment_frame,
            text="Latest experiment\nNo completed run",
            justify="left",
            font=ctk.CTkFont(family="Monospace", size=13),
        )
        self.lbl_metric_summary.pack(padx=20, pady=10, anchor="w")

        self.btn_export_experiment = ctk.CTkButton(
            experiment_frame,
            text="Export Latest Experiment",
            command=self.export_latest_experiment,
            state="disabled",
            height=42,
        )
        self.btn_export_experiment.pack(pady=10)

        self.lbl_last_export = ctk.CTkLabel(
            experiment_frame,
            text="Exports will be written to exports/experiments",
            font=ctk.CTkFont(size=13),
        )
        self.lbl_last_export.pack(pady=(0, 20))

    # --- Logic ---

    def close_app(self):
        self.running = False
        self.quit()
        self.destroy()
        sys.exit(0)

    def adj_joint(self, idx, val):
        try:
            current = float(self.joint_entries[idx].get())
            self.joint_entries[idx].delete(0, "end")
            self.joint_entries[idx].insert(0, str(int(current + val)))
        except: pass

    def ros_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.running:
            try:
                msg = rospy.wait_for_message("joint_states", JointState, timeout=1.0)
                pos, vel = [], []
                for name in JOINT_NAMES:
                    if name in msg.name:
                        idx = msg.name.index(name)
                        pos.append(msg.position[idx])
                        vel.append(msg.velocity[idx])
                
                if len(pos)==6:
                    self.current_joints = pos
                    self.current_vels = vel
                    self.lbl_status.configure(text="System: Connected", text_color="#50FA7B")
                    if self.experiment_recorder.is_recording:
                        ee_position = self.kin_mod.compute_fk(pos)[:3, 3].tolist()
                        self.experiment_recorder.add_sample(
                            timestamp=time.time(),
                            joint_positions=pos,
                            joint_velocities=vel,
                            ee_position=ee_position,
                        )
                
                self.after(0, self.update_dashboard_data)
                
                if self.is_recording: self.recorded_traj.append(pos)
                rate.sleep()
            except Exception as e:
                self.lbl_status.configure(text="System: Disconnected", text_color="#FF5555")

    def update_dashboard_data(self):
        T = self.kin_mod.compute_fk(self.current_joints)
        pos = T[:3, 3]
        self.lbl_ee.configure(text=f"EE (m): X {pos[0]:.2f} Y {pos[1]:.2f} Z {pos[2]:.2f}")
        j_str = " ".join([f"{np.degrees(x):.0f}" for x in self.current_joints])
        self.lbl_j.configure(text=f"Joints (deg): {j_str}")

        if hasattr(self, 'lines') and np.random.rand() < 0.2:
            for i, name in enumerate(JOINT_NAMES):
                xdata = list(self.lines[name].get_xdata()) + [time.time()]
                ydata = list(self.lines[name].get_ydata()) + [np.degrees(self.current_joints[i])]
                if len(xdata)>50: xdata, ydata = xdata[-50:], ydata[-50:]
                self.lines[name].set_data(xdata, ydata)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()

    def update_analysis_ui(self):
        while self.running:
            time.sleep(0.5)
            # DH Table
            dh = self.kin_mod.get_dh_table(self.current_joints)
            
            # Matrices
            T = self.kin_mod.compute_fk(self.current_joints)
            
            # Dynamics
            dyn = self.dyn_mod.compute_dynamics(self.current_joints, self.current_vels)
            
            self.after(0, lambda: self._update_widgets(dh, T, dyn))

    def _update_widgets(self, dh, T, dyn):
        self.dh_table.update_table(dh)
        self.mat_view.update_matrix(T)
        if dyn:
            g_str = "\n".join([f"{x:.2f}" for x in dyn['G']])
            c_str = "\n".join([f"{x:.2f}" for x in dyn['C']])
            self.lbl_grav.configure(text=g_str)
            self.lbl_cor.configure(text=c_str)

    def _default_experiment_name(self):
        return time.strftime("run-%Y%m%d-%H%M%S")

    def _build_experiment_config(self, target_joints_deg):
        raw_name = self.entry_experiment_name.get().strip()
        raw_duration = self.entry_duration.get().strip()

        name = raw_name or self._default_experiment_name()
        duration = float(raw_duration or "4.0")
        planner_id = self.planner_menu.get()

        return ExperimentConfig(
            name=name,
            target_joints_deg=target_joints_deg,
            duration_seconds=duration,
            planner_id=planner_id,
            sample_rate_hz=10.0,
        )

    def _start_experiment_recording(self, config):
        self.experiment_recorder.start(config, started_at=time.time())
        self.after(0, lambda: self.lbl_last_experiment.configure(
            text=f"Recording: {config.name} ({config.planner_id})",
            text_color="orange",
        ))
        self.after(0, lambda: self.lbl_last_export.configure(
            text="Recording in progress...",
            text_color="orange",
        ))

    def _finish_experiment_recording(self, warning_message=None):
        if not self.experiment_recorder.is_recording:
            return

        record = self.experiment_recorder.stop(completed_at=time.time())
        self.latest_experiment = record
        self.last_export_paths = None

        summary = format_metrics_summary(record.metrics)
        self.after(0, lambda: self.lbl_last_experiment.configure(
            text=f"Completed: {record.config.name}",
            text_color="#50FA7B",
        ))
        self.after(0, lambda: self.lbl_metric_summary.configure(text=summary))
        self.after(0, lambda: self.btn_export_experiment.configure(state="normal"))

        export_message = "Ready to export latest experiment"
        export_color = "silver"
        if warning_message:
            export_message = warning_message
            export_color = "orange"
        self.after(0, lambda: self.lbl_last_export.configure(
            text=export_message,
            text_color=export_color,
        ))
        self.after(0, self._refresh_experiment_name)

    def _refresh_experiment_name(self):
        self.entry_experiment_name.delete(0, "end")
        self.entry_experiment_name.insert(0, self._default_experiment_name())

    def _build_moveit_experiment_config(self, target_joint_positions, planner_id):
        target_joint_deg = [float(np.degrees(value)) for value in target_joint_positions]
        return ExperimentConfig(
            name=self.entry_experiment_name.get().strip() or self._default_experiment_name(),
            target_joints_deg=target_joint_deg,
            duration_seconds=float(self.entry_duration.get().strip() or "4.0"),
            planner_id=planner_id,
            sample_rate_hz=10.0,
            notes=(
                f"Cartesian target x={self.entry_target_x.get().strip()} "
                f"y={self.entry_target_y.get().strip()} z={self.entry_target_z.get().strip()} "
                f"roll={self.entry_target_roll.get().strip()} pitch={self.entry_target_pitch.get().strip()} "
                f"yaw={self.entry_target_yaw.get().strip()}"
            ),
        )

    def move_to_preset(self, name):
        targets = {
            "home": [0, -1.57, 0, -1.57, 0, 0],
            "up": [0, -1.57, 0, 0, 0, 0], 
            "ready": [0, -1.57, 1.57, -1.57, -1.57, 0]
        }
        target_deg = [float(np.degrees(value)) for value in targets[name]]
        config = ExperimentConfig(
            name=f"preset-{name}-{time.strftime('%H%M%S')}",
            target_joints_deg=target_deg,
            duration_seconds=3.0,
            planner_id="preset_pose",
            sample_rate_hz=10.0,
        )
        threading.Thread(
            target=self.execute_trajectory,
            args=([targets[name]], 3.0, config),
            daemon=True,
        ).start()

    def plan_joint(self):
        try:
            vals = [ji.get_value() for ji in self.joint_inputs]
            target_rad = [np.radians(v) for v in vals]
            config = self._build_experiment_config(vals)

            self.plan_status.configure(text="Status: Planning...", text_color="yellow")
            traj = self.plan_mod.plan_joint_space(
                self.current_joints,
                target_rad,
                config.duration_seconds,
            )

            threading.Thread(
                target=self.execute_trajectory_points,
                args=(traj, config.duration_seconds, config),
                daemon=True,
            ).start()
            self.plan_status.configure(text="Status: Executing...", text_color="green")
            self.after(
                int((config.duration_seconds + 1.0) * 1000),
                lambda: self.plan_status.configure(text="Status: Ready", text_color="silver"),
            )
        except Exception as e:
            self.plan_status.configure(text=f"Error: {e}", text_color="red")

    def plan_cartesian_pose(self):
        if self.moveit_planner is None:
            self.cartesian_status.configure(
                text=f"Cartesian planner unavailable: {self.moveit_error}",
                text_color="orange",
            )
            return

        try:
            x = float(self.entry_target_x.get())
            y = float(self.entry_target_y.get())
            z = float(self.entry_target_z.get())
            roll = float(self.entry_target_roll.get() or "0.0")
            pitch = float(self.entry_target_pitch.get() or "0.0")
            yaw = float(self.entry_target_yaw.get() or "0.0")
        except ValueError:
            self.cartesian_status.configure(
                text="Cartesian target must be valid X/Y/Z/Roll/Pitch/Yaw numbers",
                text_color="red",
            )
            return

        self.cartesian_status.configure(text="Cartesian pose status: Planning...", text_color="yellow")
        threading.Thread(
            target=self._run_cartesian_pose_plan,
            args=(x, y, z, roll, pitch, yaw),
            daemon=True,
        ).start()

    def _run_cartesian_pose_plan(self, x, y, z, roll, pitch, yaw):
        qx, qy, qz, qw = self._quaternion_from_euler_degrees(roll, pitch, yaw)
        result = self.moveit_planner.plan_pose_target(x, y, z, qx, qy, qz, qw)
        if not result["success"]:
            self.after(0, lambda: self.cartesian_status.configure(
                text=f"Cartesian pose status: {result['message']}",
                text_color="red",
            ))
            return

        target_joint_positions = result["target_joint_positions"] or list(self.current_joints)
        config = self._build_moveit_experiment_config(target_joint_positions, result["planner_id"])
        self._start_experiment_recording(config)
        self.after(0, lambda: self.cartesian_status.configure(
            text="Cartesian pose status: Executing...",
            text_color="green",
        ))

        warning_message = None
        try:
            if not self.moveit_planner.execute_plan(result["plan"]):
                warning_message = "MoveIt execution failed"
        except Exception as exc:
            warning_message = f"MoveIt execution failed: {exc}"
        finally:
            self._finish_experiment_recording(warning_message=warning_message)

        final_status = "Cartesian pose status: Succeeded" if warning_message is None else f"Cartesian pose status: {warning_message}"
        final_color = "#50FA7B" if warning_message is None else "orange"
        self.after(0, lambda: self.cartesian_status.configure(
            text=final_status,
            text_color=final_color,
        ))

    def execute_trajectory(self, waypoints, duration, experiment_config=None):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        g.trajectory.points = [
            JointTrajectoryPoint(positions=waypoints[0], time_from_start=rospy.Duration(duration))
        ]
        self._execute_goal(g, duration, experiment_config)

    def execute_trajectory_points(self, points, duration=4.0, experiment_config=None):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        dt = duration / max(len(points), 1)
        for i, p in enumerate(points):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=p, time_from_start=rospy.Duration((i+1)*dt))
            )
        self._execute_goal(g, duration, experiment_config)

    def _execute_goal(self, goal, duration, experiment_config=None):
        warning_message = None
        try:
            if experiment_config is not None:
                self._start_experiment_recording(experiment_config)

            self.client.send_goal(goal)
            finished = self.client.wait_for_result(rospy.Duration(duration + 5.0))
            if not finished:
                warning_message = "Execution timed out before result acknowledgement"
        except Exception as exc:
            warning_message = f"Experiment finished with warning: {exc}"
        finally:
            if experiment_config is not None:
                self._finish_experiment_recording(warning_message=warning_message)

    def toggle_rec(self):
        self.is_recording = not self.is_recording
        if self.is_recording:
            self.recorded_traj = []
        self.lbl_rec_count.configure(text="Recording..." if self.is_recording else f"{len(self.recorded_traj)} Points Saved", text_color="orange" if self.is_recording else "silver")
        if not self.is_recording: self.btn_replay.configure(state="normal")

    def replay_rec(self):
        if not self.recorded_traj:
            self.lbl_last_export.configure(
                text="No manual recording available to replay",
                text_color="orange",
            )
            return
        threading.Thread(
            target=self.execute_trajectory_points,
            args=(self.recorded_traj,),
            daemon=True,
        ).start()

    def export_latest_experiment(self):
        if self.latest_experiment is None:
            self.lbl_last_export.configure(
                text="No completed experiment available for export",
                text_color="orange",
            )
            return

        try:
            self.last_export_paths = export_experiment_record(
                self.latest_experiment,
                self.experiment_output_dir,
            )
            self.lbl_last_export.configure(
                text=(
                    f"Exported JSON: {self.last_export_paths['json'].name} | "
                    f"CSV: {self.last_export_paths['csv'].name}"
                ),
                text_color="#50FA7B",
            )
        except Exception as exc:
            self.lbl_last_export.configure(
                text=f"Export failed: {exc}",
                text_color="red",
            )

    def emergency_stop(self):
        self.client.cancel_all_goals()
        self.lbl_status.configure(text="Status: EMERGENCY STOP", text_color="red")

if __name__ == "__main__":
    app = UR5LabApp()
    app.mainloop()
