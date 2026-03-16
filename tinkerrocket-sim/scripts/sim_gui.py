#!/usr/bin/env python3
"""TinkerRocket Simulation GUI.

Provides a graphical interface to configure, run, and visualize
closed-loop rocket flight simulations with roll control.

Usage:
    python3 scripts/sim_gui.py
"""
import sys
import os
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog
import numpy as np

# Ensure project root is on the Python path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(PROJECT_DIR, "src"))

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from tinkerrocket_sim.rocket.definition import RocketDefinition, MotorConfig, from_ork
from tinkerrocket_sim.simulation.closed_loop_sim import run_closed_loop, SimConfig


# ============================================================================
# Plot generation functions (reusable from both GUI and CLI)
# ============================================================================

def plot_flight_performance(df, fig, title=""):
    """Page 1: altitude, speed, AoA."""
    fig.clear()
    axes = fig.subplots(3, 1, sharex=True)
    fig.suptitle(f"{title} — Flight Performance" if title else "Flight Performance")

    axes[0].plot(df['time'], df['altitude'], 'b-')
    axes[0].set_ylabel("Altitude (m)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(df['time'], df['speed'], 'r-')
    axes[1].set_ylabel("Speed (m/s)")
    axes[1].grid(True, alpha=0.3)

    if 'alpha_deg' in df.columns:
        axes[2].plot(df['time'], df['alpha_deg'], 'purple', alpha=0.7)
        axes[2].set_ylabel("AoA (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()


def plot_attitude_control(df, fig, title=""):
    """Page 2: roll tracking, pitch/yaw rates, roll rate, fin tab."""
    fig.clear()
    has_angle_profile = 'roll_target_deg' in df.columns
    n_panels = 4 if has_angle_profile else 3
    axes = fig.subplots(n_panels, 1, sharex=True)
    fig.suptitle(f"{title} — Attitude & Control" if title else "Attitude & Control")

    ax_idx = 0

    if has_angle_profile:
        target = df['roll_target_deg'].values.copy()
        speed = df['speed'].values
        active_mask = (~np.isnan(target)) & (speed > 5.0)
        first_active = np.argmax(active_mask) if active_mask.any() else 0

        if 'ekf_roll_deg' in df.columns:
            ekf_raw = df['ekf_roll_deg'].values.copy()
            ekf_unwrap = ekf_raw.copy()
            if first_active < len(ekf_raw) - 1:
                active_section = np.degrees(np.unwrap(
                    np.radians(ekf_raw[first_active:])))
                ekf_unwrap[first_active:] = active_section
            ekf_unwrap[:first_active] = np.nan
            axes[ax_idx].plot(df['time'], ekf_unwrap, 'b-',
                              label='EKF Estimate', alpha=0.7)

        true_raw = df['roll_deg'].values.copy()
        true_unwrap = true_raw.copy()
        if first_active < len(true_raw) - 1:
            active_section = np.degrees(np.unwrap(
                np.radians(true_raw[first_active:])))
            true_unwrap[first_active:] = active_section
        true_unwrap[:first_active] = np.nan

        axes[ax_idx].plot(df['time'], target, 'r--',
                          label='Target', linewidth=2, alpha=0.8)
        axes[ax_idx].plot(df['time'], true_unwrap, 'g-',
                          label='True', alpha=0.5, linewidth=0.8)
        axes[ax_idx].set_ylabel("Roll Angle (deg)")
        axes[ax_idx].legend()
        axes[ax_idx].grid(True, alpha=0.3)
        ax_idx += 1

    if 'pitch_rate_dps' in df.columns:
        axes[ax_idx].plot(df['time'], df['pitch_rate_dps'], 'b-', label='Pitch', alpha=0.7)
        axes[ax_idx].plot(df['time'], df['yaw_rate_dps'], 'r-', label='Yaw', alpha=0.7)
    axes[ax_idx].set_ylabel("Pitch/Yaw Rate (°/s)")
    axes[ax_idx].legend()
    axes[ax_idx].grid(True, alpha=0.3)
    ax_idx += 1

    axes[ax_idx].plot(df['time'], df['roll_rate_dps'], 'b-', label='True', alpha=0.7)
    axes[ax_idx].set_ylabel("Roll Rate (°/s)")
    axes[ax_idx].legend()
    axes[ax_idx].grid(True, alpha=0.3)
    ax_idx += 1

    axes[ax_idx].plot(df['time'], df['fin_tab_cmd'], 'g-', label='Command')
    axes[ax_idx].plot(df['time'], df['fin_tab_actual'], 'r--', label='Actual', alpha=0.7)
    axes[ax_idx].set_ylabel("Fin Tab (deg)")
    axes[ax_idx].set_xlabel("Time (s)")
    axes[ax_idx].legend()
    axes[ax_idx].grid(True, alpha=0.3)

    fig.tight_layout()


def plot_ekf_performance(df, fig, title=""):
    """Page 3: EKF orientation/velocity/position errors."""
    fig.clear()
    has_ekf = all(c in df.columns for c in ['ekf_roll_deg', 'ekf_vn', 'ekf_pn'])
    if not has_ekf:
        ax = fig.add_subplot(111)
        ax.text(0.5, 0.5, "No EKF data available", ha='center', va='center',
                fontsize=14, transform=ax.transAxes)
        return

    axes = fig.subplots(3, 1, sharex=True)
    fig.suptitle(f"{title} — EKF Estimation Error" if title else "EKF Estimation Error")

    roll_err = (df['ekf_roll_deg'] - df['roll_deg'] + 180.0) % 360.0 - 180.0
    pitch_err = (df['ekf_pitch_deg'] - df['pitch_deg'] + 180.0) % 360.0 - 180.0
    yaw_err = (df['ekf_yaw_deg'] - df['yaw_deg'] + 180.0) % 360.0 - 180.0

    axes[0].plot(df['time'], roll_err, 'r-', label='Roll', alpha=0.8)
    axes[0].plot(df['time'], pitch_err, 'g-', label='Pitch', alpha=0.8)
    axes[0].plot(df['time'], yaw_err, 'b-', label='Yaw', alpha=0.8)
    axes[0].set_ylabel("Orientation Error (deg)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(0, color='k', alpha=0.2)
    axes[0].axvline(0, color='k', linestyle='--', alpha=0.3)

    axes[1].plot(df['time'], df['ekf_vn'] - df['true_vn'], 'r-', label='North', alpha=0.8)
    axes[1].plot(df['time'], df['ekf_ve'] - df['true_ve'], 'g-', label='East', alpha=0.8)
    axes[1].plot(df['time'], df['ekf_vd'] - df['true_vd'], 'b-', label='Down', alpha=0.8)
    axes[1].set_ylabel("Velocity Error (m/s)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(0, color='k', alpha=0.2)
    axes[1].axvline(0, color='k', linestyle='--', alpha=0.3)

    axes[2].plot(df['time'], df['ekf_pn'] - df['true_pn'], 'r-', label='North', alpha=0.8)
    axes[2].plot(df['time'], df['ekf_pe'] - df['true_pe'], 'g-', label='East', alpha=0.8)
    axes[2].plot(df['time'], df['ekf_pd'] - df['true_pd'], 'b-', label='Down', alpha=0.8)
    axes[2].set_ylabel("Position Error (m)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(0, color='k', alpha=0.2)
    axes[2].axvline(0, color='k', linestyle='--', alpha=0.3)

    fig.tight_layout()


def plot_imu_vs_truth(df, fig, title=""):
    """Page 4: IMU accelerometer vs true specific force (body FRD)."""
    fig.clear()
    has_imu = 'imu_acc_x' in df.columns and 'true_acc_x' in df.columns
    if not has_imu:
        ax = fig.add_subplot(111)
        ax.text(0.5, 0.5, "No IMU data available", ha='center', va='center',
                fontsize=14, transform=ax.transAxes)
        return

    axes = fig.subplots(4, 1, sharex=True)
    fig.suptitle(f"{title} — IMU Accel vs Truth (Body FRD)" if title else
                 "IMU Accel vs Truth (Body FRD)")

    axis_labels = [
        ('imu_acc_x', 'true_acc_x', 'Forward (X)'),
        ('imu_acc_y', 'true_acc_y', 'Right (Y)'),
        ('imu_acc_z', 'true_acc_z', 'Down (Z)'),
    ]

    for i, (imu_col, true_col, label) in enumerate(axis_labels):
        axes[i].plot(df['time'], df[imu_col], 'b-', alpha=0.4,
                     linewidth=0.5, label='IMU (noisy)')
        axes[i].plot(df['time'], df[true_col], 'r-', alpha=0.8,
                     linewidth=0.8, label='Truth')
        axes[i].set_ylabel(f"{label} (m/s²)")
        axes[i].legend(loc='upper right', fontsize=8)
        axes[i].grid(True, alpha=0.3)
        axes[i].axvline(0, color='k', linestyle='--', alpha=0.3)

    imu_mag = np.sqrt(df['imu_acc_x']**2 + df['imu_acc_y']**2 + df['imu_acc_z']**2)
    true_mag = np.sqrt(df['true_acc_x']**2 + df['true_acc_y']**2 + df['true_acc_z']**2)
    axes[3].plot(df['time'], imu_mag, 'b-', alpha=0.4, linewidth=0.5, label='IMU |a|')
    axes[3].plot(df['time'], true_mag, 'r-', alpha=0.8, linewidth=0.8, label='Truth |a|')
    axes[3].set_ylabel("|Accel| (m/s²)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc='upper right', fontsize=8)
    axes[3].grid(True, alpha=0.3)
    axes[3].axvline(0, color='k', linestyle='--', alpha=0.3)

    fig.tight_layout()


def plot_3d_trajectory(df, fig, title=""):
    """3D ENU trajectory plot."""
    fig.clear()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(df['x'], df['y'], df['z'], 'b-', linewidth=0.8)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")
    ax.set_title(f"{title} — 3D Trajectory" if title else "3D Trajectory")


# ============================================================================
# GUI Application
# ============================================================================

class SimGUI:
    """Main GUI application for TinkerRocket simulation."""

    # Default parameter values (matching run_closed_loop.py)
    DEFAULTS = {
        'ork_file': 'rockets/54mm_Roll_Control.ork',
        'kp': 0.04, 'ki': 0.001, 'kd': 0.001,
        'roll_mode': 'profile',  # 'rate' or 'profile'
        'roll_setpoint_dps': 0.0,
        'roll_profile_text': '0.0, 0.0\n1.0, 180.0',
        'kp_angle': 5.0,
        'control_enabled': True,
        'roll_disturbance': 0.0,
        'gain_v_ref': 95.0, 'gain_v_min': 30.0, 'gain_max_scale': 3.0,
        'defl_min': -10.0, 'defl_max': 10.0, 'servo_rate': 500.0,
        'wind_speed': 0.0, 'wind_dir': 0.0,
        'pad_time': 2.0, 'duration': 16.0, 'physics_dt': 0.001,
        'launch_angle': 89.0, 'heading': 0.0,
        'imu_rate': 1200.0, 'baro_rate': 500.0,
        'mag_rate': 1000.0, 'gnss_rate': 25.0,
    }

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("TinkerRocket Simulation")
        self.root.geometry("1400x900")
        self.root.minsize(1200, 700)

        # State
        self.result_df = None
        self.rocket_def = None
        self.sim_running = False
        self.vars = {}  # tk variables for config fields

        # Build UI
        self._build_menu()
        self._build_layout()

    def _build_menu(self):
        """Build the menu bar."""
        menubar = tk.Menu(self.root)
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Export CSV...", command=self._export_csv)
        file_menu.add_separator()
        file_menu.add_command(label="Quit", command=self.root.quit)
        menubar.add_cascade(label="File", menu=file_menu)
        self.root.config(menu=menubar)

    def _build_layout(self):
        """Build the main layout: sidebar + plot area."""
        # Main paned window
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left sidebar with scrollbar
        sidebar_frame = ttk.Frame(paned)
        paned.add(sidebar_frame, weight=0)

        canvas = tk.Canvas(sidebar_frame, width=320)
        scrollbar = ttk.Scrollbar(sidebar_frame, orient=tk.VERTICAL, command=canvas.yview)
        self.sidebar = ttk.Frame(canvas)

        self.sidebar.bind("<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.sidebar, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Build sidebar sections
        self._build_sidebar()

        # Right side: plot notebook
        plot_frame = ttk.Frame(paned)
        paned.add(plot_frame, weight=1)
        self._build_plot_area(plot_frame)

    def _make_entry(self, parent, label, key, default, row, width=10):
        """Create a labeled entry field and store the variable."""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky='w', padx=2, pady=1)
        var = tk.StringVar(value=str(default))
        entry = ttk.Entry(parent, textvariable=var, width=width)
        entry.grid(row=row, column=1, sticky='ew', padx=2, pady=1)
        self.vars[key] = var
        return entry

    def _build_sidebar(self):
        """Build configuration sections in the sidebar."""
        parent = self.sidebar

        # --- Rocket source ---
        grp = ttk.LabelFrame(parent, text="Rocket", padding=5)
        grp.pack(fill=tk.X, padx=5, pady=3)

        self.vars['ork_file'] = tk.StringVar(value=self.DEFAULTS['ork_file'])
        ttk.Label(grp, text=".ork file:").grid(row=0, column=0, sticky='w')
        ork_entry = ttk.Entry(grp, textvariable=self.vars['ork_file'], width=25)
        ork_entry.grid(row=0, column=1, sticky='ew', padx=2)
        ttk.Button(grp, text="...", width=3,
                   command=self._browse_ork).grid(row=0, column=2, padx=2)
        grp.columnconfigure(1, weight=1)

        # --- PID Control ---
        grp = ttk.LabelFrame(parent, text="PID Control", padding=5)
        grp.pack(fill=tk.X, padx=5, pady=3)
        grp.columnconfigure(1, weight=1)

        self._make_entry(grp, "Kp:", 'kp', self.DEFAULTS['kp'], 0)
        self._make_entry(grp, "Ki:", 'ki', self.DEFAULTS['ki'], 1)
        self._make_entry(grp, "Kd:", 'kd', self.DEFAULTS['kd'], 2)

        self.vars['control_enabled'] = tk.BooleanVar(value=self.DEFAULTS['control_enabled'])
        ttk.Checkbutton(grp, text="Control enabled",
                        variable=self.vars['control_enabled']).grid(
                            row=3, column=0, columnspan=2, sticky='w', pady=2)

        # --- Roll Mode ---
        grp = ttk.LabelFrame(parent, text="Roll Control Mode", padding=5)
        grp.pack(fill=tk.X, padx=5, pady=3)
        grp.columnconfigure(1, weight=1)

        self.vars['roll_mode'] = tk.StringVar(value=self.DEFAULTS['roll_mode'])
        ttk.Radiobutton(grp, text="Angle profile", variable=self.vars['roll_mode'],
                        value='profile', command=self._on_roll_mode_change).grid(
                            row=0, column=0, sticky='w')
        ttk.Radiobutton(grp, text="Constant rate", variable=self.vars['roll_mode'],
                        value='rate', command=self._on_roll_mode_change).grid(
                            row=0, column=1, sticky='w')

        self._make_entry(grp, "Rate (°/s):", 'roll_setpoint_dps',
                         self.DEFAULTS['roll_setpoint_dps'], 1)
        self._make_entry(grp, "Kp_angle:", 'kp_angle',
                         self.DEFAULTS['kp_angle'], 2)

        ttk.Label(grp, text="Profile (t, angle):").grid(row=3, column=0, sticky='nw', padx=2)
        self.vars['roll_profile_text'] = tk.StringVar(
            value=self.DEFAULTS['roll_profile_text'])
        self.profile_text = tk.Text(grp, height=3, width=20, font=('Courier', 10))
        self.profile_text.grid(row=3, column=1, sticky='ew', padx=2, pady=2)
        self.profile_text.insert('1.0', self.DEFAULTS['roll_profile_text'])

        # --- Actuator ---
        grp = ttk.LabelFrame(parent, text="Actuator", padding=5)
        grp.pack(fill=tk.X, padx=5, pady=3)
        grp.columnconfigure(1, weight=1)

        self._make_entry(grp, "Defl min (°):", 'defl_min', self.DEFAULTS['defl_min'], 0)
        self._make_entry(grp, "Defl max (°):", 'defl_max', self.DEFAULTS['defl_max'], 1)
        self._make_entry(grp, "Servo rate (°/s):", 'servo_rate',
                         self.DEFAULTS['servo_rate'], 2)

        # --- Gain Scheduling ---
        grp = ttk.LabelFrame(parent, text="Gain Schedule", padding=5)
        grp.pack(fill=tk.X, padx=5, pady=3)
        grp.columnconfigure(1, weight=1)

        self._make_entry(grp, "V_ref (m/s):", 'gain_v_ref',
                         self.DEFAULTS['gain_v_ref'], 0)
        self._make_entry(grp, "V_min (m/s):", 'gain_v_min',
                         self.DEFAULTS['gain_v_min'], 1)
        self._make_entry(grp, "Max scale:", 'gain_max_scale',
                         self.DEFAULTS['gain_max_scale'], 2)

        # --- Launch ---
        grp = ttk.LabelFrame(parent, text="Launch", padding=5)
        grp.pack(fill=tk.X, padx=5, pady=3)
        grp.columnconfigure(1, weight=1)

        self._make_entry(grp, "Angle (°):", 'launch_angle',
                         self.DEFAULTS['launch_angle'], 0)
        self._make_entry(grp, "Heading (°):", 'heading',
                         self.DEFAULTS['heading'], 1)
        self._make_entry(grp, "Pad time (s):", 'pad_time',
                         self.DEFAULTS['pad_time'], 2)
        self._make_entry(grp, "Duration (s):", 'duration',
                         self.DEFAULTS['duration'], 3)
        self._make_entry(grp, "Physics dt (s):", 'physics_dt',
                         self.DEFAULTS['physics_dt'], 4)

        # --- Wind ---
        grp = ttk.LabelFrame(parent, text="Wind", padding=5)
        grp.pack(fill=tk.X, padx=5, pady=3)
        grp.columnconfigure(1, weight=1)

        self._make_entry(grp, "Speed (m/s):", 'wind_speed',
                         self.DEFAULTS['wind_speed'], 0)
        self._make_entry(grp, "From (°):", 'wind_dir',
                         self.DEFAULTS['wind_dir'], 1)

        # --- Disturbance ---
        grp = ttk.LabelFrame(parent, text="Disturbances", padding=5)
        grp.pack(fill=tk.X, padx=5, pady=3)
        grp.columnconfigure(1, weight=1)

        self._make_entry(grp, "Roll torque (mN-m):", 'roll_disturbance',
                         self.DEFAULTS['roll_disturbance'], 0)

        # --- Run Button + Status ---
        run_frame = ttk.Frame(parent)
        run_frame.pack(fill=tk.X, padx=5, pady=10)

        self.run_btn = ttk.Button(run_frame, text="▶ RUN SIMULATION",
                                  command=self._run_sim, style='Accent.TButton')
        self.run_btn.pack(fill=tk.X, pady=5)

        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(run_frame, textvariable=self.status_var,
                                 font=('Helvetica', 10, 'bold'))
        status_label.pack(anchor='w')

        # Results display
        self.results_text = tk.Text(run_frame, height=8, width=35,
                                    font=('Courier', 9), state='disabled',
                                    background='#f5f5f5')
        self.results_text.pack(fill=tk.X, pady=5)

    def _build_plot_area(self, parent):
        """Build the plot notebook with tabs."""
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Define plot tabs
        self.plot_tabs = {}
        tab_defs = [
            ('flight', 'Flight Performance', plot_flight_performance),
            ('control', 'Attitude & Control', plot_attitude_control),
            ('ekf', 'EKF Performance', plot_ekf_performance),
            ('imu', 'IMU vs Truth', plot_imu_vs_truth),
            ('traj3d', '3D Trajectory', plot_3d_trajectory),
        ]

        for key, label, plot_func in tab_defs:
            frame = ttk.Frame(self.notebook)
            self.notebook.add(frame, text=label)

            fig = Figure(figsize=(10, 7), dpi=100)
            canvas = FigureCanvasTkAgg(fig, master=frame)

            # Toolbar
            toolbar_frame = ttk.Frame(frame)
            toolbar_frame.pack(side=tk.TOP, fill=tk.X)
            toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
            toolbar.update()

            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

            self.plot_tabs[key] = {
                'frame': frame,
                'fig': fig,
                'canvas': canvas,
                'plot_func': plot_func,
            }

        # Placeholder text on each tab
        for key, info in self.plot_tabs.items():
            ax = info['fig'].add_subplot(111)
            ax.text(0.5, 0.5, "Run simulation to see plots",
                    ha='center', va='center', fontsize=14, color='gray',
                    transform=ax.transAxes)
            ax.set_xticks([])
            ax.set_yticks([])
            info['canvas'].draw()

    def _on_roll_mode_change(self):
        """Handle roll mode radio button change."""
        pass  # Could enable/disable relevant widgets

    def _browse_ork(self):
        """Open file dialog for .ork file."""
        path = filedialog.askopenfilename(
            title="Select OpenRocket file",
            initialdir=PROJECT_DIR,
            filetypes=[("OpenRocket files", "*.ork"), ("All files", "*.*")]
        )
        if path:
            # Convert to relative path if possible
            try:
                rel = os.path.relpath(path, PROJECT_DIR)
                self.vars['ork_file'].set(rel)
            except ValueError:
                self.vars['ork_file'].set(path)

    def _get_float(self, key, default=0.0):
        """Get a float value from a tk variable."""
        try:
            return float(self.vars[key].get())
        except (ValueError, KeyError):
            return default

    def _get_roll_profile(self):
        """Parse roll profile from text widget."""
        text = self.profile_text.get('1.0', tk.END).strip()
        if not text:
            return None
        profile = []
        for line in text.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) == 2:
                try:
                    t_val = float(parts[0].strip())
                    a_val = float(parts[1].strip())
                    profile.append((t_val, a_val))
                except ValueError:
                    continue
        return profile if profile else None

    def _build_config(self):
        """Build SimConfig from GUI values."""
        roll_mode = self.vars['roll_mode'].get()
        roll_profile = self._get_roll_profile() if roll_mode == 'profile' else None

        return SimConfig(
            physics_dt=self._get_float('physics_dt', 0.001),
            imu_rate=self._get_float('imu_rate', 1200.0),
            baro_rate=self._get_float('baro_rate', 500.0),
            mag_rate=self._get_float('mag_rate', 1000.0),
            gnss_rate=self._get_float('gnss_rate', 25.0),
            duration=self._get_float('duration', 16.0),
            pad_time=self._get_float('pad_time', 2.0),
            launch_angle_deg=self._get_float('launch_angle', 89.0),
            heading_deg=self._get_float('heading', 0.0),
            pid_kp=self._get_float('kp', 0.04),
            pid_ki=self._get_float('ki', 0.001),
            pid_kd=self._get_float('kd', 0.001),
            roll_setpoint_dps=self._get_float('roll_setpoint_dps', 0.0),
            control_enabled=self.vars['control_enabled'].get(),
            roll_profile=roll_profile,
            kp_angle=self._get_float('kp_angle', 5.0),
            gain_V_ref=self._get_float('gain_v_ref', 95.0),
            gain_V_min=self._get_float('gain_v_min', 30.0),
            gain_max_scale=self._get_float('gain_max_scale', 3.0),
            deflection_min=self._get_float('defl_min', -10.0),
            deflection_max=self._get_float('defl_max', 10.0),
            servo_rate_limit=self._get_float('servo_rate', 500.0),
            wind_speed=self._get_float('wind_speed', 0.0),
            wind_direction_deg=self._get_float('wind_dir', 0.0),
        )

    def _load_rocket(self):
        """Load rocket definition from .ork file."""
        ork_rel = self.vars['ork_file'].get()
        ork_path = os.path.join(PROJECT_DIR, ork_rel)
        if not os.path.exists(ork_path):
            ork_path = ork_rel  # try absolute
        rd = from_ork(ork_path)
        # Apply disturbance
        rd.roll_disturbance_torque = self._get_float('roll_disturbance', 0.0) * 0.001  # mN-m → N-m
        return rd

    def _run_sim(self):
        """Run simulation in background thread."""
        if self.sim_running:
            return

        self.sim_running = True
        self.run_btn.config(state='disabled')
        self.status_var.set("Loading rocket...")

        def worker():
            try:
                # Load rocket
                rd = self._load_rocket()
                self.rocket_def = rd

                self.root.after(0, lambda: self.status_var.set("Running simulation..."))

                # Build config
                cfg = self._build_config()

                # Run
                t0 = time.time()
                result = run_closed_loop(rd, cfg)
                elapsed = time.time() - t0

                # Update UI on main thread
                self.root.after(0, lambda: self._on_sim_complete(result, elapsed))

            except Exception as e:
                import traceback
                tb = traceback.format_exc()
                self.root.after(0, lambda: self._on_sim_error(str(e), tb))

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

    def _on_sim_complete(self, result, elapsed):
        """Called on main thread when simulation completes."""
        self.sim_running = False
        self.run_btn.config(state='normal')
        self.result_df = result.df

        self.status_var.set(f"Done ({elapsed:.1f}s)")

        # Update results text
        self.results_text.config(state='normal')
        self.results_text.delete('1.0', tk.END)

        df = result.df
        rd = self.rocket_def
        burn_time = rd.motor.burn_time if rd else 0

        lines = [
            f"Apogee:      {result.apogee_m:.1f} m",
            f"Max speed:   {result.max_speed_mps:.1f} m/s",
            f"Flight time: {result.flight_time_s:.1f} s",
            f"Data points: {len(df)}",
            f"Wall-clock:  {elapsed:.1f} s",
        ]

        # Roll stats
        if 'roll_rate_dps' in df.columns:
            boost = df[(df['time'] > 0.5) & (df['time'] < burn_time)]
            if len(boost) > 0:
                lines.append(f"")
                lines.append(f"Boost roll rate:")
                lines.append(f"  mean={boost['roll_rate_dps'].mean():+.2f} °/s")
                lines.append(f"  std={boost['roll_rate_dps'].std():.2f} °/s")

        # AoA
        if 'alpha_deg' in df.columns:
            flight = df[df['speed'] > 5.0]
            if len(flight) > 0:
                lines.append(f"Max AoA: {flight['alpha_deg'].max():.2f}°")

        self.results_text.insert('1.0', '\n'.join(lines))
        self.results_text.config(state='disabled')

        # Update all plot tabs
        title = rd.name if rd else "Simulation"
        for key, info in self.plot_tabs.items():
            try:
                info['plot_func'](df, info['fig'], title=title)
                info['canvas'].draw()
            except Exception as e:
                print(f"Plot error ({key}): {e}")

    def _on_sim_error(self, error_msg, traceback_str=""):
        """Called on main thread when simulation fails."""
        self.sim_running = False
        self.run_btn.config(state='normal')
        self.status_var.set(f"ERROR")

        self.results_text.config(state='normal')
        self.results_text.delete('1.0', tk.END)
        self.results_text.insert('1.0', f"Error: {error_msg}\n\n{traceback_str}")
        self.results_text.config(state='disabled')

    def _export_csv(self):
        """Export simulation data to CSV."""
        if self.result_df is None:
            return

        path = filedialog.asksaveasfilename(
            title="Export CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if path:
            self.result_df.to_csv(path, index=False)
            self.status_var.set(f"Exported to {os.path.basename(path)}")

    def run(self):
        """Start the GUI event loop."""
        self.root.mainloop()


# ============================================================================
# Entry point
# ============================================================================

if __name__ == "__main__":
    app = SimGUI()
    app.run()
