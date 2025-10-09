#!/usr/bin/env python3
"""
Advanced Joint Control GUI for PID Tuning

Features:
- Individual joint position sliders with live control
- Enable/Disable buttons for isolated joint testing
- Real-time effort (torque) visualization per joint
- Joint state monitoring (position, velocity, effort)

Perfect for PID tuning - control one joint at a time while monitoring torque output!
"""

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import threading
import time


class JointControlGUI:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title("🦾 Joint Control & Effort Monitor - Humanoid Arm")
        self.window.geometry("1000x700")
        self.window.configure(bg='#f0f0f0')

        # Joint configuration
        self.joints = [
            {'name': 'base_rotation_joint', 'min': -3.14, 'max': 3.14, 'color': '#FF5252'},
            {'name': 'shoulder_pitch_joint', 'min': -0.55, 'max': 3.1, 'color': '#FF9800'},
            {'name': 'elbow_pitch_joint', 'min': -3.14, 'max': 3.14, 'color': '#4CAF50'},
            {'name': 'wrist_pitch_joint', 'min': -0.31, 'max': 2.8, 'color': '#2196F3'},
            {'name': 'wrist_roll_joint', 'min': -3.14, 'max': 3.14, 'color': '#9C27B0'},
        ]

        # Joint states
        self.joint_enabled = {j['name']: tk.BooleanVar(value=False) for j in self.joints}
        self.joint_positions = {j['name']: 0.0 for j in self.joints}
        self.joint_velocities = {j['name']: 0.0 for j in self.joints}
        self.joint_efforts = {j['name']: 0.0 for j in self.joints}

        # GUI elements storage
        self.sliders = {}
        self.position_labels = {}
        self.velocity_labels = {}
        self.effort_bars = {}
        self.effort_value_labels = {}
        self.enable_buttons = {}

        # Max effort for visualization (will be auto-scaled)
        self.max_effort_display = 50.0  # Nm (adjust based on your motors)

        # Setup GUI
        self.setup_gui()

        # Update timer for effort bars
        self.update_displays()

    def setup_gui(self):
        """Setup the main GUI layout"""
        # Title
        title_frame = tk.Frame(self.window, bg='#2c3e50', height=60)
        title_frame.pack(fill='x')

        title = tk.Label(title_frame, text="🦾 Joint Control & Effort Monitor",
                        font=("Arial", 18, "bold"), bg='#2c3e50', fg='white')
        title.pack(pady=15)

        # Info bar
        info_frame = tk.Frame(self.window, bg='#34495e')
        info_frame.pack(fill='x')

        info_text = "Enable joints individually to test PID tuning | Watch effort (torque) in real-time"
        info_label = tk.Label(info_frame, text=info_text, font=("Arial", 10),
                            bg='#34495e', fg='#ecf0f1')
        info_label.pack(pady=5)

        # Main content frame
        content_frame = tk.Frame(self.window, bg='#f0f0f0')
        content_frame.pack(fill='both', expand=True, padx=10, pady=10)

        # Create a frame for each joint
        for joint_config in self.joints:
            self.create_joint_control(content_frame, joint_config)

        # Status bar at bottom
        self.status_label = tk.Label(self.window, text="Ready - Enable joints to start",
                                     font=("Arial", 10), bg='#4CAF50', fg='white',
                                     relief='sunken', anchor='w', padx=10)
        self.status_label.pack(side=tk.BOTTOM, fill='x')

        # Emergency stop button
        estop_btn = tk.Button(self.window, text="⚠ STOP ALL",
                             command=self.emergency_stop,
                             bg='#f44336', fg='white',
                             font=("Arial", 14, "bold"), height=2)
        estop_btn.pack(side=tk.BOTTOM, fill='x', pady=5, padx=10)

    def create_joint_control(self, parent, joint_config):
        """Create control panel for a single joint"""
        joint_name = joint_config['name']
        color = joint_config['color']

        # Main frame for this joint
        joint_frame = tk.LabelFrame(parent, text=f"  {joint_name}  ",
                                   font=("Arial", 11, "bold"),
                                   bg='white', relief='ridge', bd=2)
        joint_frame.pack(fill='x', pady=5, padx=5)

        # Top row: Enable button and current state
        top_row = tk.Frame(joint_frame, bg='white')
        top_row.pack(fill='x', padx=10, pady=5)

        # Enable/Disable button
        enable_btn = tk.Checkbutton(
            top_row,
            text="ENABLE",
            variable=self.joint_enabled[joint_name],
            command=lambda: self.on_enable_toggle(joint_name),
            font=("Arial", 10, "bold"),
            bg='white',
            activebackground='white',
            selectcolor=color,
            indicatoron=False,
            width=10,
            relief='raised',
            bd=3
        )
        enable_btn.pack(side=tk.LEFT, padx=5)
        self.enable_buttons[joint_name] = enable_btn

        # Current state display
        state_frame = tk.Frame(top_row, bg='white')
        state_frame.pack(side=tk.LEFT, padx=20)

        tk.Label(state_frame, text="Pos:", font=("Arial", 9), bg='white').grid(row=0, column=0, sticky='e')
        pos_lbl = tk.Label(state_frame, text="0.00 rad", font=("Arial", 9, "bold"),
                          bg='#e0e0e0', width=10, relief='sunken')
        pos_lbl.grid(row=0, column=1, padx=5)
        self.position_labels[joint_name] = pos_lbl

        tk.Label(state_frame, text="Vel:", font=("Arial", 9), bg='white').grid(row=0, column=2, sticky='e', padx=(10,0))
        vel_lbl = tk.Label(state_frame, text="0.00 rad/s", font=("Arial", 9, "bold"),
                          bg='#e0e0e0', width=10, relief='sunken')
        vel_lbl.grid(row=0, column=3, padx=5)
        self.velocity_labels[joint_name] = vel_lbl

        # Middle row: Position slider
        slider_frame = tk.Frame(joint_frame, bg='white')
        slider_frame.pack(fill='x', padx=10, pady=5)

        tk.Label(slider_frame, text="Target Position:", font=("Arial", 9, "bold"),
                bg='white').pack(side=tk.LEFT, padx=5)

        slider = tk.Scale(
            slider_frame,
            from_=joint_config['min'],
            to=joint_config['max'],
            resolution=0.01,
            orient=tk.HORIZONTAL,
            length=500,
            bg=color,
            troughcolor='#ecf0f1',
            command=lambda val, jn=joint_name: self.on_slider_change(jn, val),
            state=tk.DISABLED  # Start disabled
        )
        slider.pack(side=tk.LEFT, fill='x', expand=True, padx=10)
        slider.set(0.0)  # Start at zero
        self.sliders[joint_name] = slider

        # Bottom row: Effort bar
        effort_frame = tk.Frame(joint_frame, bg='white')
        effort_frame.pack(fill='x', padx=10, pady=5)

        tk.Label(effort_frame, text="Effort (Torque):", font=("Arial", 9, "bold"),
                bg='white').pack(side=tk.LEFT, padx=5)

        # Canvas for effort bar
        canvas = tk.Canvas(effort_frame, height=25, bg='#ecf0f1', relief='sunken', bd=2)
        canvas.pack(side=tk.LEFT, fill='x', expand=True, padx=10)

        # Create effort bar rectangle (starts at zero width)
        bar = canvas.create_rectangle(0, 0, 0, 25, fill=color, outline='')
        self.effort_bars[joint_name] = {'canvas': canvas, 'bar': bar, 'color': color}

        # Effort value label
        effort_val_lbl = tk.Label(effort_frame, text="0.0 Nm", font=("Arial", 9, "bold"),
                                 bg='#e0e0e0', width=10, relief='sunken')
        effort_val_lbl.pack(side=tk.LEFT, padx=5)
        self.effort_value_labels[joint_name] = effort_val_lbl

    def on_enable_toggle(self, joint_name):
        """Handle enable/disable button toggle"""
        is_enabled = self.joint_enabled[joint_name].get()

        if is_enabled:
            # Enable this joint
            self.sliders[joint_name].config(state=tk.NORMAL)
            self.enable_buttons[joint_name].config(bg='#4CAF50', fg='white', text='ENABLED')

            # Send current slider position
            current_pos = self.sliders[joint_name].get()
            self.send_joint_command(joint_name, current_pos)

            self.status_label.config(text=f"✓ {joint_name} enabled", bg='#4CAF50')
        else:
            # Disable this joint
            self.sliders[joint_name].config(state=tk.DISABLED)
            self.enable_buttons[joint_name].config(bg='#f0f0f0', fg='black', text='ENABLE')
            self.status_label.config(text=f"○ {joint_name} disabled", bg='#FF9800')

    def on_slider_change(self, joint_name, value):
        """Handle slider movement - send command if joint is enabled"""
        if self.joint_enabled[joint_name].get():
            self.send_joint_command(joint_name, float(value))

    def send_joint_command(self, joint_name, position):
        """Send trajectory command for single joint"""
        if not self.joint_enabled[joint_name].get():
            return

        # Create trajectory message for this joint only
        traj = JointTrajectory()
        traj.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5 seconds

        traj.points = [point]

        # Publish
        self.node.publish_trajectory(traj)

    def update_joint_state(self, joint_name, position, velocity, effort):
        """Update stored joint state (called by ROS callback)"""
        self.joint_positions[joint_name] = position
        self.joint_velocities[joint_name] = velocity
        self.joint_efforts[joint_name] = effort

    def update_displays(self):
        """Update all display elements (called periodically)"""
        for joint_name in self.joint_positions.keys():
            # Update position label
            pos = self.joint_positions[joint_name]
            self.position_labels[joint_name].config(text=f"{pos:.2f} rad")

            # Update velocity label
            vel = self.joint_velocities[joint_name]
            self.velocity_labels[joint_name].config(text=f"{vel:.2f} rad/s")

            # Update effort bar and label
            effort = self.joint_efforts[joint_name]
            self.effort_value_labels[joint_name].config(text=f"{effort:.1f} Nm")

            # Update effort bar visualization
            canvas = self.effort_bars[joint_name]['canvas']
            bar = self.effort_bars[joint_name]['bar']
            color = self.effort_bars[joint_name]['color']

            canvas_width = canvas.winfo_width()
            if canvas_width > 1:  # Canvas is initialized
                # Scale effort to bar width (center at zero, grows left/right)
                normalized_effort = effort / self.max_effort_display
                normalized_effort = max(-1.0, min(1.0, normalized_effort))  # Clamp

                center = canvas_width / 2
                bar_width = abs(normalized_effort) * center

                if normalized_effort >= 0:
                    # Positive effort - bar grows right
                    canvas.coords(bar, center, 0, center + bar_width, 25)
                    canvas.itemconfig(bar, fill=color)
                else:
                    # Negative effort - bar grows left
                    canvas.coords(bar, center - bar_width, 0, center, 25)
                    canvas.itemconfig(bar, fill='#f44336')  # Red for negative

        # Schedule next update
        self.window.after(50, self.update_displays)  # 20 Hz update rate

    def emergency_stop(self):
        """Disable all joints immediately"""
        for joint_name in self.joint_enabled.keys():
            self.joint_enabled[joint_name].set(False)
            self.sliders[joint_name].config(state=tk.DISABLED)
            self.enable_buttons[joint_name].config(bg='#f0f0f0', fg='black', text='ENABLE')

        self.status_label.config(text="⚠ EMERGENCY STOP - All joints disabled", bg='#f44336')

    def run(self):
        """Start the GUI main loop"""
        self.window.mainloop()


class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_gui')

        # Publisher for trajectory commands
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.gui = None
        self.get_logger().info('Joint Control Node initialized')

    def set_gui(self, gui):
        """Set reference to GUI for callbacks"""
        self.gui = gui

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        if self.gui is None:
            return

        for i, name in enumerate(msg.name):
            if i < len(msg.position) and i < len(msg.velocity) and i < len(msg.effort):
                self.gui.update_joint_state(
                    name,
                    msg.position[i],
                    msg.velocity[i],
                    msg.effort[i]
                )

    def publish_trajectory(self, trajectory):
        """Publish trajectory command"""
        self.traj_pub.publish(trajectory)


def spin_ros(node):
    """Spin ROS in background thread"""
    rclpy.spin(node)


def main():
    rclpy.init()

    # Create ROS node
    node = JointControlNode()

    # Start ROS spinning in background thread
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    # Create and run GUI in main thread
    gui = JointControlGUI(node)
    node.set_gui(gui)

    gui.run()

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
