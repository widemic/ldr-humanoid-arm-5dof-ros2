#!/usr/bin/env python3
"""
Simple GUI for tuning PID parameters in real-time
Uses tkinter for cross-platform compatibility
"""

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
import threading


class PIDTunerGUI:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title("🎛️ PID Tuner - Humanoid Arm 5-DOF")
        self.window.geometry("800x600")

        # Joint names
        self.joints = [
            'base_rotation_joint',
            'shoulder_pitch_joint',
            'elbow_pitch_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]

        # Current values storage
        self.current_values = {}

        # Detect active controller
        self.detect_controller()

        # Setup GUI
        self.setup_gui()

        # Load current parameters
        self.load_current_parameters()

    def detect_controller(self):
        """Detect which controller is active"""
        # Try to detect active single-joint controllers first (preferred for isolated tuning)
        try:
            import subprocess
            result = subprocess.run(['ros2', 'control', 'list_controllers'],
                                  capture_output=True, text=True, timeout=2)
            output = result.stdout

            # Check for single-joint controllers first (higher priority)
            for joint in self.joints:
                controller = f'{joint}_position_controller'
                if controller in output and 'active' in output:
                    self.controller_name = controller
                    print(f"[INFO] Detected active single-joint controller: {self.controller_name}")
                    return  # Found single-joint controller, use it!

            # Fall back to multi-joint controller if no single-joint found
            if 'joint_trajectory_controller' in output and 'active' in output:
                self.controller_name = 'joint_trajectory_controller'
                print(f"[INFO] Using multi-joint controller: joint_trajectory_controller")
                return

            # No active controller found - default to joint_trajectory_controller
            print(f"[WARN] No active controller detected, defaulting to joint_trajectory_controller")
            self.controller_name = 'joint_trajectory_controller'

        except Exception as e:
            print(f"[WARN] Could not detect controller: {e}, defaulting to joint_trajectory_controller")
            self.controller_name = 'joint_trajectory_controller'

    def setup_gui(self):
        # Title
        title = tk.Label(self.window, text=f"PID Tuner - {self.controller_name}",
                        font=("Arial", 16, "bold"))
        title.pack(pady=10)

        # Controller info
        controller_label = tk.Label(self.window, text=f"Controller: {self.controller_name}",
                                    font=("Arial", 10), fg="blue")
        controller_label.pack(pady=2)

        # Joint selector
        joint_frame = tk.Frame(self.window)
        joint_frame.pack(pady=10)

        tk.Label(joint_frame, text="Select Joint:", font=("Arial", 12)).pack(side=tk.LEFT, padx=5)

        self.joint_var = tk.StringVar(value=self.joints[0])
        joint_menu = ttk.Combobox(joint_frame, textvariable=self.joint_var,
                                  values=self.joints, state="readonly", width=25)
        joint_menu.pack(side=tk.LEFT, padx=5)
        joint_menu.bind('<<ComboboxSelected>>', self.on_joint_changed)

        # Load button
        load_btn = tk.Button(joint_frame, text="🔄 Reload", command=self.load_current_parameters,
                            bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        load_btn.pack(side=tk.LEFT, padx=10)

        # Separator
        ttk.Separator(self.window, orient='horizontal').pack(fill='x', pady=10)

        # PID Parameters frame
        params_frame = tk.Frame(self.window)
        params_frame.pack(pady=10, padx=20, fill='both', expand=True)

        # Create sliders for each parameter
        self.sliders = {}
        self.value_labels = {}

        params = [
            ('P Gain', 'p', 0, 500, 1),
            ('I Gain', 'i', 0, 50, 0.1),
            ('D Gain', 'd', 0, 50, 0.1),
            ('I Clamp', 'i_clamp', 0, 100, 1),
            ('FF Velocity', 'ff_velocity_scale', 0, 1.0, 0.01),
        ]

        for idx, (label, param_name, min_val, max_val, resolution) in enumerate(params):
            frame = tk.Frame(params_frame)
            frame.pack(fill='x', pady=10)

            # Label
            lbl = tk.Label(frame, text=label, font=("Arial", 11, "bold"), width=15, anchor='w')
            lbl.pack(side=tk.LEFT, padx=5)

            # Value display
            val_lbl = tk.Label(frame, text="0.00", font=("Arial", 11), width=10,
                              bg="#e0e0e0", relief='sunken')
            val_lbl.pack(side=tk.LEFT, padx=5)
            self.value_labels[param_name] = val_lbl

            # Slider
            slider = tk.Scale(frame, from_=min_val, to=max_val, resolution=resolution,
                            orient=tk.HORIZONTAL, length=400,
                            command=lambda val, p=param_name: self.on_slider_changed(p, val))
            slider.pack(side=tk.LEFT, padx=5, fill='x', expand=True)
            self.sliders[param_name] = slider

        # Separator
        ttk.Separator(self.window, orient='horizontal').pack(fill='x', pady=10)

        # Control buttons
        button_frame = tk.Frame(self.window)
        button_frame.pack(pady=10)

        apply_btn = tk.Button(button_frame, text="✓ Apply Changes",
                             command=self.apply_parameters,
                             bg="#2196F3", fg="white", font=("Arial", 12, "bold"),
                             width=15, height=2)
        apply_btn.pack(side=tk.LEFT, padx=10)

        reset_btn = tk.Button(button_frame, text="↺ Reset to Current",
                             command=self.reset_sliders,
                             bg="#FF9800", fg="white", font=("Arial", 12, "bold"),
                             width=15, height=2)
        reset_btn.pack(side=tk.LEFT, padx=10)

        # Status label
        self.status_label = tk.Label(self.window, text="Ready", font=("Arial", 10),
                                     bg="#4CAF50", fg="white", relief='sunken')
        self.status_label.pack(side=tk.BOTTOM, fill='x', pady=5)

    def on_slider_changed(self, param_name, value):
        """Update value label when slider moves"""
        self.value_labels[param_name].config(text=f"{float(value):.2f}")

    def on_joint_changed(self, event):
        """Load parameters for newly selected joint"""
        self.load_current_parameters()

    def load_current_parameters(self):
        """Load current PID parameters from controller"""
        joint = self.joint_var.get()
        self.status_label.config(text=f"Loading parameters for {joint}...", bg="#FF9800")
        self.window.update()

        try:
            # Get parameters from controller
            param_names = [
                f'gains.{joint}.p',
                f'gains.{joint}.i',
                f'gains.{joint}.d',
                f'gains.{joint}.i_clamp',
                f'gains.{joint}.ff_velocity_scale',
            ]

            print(f"[DEBUG] Requesting parameters: {param_names}")
            values = self.node.get_parameters(param_names)
            print(f"[DEBUG] Received values: {values}")

            if values:
                self.current_values[joint] = values
                self.reset_sliders()
                self.status_label.config(text=f"✓ Loaded: P={values.get('p', 0):.1f} D={values.get('d', 0):.1f}", bg="#4CAF50")
            else:
                # Set default values
                default_values = {'p': 150.0, 'i': 5.0, 'd': 8.0, 'i_clamp': 20.0, 'ff_velocity_scale': 0.1}
                self.current_values[joint] = default_values
                self.reset_sliders()
                self.status_label.config(text="⚠ Using default values", bg="#FF9800")

        except Exception as e:
            print(f"[ERROR] {str(e)}")
            import traceback
            traceback.print_exc()
            self.status_label.config(text=f"✗ Error: {str(e)}", bg="#f44336")

    def reset_sliders(self):
        """Reset sliders to current values"""
        joint = self.joint_var.get()

        if joint in self.current_values:
            values = self.current_values[joint]

            param_map = {
                'p': values.get('p', 100.0),
                'i': values.get('i', 0.0),
                'd': values.get('d', 0.0),
                'i_clamp': values.get('i_clamp', 0.0),
                'ff_velocity_scale': values.get('ff_velocity_scale', 0.0),
            }

            for param_name, value in param_map.items():
                self.sliders[param_name].set(value)
                self.value_labels[param_name].config(text=f"{float(value):.2f}")

    def apply_parameters(self):
        """Apply current slider values to controller"""
        joint = self.joint_var.get()
        self.status_label.config(text=f"Applying parameters to {joint}...", bg="#2196F3")
        self.window.update()

        try:
            params = {}
            for param_name, slider in self.sliders.items():
                params[param_name] = slider.get()

            success = self.node.set_gains(joint, **params)

            if success:
                self.current_values[joint] = params
                self.status_label.config(text=f"✓ Parameters applied to {joint}", bg="#4CAF50")
            else:
                self.status_label.config(text="✗ Failed to apply parameters", bg="#f44336")

        except Exception as e:
            self.status_label.config(text=f"✗ Error: {str(e)}", bg="#f44336")

    def run(self):
        """Start the GUI"""
        self.window.mainloop()


class PIDTunerNode(Node):
    def __init__(self, controller_name='joint_trajectory_controller'):
        super().__init__('pid_tuner_gui')

        self.controller_name = controller_name

        self.set_param_client = self.create_client(
            SetParameters,
            f'/{controller_name}/set_parameters'
        )

        self.get_param_client = self.create_client(
            GetParameters,
            f'/{controller_name}/get_parameters'
        )

        # Wait for services
        self.get_logger().info(f'Waiting for parameter services from {controller_name}...')
        self.set_param_client.wait_for_service(timeout_sec=5.0)
        self.get_param_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('Services ready!')

    def get_parameters(self, param_names):
        """Get parameters from controller"""
        request = GetParameters.Request()
        request.names = param_names

        future = self.get_param_client.call_async(request)

        # Wait for result without blocking (node is already spinning in background)
        import time
        timeout = 2.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if future.done() and future.result() is not None:
            result = {}
            for name, value in zip(param_names, future.result().values):
                param_key = name.split('.')[-1]  # Get last part (p, i, d, etc.)
                # Handle different parameter types
                if value.type == ParameterType.PARAMETER_DOUBLE:
                    result[param_key] = value.double_value
                elif value.type == ParameterType.PARAMETER_INTEGER:
                    result[param_key] = float(value.integer_value)
                else:
                    self.get_logger().warn(f"Unexpected parameter type for {name}: {value.type}")

            self.get_logger().info(f"Retrieved parameters: {result}")
            return result if result else None

        self.get_logger().error(f"Failed to get parameters - timeout or error (done={future.done()})")
        return None

    def set_gains(self, joint_name, p=None, i=None, d=None, i_clamp=None, ff_velocity_scale=None):
        """Set PID gains for a specific joint"""
        params = []

        if p is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.p'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(p))
            params.append(param)

        if i is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.i'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(i))
            params.append(param)

        if d is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.d'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(d))
            params.append(param)

        if i_clamp is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.i_clamp'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(i_clamp))
            params.append(param)

        if ff_velocity_scale is not None:
            param = Parameter()
            param.name = f'gains.{joint_name}.ff_velocity_scale'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(ff_velocity_scale))
            params.append(param)

        request = SetParameters.Request()
        request.parameters = params

        future = self.set_param_client.call_async(request)

        # Wait for result without blocking
        import time
        timeout = 2.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if future.done() and future.result() is not None:
            return all(r.successful for r in future.result().results)
        return False


def spin_ros(node):
    """Spin ROS in background thread"""
    rclpy.spin(node)


def main():
    rclpy.init()

    # Detect which controller is active (prioritize single-joint for isolation)
    import subprocess
    controller_name = 'joint_trajectory_controller'  # Default fallback

    try:
        result = subprocess.run(['ros2', 'control', 'list_controllers'],
                              capture_output=True, text=True, timeout=2)
        output = result.stdout

        # Check for single-joint controllers FIRST (preferred for isolated tuning)
        joints = ['base_rotation_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint',
                 'wrist_pitch_joint', 'wrist_roll_joint']

        for joint in joints:
            controller = f'{joint}_position_controller'
            if controller in output and 'active' in output:
                controller_name = controller
                print(f"[INFO] Detected active single-joint controller: {controller_name}")
                break  # Found single-joint controller, use it!
        else:
            # Only check for multi-joint if no single-joint found
            if 'joint_trajectory_controller' in output and 'active' in output:
                controller_name = 'joint_trajectory_controller'
                print(f"[INFO] Using multi-joint controller: joint_trajectory_controller")

    except Exception as e:
        print(f"[WARN] Could not detect controller: {e}, using default")

    node = PIDTunerNode(controller_name=controller_name)

    # Start ROS spin in background thread
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    # Start GUI in main thread
    gui = PIDTunerGUI(node)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()