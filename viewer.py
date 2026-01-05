#!/usr/bin/env python3
"""
Viser-based IMU Visualization

Provides real-time 3D orientation and time series visualization for BNO080 IMU data.
Access the visualization at http://localhost:8080
"""

import viser
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from collections import deque
import math
import time


class IMUViewer:
    """Real-time IMU visualization using Viser."""

    def __init__(self, port=8080, buffer_size=200, axis_transform=None):
        """
        Initialize Viser-based IMU visualizer.

        Args:
            port: Web server port (default 8080)
            buffer_size: Number of data points to display (default 200 = 5s at 40Hz)
            axis_transform: Optional axis transformation matrix (3x3) or preset string
                          Options: None, 'swap_xy', 'swap_xz', 'swap_yz',
                                   'bno_to_visual' (try this first!)
        """
        print(f"Starting Viser server on port {port}...")
        self.server = viser.ViserServer(port=port)
        self.buffer_size = buffer_size

        # Axis transformation configuration
        self.axis_transform = self._setup_axis_transform(axis_transform)
        if axis_transform:
            print(f"Using axis transformation: {axis_transform}")

        # Time tracking
        self.start_time = None

        # Data buffers for time series
        self.time_buffer = deque(maxlen=buffer_size)
        self.gyro_buffer = {
            'x': deque(maxlen=buffer_size),
            'y': deque(maxlen=buffer_size),
            'z': deque(maxlen=buffer_size)
        }
        self.accel_buffer = {
            'x': deque(maxlen=buffer_size),
            'y': deque(maxlen=buffer_size),
            'z': deque(maxlen=buffer_size)
        }
        self.mag_buffer = {
            'x': deque(maxlen=buffer_size),
            'y': deque(maxlen=buffer_size),
            'z': deque(maxlen=buffer_size)
        }
        self.linear_accel_buffer = {
            'x': deque(maxlen=buffer_size),
            'y': deque(maxlen=buffer_size),
            'z': deque(maxlen=buffer_size)
        }
        self.euler_buffer = {
            'roll': deque(maxlen=buffer_size),
            'pitch': deque(maxlen=buffer_size),
            'yaw': deque(maxlen=buffer_size)
        }

        # Setup 3D visualization
        self._setup_3d_scene()

        # Setup time series plots
        self._setup_plots()

        print(f"✓ Visualization ready at http://localhost:{port}")

    def _setup_axis_transform(self, transform):
        """
        Setup axis transformation matrix.

        Common BNO080 issue: Sensor axes don't match visualization axes.
        This allows remapping axes to fix orientation mismatches.
        """
        if transform is None:
            return None

        # Predefined transformations
        # Based on manual BNO080 testing:
        #   - Physical Z (perpendicular) shows as Visual X (red) without transform
        #   - Physical X (along length) shows as Visual Z (blue) without transform
        #   - Solution: swap_xz to fix the mapping
        #   - Additional issue: Z axis inverted (pointing down instead of up)
        #   - Final solution: swap_xz_flipz combines swap + 180° rotation
        presets = {
            'swap_xy': np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]]),  # Swap X and Y
            'swap_xz': np.array([[0, 0, 1], [0, 1, 0], [1, 0, 0]]),  # Swap X and Z
            'swap_yz': np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]]),  # Swap Y and Z
            'flip_z': np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]]),  # Flip Z direction only
            'swap_xz_flipz': np.array([[0, 0, -1], [0, 1, 0], [-1, 0, 0]]),  # Swap X↔Z + flip both
            'swap_xz_flipy': np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]]),  # Swap X↔Z + flip Y only
            'swap_xz_rot180y': np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]]),  # Swap X↔Z + 180° around Y
            'bno080_fix': np.array([[0, 0, 1], [0, -1, 0], [-1, 0, 0]]),  # Swap X↔Z + flip both Y and Z (BNO080 SparkFun board)
        }

        if isinstance(transform, str):
            if transform in presets:
                return presets[transform]
            else:
                print(f"Warning: Unknown preset '{transform}', using no transform")
                return None
        elif isinstance(transform, np.ndarray):
            return transform
        else:
            return None

    def _transform_quaternion(self, quat_dict):
        """
        Apply axis transformation to quaternion if configured.

        Args:
            quat_dict: Dictionary with 'w', 'x', 'y', 'z' keys

        Returns:
            Transformed quaternion dictionary
        """
        if self.axis_transform is None:
            return quat_dict

        from scipy.spatial.transform import Rotation

        # First: Apply axis transformation
        quat_array = [quat_dict['x'], quat_dict['y'], quat_dict['z'], quat_dict['w']]  # scipy uses x,y,z,w
        rot = Rotation.from_quat(quat_array)
        rot_matrix = rot.as_matrix()

        # Apply axis transformation: R_new = T * R_old * T^T
        rot_matrix_transformed = self.axis_transform @ rot_matrix @ self.axis_transform.T

        # Convert back to quaternion
        rot_transformed = Rotation.from_matrix(rot_matrix_transformed)

        # Second: Apply 180° correction rotation around Y axis (to flip the IMU right-side up)
        # 180° rotation around Y = quaternion (w=0, x=0, y=1, z=0)
        correction_rot = Rotation.from_quat([0, 1, 0, 0])  # 180° around Y

        # Multiply quaternions: final = correction * transformed
        final_rot = correction_rot * rot_transformed

        quat_final = final_rot.as_quat()  # Returns [x, y, z, w]

        return {
            'w': quat_final[3],
            'x': quat_final[0],
            'y': quat_final[1],
            'z': quat_final[2]
        }

    def _transform_euler(self, quat_dict):
        """
        Calculate Euler angles from transformed quaternion if transformation is active.

        Args:
            quat_dict: Original quaternion dictionary with 'w', 'x', 'y', 'z' keys

        Returns:
            Transformed euler angles dictionary with 'roll', 'pitch', 'yaw' keys (in radians)
            or None if no transformation is active
        """
        if self.axis_transform is None:
            return None  # Use original euler angles

        # Get transformed quaternion
        transformed_quat = self._transform_quaternion(quat_dict)

        # Convert transformed quaternion to Euler angles
        from scipy.spatial.transform import Rotation
        quat_array = [
            transformed_quat['x'],
            transformed_quat['y'],
            transformed_quat['z'],
            transformed_quat['w']
        ]
        rot = Rotation.from_quat(quat_array)

        # Get Euler angles in ZYX order (yaw, pitch, roll)
        euler_zyx = rot.as_euler('ZYX', degrees=False)

        return {
            'yaw': euler_zyx[0],    # Z rotation
            'pitch': euler_zyx[1],  # Y rotation
            'roll': euler_zyx[2]    # X rotation
        }

    def _setup_3d_scene(self):
        """
        Setup 3D coordinate frame and IMU representation.

        World frame convention (matches board markings):
        - X axis: along length (forward)
        - Y axis: along width (left)
        - Z axis: perpendicular (up)
        """
        # Add world coordinate frame at origin (Z-up convention)
        self.world_frame = self.server.scene.add_frame(
            name="/world",
            wxyz=(1.0, 0.0, 0.0, 0.0),
            position=(0.0, 0.0, 0.0),
            show_axes=True,
            axes_length=0.045,  # 4.5cm - 1.5x the IMU length
            axes_radius=0.002
        )

        # Add IMU coordinate frame (child of world)
        # Scaled to match IMU board size (3cm)
        self.imu_frame = self.server.scene.add_frame(
            name="/world/imu",
            wxyz=(1.0, 0.0, 0.0, 0.0),  # Identity quaternion (no rotation)
            position=(0.0, 0.0, 0.0),
            show_axes=True,
            axes_length=0.03,   # 3cm - 1x the IMU length (same as board)
            axes_radius=0.001   # Very thin axes
        )

        # Add a cuboid to represent the IMU sensor
        # Dimensions: (length_x, width_y, height_z)
        self.imu_length = 0.03   # X - length: 3cm
        self.imu_width = 0.025   # Y - width: 2.5cm
        self.imu_height = 0.002  # Z - height: 2mm

        self.imu_box = self.server.scene.add_box(
            name="/world/imu/sensor",
            dimensions=(self.imu_length, self.imu_width, self.imu_height),
            color=(20, 20, 20),  # Black color (slightly off-black for visibility)
            position=(0.0, 0.0, 0.0),  # Centered at IMU frame origin
        )

        # Add LED indicator (small red cuboid on top surface)
        # LED dimensions: 1mm height (Z), 1mm along X, 2mm along Y
        led_x = 0.001  # 1mm along X axis
        led_y = 0.002  # 2mm along Y axis
        led_z = 0.001  # 1mm height

        # LED offset from IMU center: X=1.3cm, Y=0.5cm
        led_offset_x = 0.013  # 1.3cm along X
        led_offset_y = 0.005  # 0.5cm along Y
        led_offset_z = self.imu_height / 2 + led_z / 2  # On top surface

        self.led_indicator = self.server.scene.add_box(
            name="/world/imu/led",
            dimensions=(led_x, led_y, led_z),
            color=(139, 0, 0),  # Dark red color
            position=(led_offset_x, led_offset_y, led_offset_z)
        )

        # Add ground plane for reference
        self.server.scene.add_grid(
            name="/ground",
            width=0.3,      # 30cm
            height=0.3,     # 30cm
            cell_size=0.05,  # 5cm grid cells
            cell_thickness=1.0,
            position=(0.0, 0.0, -0.02)  # 2cm below origin
        )

    def _setup_plots(self):
        """Setup Plotly charts for time series visualization."""
        # Create empty figures for each sensor type
        self.gyro_plot = self._create_3axis_plot(
            "Gyroscope (rad/s)",
            ["Gyro X", "Gyro Y", "Gyro Z"],
            ["red", "green", "blue"]
        )

        self.accel_plot = self._create_3axis_plot(
            "Accelerometer (m/s²)",
            ["Accel X", "Accel Y", "Accel Z"],
            ["red", "green", "blue"]
        )

        self.mag_plot = self._create_3axis_plot(
            "Magnetometer (µT)",
            ["Mag X", "Mag Y", "Mag Z"],
            ["red", "green", "blue"]
        )

        self.linear_accel_plot = self._create_3axis_plot(
            "Linear Acceleration (m/s²)",
            ["Lin Accel X", "Lin Accel Y", "Lin Accel Z"],
            ["red", "green", "blue"]
        )

        self.euler_plot = self._create_3axis_plot(
            "Orientation (degrees)",
            ["Roll", "Pitch", "Yaw"],
            ["orange", "purple", "cyan"]
        )

        # Add plots to Viser GUI
        self.gyro_plotly = self.server.gui.add_plotly(figure=self.gyro_plot)
        self.accel_plotly = self.server.gui.add_plotly(figure=self.accel_plot)
        self.mag_plotly = self.server.gui.add_plotly(figure=self.mag_plot)
        self.linear_accel_plotly = self.server.gui.add_plotly(figure=self.linear_accel_plot)
        self.euler_plotly = self.server.gui.add_plotly(figure=self.euler_plot)

    def _create_3axis_plot(self, title, labels, colors):
        """Create a Plotly figure with 3 traces (X, Y, Z axes)."""
        fig = go.Figure()

        for i, (label, color) in enumerate(zip(labels, colors)):
            fig.add_trace(go.Scatter(
                x=[],
                y=[],
                mode='lines',
                name=label,
                line=dict(color=color, width=2)
            ))

        fig.update_layout(
            title=title,
            xaxis_title="Time (s)",
            yaxis_title="Value",
            height=300,
            margin=dict(l=40, r=40, t=40, b=40),
            hovermode='x unified',
            showlegend=True
        )

        return fig

    def update(self, data, position=None):
        """
        Update visualization with new IMU data.

        Args:
            data: Dictionary from BNO080Reader.read_data() containing:
                  - timestamp, accel, gyro, mag, quaternion, euler, linear_accel
            position: Optional tuple (x, y, z) for IMU position in world frame
        """
        # Initialize start time on first update
        if self.start_time is None:
            self.start_time = data['timestamp']

        # Calculate relative time
        relative_time = data['timestamp'] - self.start_time

        # Update data buffers
        self.time_buffer.append(relative_time)

        # Gyroscope
        self.gyro_buffer['x'].append(data['gyro']['x'])
        self.gyro_buffer['y'].append(data['gyro']['y'])
        self.gyro_buffer['z'].append(data['gyro']['z'])

        # Accelerometer
        self.accel_buffer['x'].append(data['accel']['x'])
        self.accel_buffer['y'].append(data['accel']['y'])
        self.accel_buffer['z'].append(data['accel']['z'])

        # Magnetometer
        self.mag_buffer['x'].append(data['mag']['x'])
        self.mag_buffer['y'].append(data['mag']['y'])
        self.mag_buffer['z'].append(data['mag']['z'])

        # Linear Acceleration
        self.linear_accel_buffer['x'].append(data['linear_accel']['x'])
        self.linear_accel_buffer['y'].append(data['linear_accel']['y'])
        self.linear_accel_buffer['z'].append(data['linear_accel']['z'])

        # Euler angles - use transformed values if axis transformation is active
        transformed_euler = self._transform_euler(data['quaternion'])
        if transformed_euler is not None:
            # Use transformed Euler angles (in transformed coordinate frame)
            euler_to_use = transformed_euler
            # Also update the data dict so printed values match visualization
            data['euler'] = transformed_euler
        else:
            # Use original Euler angles
            euler_to_use = data['euler']

        # Store Euler angles in buffer (convert to degrees)
        self.euler_buffer['roll'].append(math.degrees(euler_to_use['roll']))
        self.euler_buffer['pitch'].append(math.degrees(euler_to_use['pitch']))
        self.euler_buffer['yaw'].append(math.degrees(euler_to_use['yaw']))

        # Update 3D orientation and position
        self._update_3d_pose(data['quaternion'], position)

        # Update plots (only every N samples to reduce overhead)
        # At 40Hz, update plots every 4 samples = 10Hz
        if len(self.time_buffer) % 4 == 0:
            self._update_plots()

    def _update_3d_pose(self, quaternion, position=None):
        """
        Update the 3D coordinate frame orientation and position.

        Args:
            quaternion: Dict with 'w', 'x', 'y', 'z' keys
            position: Optional tuple (x, y, z) for position in world frame
        """
        # Apply axis transformation if configured
        transformed_quat = self._transform_quaternion(quaternion)

        # IMPORTANT: Viser requires explicit re-assignment to trigger client updates
        # Create a NEW numpy array each time (don't reuse the same object)
        new_wxyz = np.array([
            transformed_quat['w'],
            transformed_quat['x'],
            transformed_quat['y'],
            transformed_quat['z']
        ], dtype=np.float64)

        self.imu_frame.wxyz = new_wxyz

        # Update position if provided
        if position is not None:
            # Also create new array for position to ensure sync
            new_position = np.array(position, dtype=np.float64)
            self.imu_frame.position = new_position

        # Debug: Print quaternion changes (first 5 updates only)
        if not hasattr(self, '_debug_count'):
            self._debug_count = 0
        if self._debug_count < 5:
            print(f"[DEBUG] Updated IMU orientation: w={new_wxyz[0]:.3f}, x={new_wxyz[1]:.3f}, y={new_wxyz[2]:.3f}, z={new_wxyz[3]:.3f}")
            self._debug_count += 1

    def _update_plots(self):
        """Update all time series plots with buffered data."""
        time_array = list(self.time_buffer)

        # Update Gyroscope plot
        self._update_3axis_figure(
            self.gyro_plot,
            time_array,
            [list(self.gyro_buffer['x']),
             list(self.gyro_buffer['y']),
             list(self.gyro_buffer['z'])]
        )
        self.gyro_plotly.figure = self.gyro_plot

        # Update Accelerometer plot
        self._update_3axis_figure(
            self.accel_plot,
            time_array,
            [list(self.accel_buffer['x']),
             list(self.accel_buffer['y']),
             list(self.accel_buffer['z'])]
        )
        self.accel_plotly.figure = self.accel_plot

        # Update Magnetometer plot
        self._update_3axis_figure(
            self.mag_plot,
            time_array,
            [list(self.mag_buffer['x']),
             list(self.mag_buffer['y']),
             list(self.mag_buffer['z'])]
        )
        self.mag_plotly.figure = self.mag_plot

        # Update Linear Acceleration plot
        self._update_3axis_figure(
            self.linear_accel_plot,
            time_array,
            [list(self.linear_accel_buffer['x']),
             list(self.linear_accel_buffer['y']),
             list(self.linear_accel_buffer['z'])]
        )
        self.linear_accel_plotly.figure = self.linear_accel_plot

        # Update Euler angles plot
        self._update_3axis_figure(
            self.euler_plot,
            time_array,
            [list(self.euler_buffer['roll']),
             list(self.euler_buffer['pitch']),
             list(self.euler_buffer['yaw'])]
        )
        self.euler_plotly.figure = self.euler_plot

    def _update_3axis_figure(self, fig, time_data, y_data_list):
        """Update a 3-axis Plotly figure with new data."""
        for i, y_data in enumerate(y_data_list):
            fig.data[i].x = time_data
            fig.data[i].y = y_data


if __name__ == "__main__":
    # Simple test with rotation and position animation
    print("Testing IMU Viewer...")
    print("This demo shows:")
    print("  - IMU orientation (rotating)")
    print("  - IMU position (moving in circle)")
    print("  - Red LED indicator on top surface")
    print("  - Time series plots")
    viewer = IMUViewer(port=8080)

    print("\nSimulating data updates...")
    print("Open http://localhost:8080 to see the visualization\n")

    for i in range(400):  # Run for 10 seconds at 40Hz
        t = i * 0.025  # 40Hz

        # Simulate rotation (spinning around Z axis)
        yaw = t * 1.0  # Moderate rotation (1 rad/s)
        pitch = 0.3 * np.sin(t * 0.5)  # Gentle pitch oscillation
        roll = 0.2 * np.cos(t * 0.7)   # Gentle roll oscillation

        # Convert Euler angles to quaternion
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        cos_half_yaw = cy * cp * cr + sy * sp * sr
        sin_half_x = cy * cp * sr - sy * sp * cr
        sin_half_y = sy * cp * sr + cy * sp * cr
        sin_half_z = sy * cp * cr - cy * sp * sr

        # Simulate circular motion in XY plane (scaled to IMU size)
        radius = 0.08  # 8cm radius - appropriate for small IMU
        pos_x = radius * np.cos(t * 0.3)
        pos_y = radius * np.sin(t * 0.3)
        pos_z = 0.05 + 0.02 * np.sin(t)  # Gentle up-down motion (5cm ± 2cm)

        fake_data = {
            'timestamp': time.time(),
            'accel': {'x': np.sin(t), 'y': np.cos(t), 'z': 9.8 + np.random.randn() * 0.1},
            'gyro': {'x': 0.1 * np.sin(t), 'y': 0.2 * np.cos(t), 'z': 0.5},
            'mag': {'x': 20 + np.random.randn(), 'y': 10 + np.random.randn(), 'z': 30 + np.random.randn()},
            'quaternion': {'w': cos_half_yaw, 'x': sin_half_x, 'y': sin_half_y, 'z': sin_half_z},
            'euler': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
            'linear_accel': {'x': -pos_x * 0.1, 'y': -pos_y * 0.1, 'z': 0.0}
        }

        # Update with both orientation and position
        viewer.update(fake_data, position=(pos_x, pos_y, pos_z))
        time.sleep(0.025)

        if i % 40 == 0:  # Print status every second
            print(f"  Time: {t:.1f}s - Position: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})")

    print("\n✓ Test complete! Visualization still running at http://localhost:8080")
    print("Press Enter to exit...")
    input()
