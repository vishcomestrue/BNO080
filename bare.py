#!/usr/bin/env python3
"""
Simple BNO080 IMU Reader for Raspberry Pi

Reads and displays 5 main sensor outputs:
1. Accelerometer (m/s²)
2. Gyroscope (rad/s)
3. Magnetometer (µT)
4. Orientation (quaternion and Euler angles)
5. Linear Acceleration (gravity removed)

I2C Address: 0x4b (default)
"""

import time
import math
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)


class BNO080Reader:
    """Simple reader for BNO080 9-axis IMU."""

    def __init__(self, i2c_address=0x4b):
        """
        Initialize BNO080 sensor.

        Args:
            i2c_address: I2C address (default 0x4b)
        """
        print(f"Initializing BNO080 at I2C address 0x{i2c_address:02x}...")
        print("Auto-calibration running in background")

        try:
            # Initialize I2C bus
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            self.bno = BNO08X_I2C(self.i2c, address=i2c_address)

            # Enable all 5 main sensor reports (uses default report interval)
            print("Enabling sensor reports...")

            print("  - Accelerometer")
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            time.sleep(0.05)

            print("  - Gyroscope")
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            time.sleep(0.05)

            print("  - Magnetometer")
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            time.sleep(0.05)

            print("  - Rotation Vector (orientation)")
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.05)

            print("  - Linear Acceleration")
            self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
            time.sleep(0.05)

            print("BNO080 initialized successfully!\n")

        except Exception as e:
            print(f"Failed to initialize BNO080: {e}")
            raise

    @staticmethod
    def quaternion_to_euler(qw, qx, qy, qz):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def read_data(self):
        """Read all sensor data from BNO080."""
        data = {
            'timestamp': time.time(),
            'accel': {'x': 0, 'y': 0, 'z': 0},
            'gyro': {'x': 0, 'y': 0, 'z': 0},
            'mag': {'x': 0, 'y': 0, 'z': 0},
            'quaternion': {'w': 1, 'x': 0, 'y': 0, 'z': 0},
            'euler': {'roll': 0, 'pitch': 0, 'yaw': 0},
            'linear_accel': {'x': 0, 'y': 0, 'z': 0},
        }

        try:
            # Read accelerometer (m/s²)
            if self.bno.acceleration is not None:
                ax, ay, az = self.bno.acceleration
                data['accel'] = {'x': ax, 'y': ay, 'z': az}

            # Read gyroscope (rad/s)
            if self.bno.gyro is not None:
                gx, gy, gz = self.bno.gyro
                data['gyro'] = {'x': gx, 'y': gy, 'z': gz}

            # Read magnetometer (µT)
            if self.bno.magnetic is not None:
                mx, my, mz = self.bno.magnetic
                data['mag'] = {'x': mx, 'y': my, 'z': mz}

            # Read quaternion (orientation)
            if self.bno.quaternion is not None:
                qw, qx, qy, qz = self.bno.quaternion
                data['quaternion'] = {'w': qw, 'x': qx, 'y': qy, 'z': qz}
                roll, pitch, yaw = self.quaternion_to_euler(qw, qx, qy, qz)
                data['euler'] = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

            # Read linear acceleration (gravity removed)
            if self.bno.linear_acceleration is not None:
                lx, ly, lz = self.bno.linear_acceleration
                data['linear_accel'] = {'x': lx, 'y': ly, 'z': lz}

        except Exception as e:
            print(f"Error reading BNO080 data: {e}")

        return data

    def print_data(self, data):
        """Print all 5 sensor outputs."""
        print(f"\n{'='*80}")
        print(f"BNO080 Sensor Data (t={data['timestamp']:.3f}s)")
        print(f"{'='*80}")

        print(f"\n1. Accelerometer (m/s²):")
        print(f"   X: {data['accel']['x']:8.4f}  Y: {data['accel']['y']:8.4f}  Z: {data['accel']['z']:8.4f}")

        print(f"\n2. Gyroscope (rad/s):")
        print(f"   X: {data['gyro']['x']:8.4f}  Y: {data['gyro']['y']:8.4f}  Z: {data['gyro']['z']:8.4f}")

        print(f"\n3. Magnetometer (µT):")
        print(f"   X: {data['mag']['x']:8.4f}  Y: {data['mag']['y']:8.4f}  Z: {data['mag']['z']:8.4f}")

        print(f"\n4. Orientation:")
        print(f"   Quaternion - W: {data['quaternion']['w']:8.4f}  X: {data['quaternion']['x']:8.4f}")
        print(f"                Y: {data['quaternion']['y']:8.4f}  Z: {data['quaternion']['z']:8.4f}")
        print(f"   Euler      - Roll:  {math.degrees(data['euler']['roll']):7.2f}°  "
              f"Pitch: {math.degrees(data['euler']['pitch']):7.2f}°  "
              f"Yaw: {math.degrees(data['euler']['yaw']):7.2f}°")

        print(f"\n5. Linear Acceleration (gravity removed, m/s²):")
        print(f"   X: {data['linear_accel']['x']:8.4f}  Y: {data['linear_accel']['y']:8.4f}  Z: {data['linear_accel']['z']:8.4f}")


def main():
    """Main loop to continuously read and display BNO080 data."""
    print("="*80)
    print("Simple BNO080 IMU Reader")
    print("="*80)

    try:
        # Initialize sensor (uses default 20Hz sample rate)
        imu = BNO080Reader()

        print("Starting data acquisition...")
        print("Press Ctrl+C to stop\n")

        # Read loop
        while True:
            # Read and print data
            data = imu.read_data()
            imu.print_data(data)

            # Sleep between reads
            time.sleep(float(1/40))  # 40Hz

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
