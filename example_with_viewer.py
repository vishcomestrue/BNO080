#!/usr/bin/env python3
"""
Example: BNO080 IMU with Viser Visualization

This shows how to integrate the viewer with IMU data collection.
The visualization runs in parallel without blocking data collection.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from bare import BNO080Reader
from viewer import IMUViewer
import time


def main():
    """Main loop with IMU data collection and visualization."""
    print("="*80)
    print("BNO080 IMU with Viser Visualization")
    print("="*80)

    try:
        # Initialize IMU sensor
        imu = BNO080Reader()

        # Initialize visualization (starts web server at http://localhost:8080)
        # Board markings: X=length, Y=width, Z=perpendicular (up)
        # Sensor frame: X=perpendicular (DOWN), Y=width, Z=length
        # Confirmed by quaternion analysis: rotating around board Z changes sensor qy/qz (rotation around sensor X)
        # Fix: Swap X ↔ Z + flip both axes
        viz = IMUViewer(port=8080, buffer_size=200, axis_transform='swap_xz')  # 5 seconds at 40Hz

        print("\n" + "="*80)
        print("📊 Visualization running at: http://localhost:8080")
        print("="*80)
        print("\nStarting data acquisition at 40Hz...")
        print("Press Ctrl+C to stop\n")

        # Data collection loop (40Hz)
        while True:
            # Read IMU data
            data = imu.read_data()

            # Update visualization (non-blocking, async)
            # Option 1: Update orientation only
            viz.update(data)

            # Option 2: Update both orientation and position
            # If you have position estimation (e.g., from sensor fusion):
            # position = (x, y, z)  # Position in world frame
            # viz.update(data, position=position)

            # Optional: Print to console every second
            if int(data['timestamp']) % 1 == 0:
                print(f"[{data['timestamp']:.1f}s] "
                      f"Orientation: Roll={data['euler']['roll']*57.3:.1f}° "
                      f"Pitch={data['euler']['pitch']*57.3:.1f}° "
                      f"Yaw={data['euler']['yaw']*57.3:.1f}°")
                # print(f"[{data['timestamp']:.1f}s] "
                #       f"Acceleration: X={data['accel']['x']:.2f} "
                #       f"Y={data['accel']['y']:.2f} "
                #       f"Z={data['accel']['z']:.2f} m/s²")

            # Sleep to maintain 40Hz rate
            time.sleep(1/40)

    except KeyboardInterrupt:
        print("\n\n✓ Stopped by user")
        print("Closing visualization server...")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
