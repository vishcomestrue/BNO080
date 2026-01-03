# IMU Viewer Updates

## Recent Updates

### Fix: Coordinate Frame Mismatch (Critical Fix)
- ✅ **Fixed BNO080 axis mapping** - rotations now match physical movements
- ✅ Measured behavior: Physical Z → Visual X (red), Physical X → Visual Z (blue)
- ✅ Solution: Apply `swap_xz` transformation to remap axes correctly
- ✅ Now: Physical Z → Visual Z (blue), Physical X → Visual X (red) ✓
- ✅ Uses scipy rotation matrices to transform quaternions
- ✅ Configurable via `axis_transform` parameter

### Fix: IMU Rotation Not Updating (Critical Fix)
- ✅ **Fixed Viser synchronization issue** - cuboid now rotates properly
- ✅ Create NEW numpy arrays for wxyz and position on each update
- ✅ Required for viser >= 0.2.10 (see [Issue #318](https://github.com/nerfstudio-project/viser/issues/318))
- ✅ Added debug output (first 5 updates) to verify quaternion changes

## Changes Made

### 1. World Frame Convention (Z-Up) with Proper Scaling
- ✅ Added explicit world coordinate frame
- ✅ X axis: Length direction (forward) - Red
- ✅ Y axis: Width direction (left) - Green
- ✅ Z axis: Height direction (up) - Blue
- ✅ Right-handed coordinate system
- ✅ **Scaled axes**: 4.5cm = 1.5× IMU length (3cm)
- ✅ **Thin axes**: 2mm radius (doesn't obscure small IMU)

### 2. IMU Cuboid Specifications (Actual BNO080 Size)
- ✅ Dimensions: 3cm (X) × 2.5cm (Y) × 2mm (Z) - matches actual BNO080 board
- ✅ Color: Black (20, 20, 20) - clean, professional look
- ✅ Now positioned as child of `/world/imu` frame
- ✅ Proper orientation in Z-up world frame
- ✅ **IMU axes**: 3cm length = 1× IMU length (same as board)
- ✅ **Thin axes**: 1mm radius (very thin, doesn't obscure)

### 3. LED Indicator Specifications
- ✅ Dimensions: 1mm (X) × 2mm (Y) × 1mm (Z)
- ✅ Color: Dark red (139, 0, 0) - stands out against black board
- ✅ Located on top surface of IMU
- ✅ Offset from IMU center: X=1.3cm, Y=0.5cm
- ✅ Helps visualize IMU orientation

### 4. Position Updates
- ✅ Added `position` parameter to `update()` method
- ✅ Can now update both orientation and position
- ✅ Optional parameter - backward compatible

### 5. Properly Scaled Scene
- ✅ **Ground plane**: 30cm × 30cm (was 2m × 2m)
- ✅ **Grid cells**: 5cm × 5cm (appropriate for scale)
- ✅ **All elements proportional**: Everything scaled to match 3cm IMU

### 6. Enhanced Test/Demo
- ✅ Demo shows rotation (spinning around Z)
- ✅ Demo shows position (circular motion with 8cm radius)
- ✅ More realistic simulation with noise
- ✅ Visual feedback in terminal
- ✅ **Scaled motion**: Appropriate for small IMU size

## API Changes

### Before
```python
viz.update(data)
```

### After
```python
# Orientation only (backward compatible)
viz.update(data)

# Orientation + Position
viz.update(data, position=(x, y, z))
```

## Visual Layout

```
Scene Hierarchy:
/world                    ← World frame (Z-up)
├── /world/imu           ← IMU frame (moving/rotating)
    ├── /world/imu/sensor   ← Blue cuboid (IMU body)
    └── /world/imu/led      ← Red cuboid (LED indicator)
/ground                  ← Grid plane (reference)
```

## Testing

Run the standalone test:
```bash
python3 viewer.py
```

This will show:
- IMU rotating around Z axis
- IMU moving in circular path
- LED indicator visible on top corner
- All 5 sensor time series plots

## Files Updated

1. `viewer.py` - Main visualization module
   - Added world frame setup
   - Updated IMU dimensions (length > width)
   - Added LED indicator
   - Added position update capability

2. `example_with_viewer.py` - Integration example
   - Added comments showing position update usage

3. `README.md` - Documentation
   - Added world frame convention details
   - Added LED indicator documentation
   - Added visual diagrams (top/side views)
   - Updated API reference

4. `CHANGELOG.md` - This file

## Coordinate System Reference

```
World Frame (Right-handed, Z-up):
      Z
      ↑
      │
      │
      └────→ Y
     ╱
    ╱
   X

IMU Orientation:
- Roll: Rotation around X axis
- Pitch: Rotation around Y axis
- Yaw: Rotation around Z axis
```

## Exact Dimensions

### IMU Board (Black)
```python
Length (X): 0.03m   # 3cm
Width (Y):  0.025m  # 2.5cm
Height (Z): 0.002m  # 2mm
Color: (20, 20, 20)  # Black (slightly off-black for visibility)
```

### LED Indicator (Dark Red)
```python
Size X: 0.001m      # 1mm
Size Y: 0.002m      # 2mm
Size Z: 0.001m      # 1mm (height)
Color: (139, 0, 0)  # Dark red

# Position offset from IMU center:
Offset X: 0.013m    # 1.3cm
Offset Y: 0.005m    # 0.5cm
Offset Z: imu_height/2 + led_z/2  # On top surface
```

This places the LED at:
- X: 1.3cm from center (towards +X)
- Y: 0.5cm from center (towards +Y)
- Z: 1.5mm above IMU center (on top surface)
