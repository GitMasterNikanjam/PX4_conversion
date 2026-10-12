# Vector Rotation Library

This library provides efficient 3D vector rotation functions for sensor and board orientation handling. It maps a predefined board mounting rotation (e.g., from an IMU or compass) to the airframe coordinate system and can be used outside PX4.

## Features

- Predefined Euler‑angle rotations (roll, pitch, yaw) for common mounting orientations.
- Optimised in‑place rotation for many frequently used cases using integer arithmetic (no floating‑point).
- Fallback to full matrix multiplication for complex or less common rotations.
- Support for both `float` and `int16_t` vectors.
- Utility to obtain the full rotation matrix or quaternion for custom processing.

## Coordinate Frame Convention

- Rotations use the matrix library's Euler convention: roll, pitch, yaw angles are passed to `Eulerf`, which constructs the corresponding ZYX rotation.
- The library transforms a vector from the **board (sensor) frame** to the **airframe frame**.
- Euler angles in the lookup table are stored in **degrees** (roll, pitch, yaw).

## API Overview

### Rotation Enumeration (`enum Rotation`)

The `Rotation` enum lists all predefined mounting orientations. Examples:

- `ROTATION_NONE` – identity.
- `ROTATION_YAW_90` – 90° rotation around yaw (Z) axis.
- `ROTATION_ROLL_180_PITCH_90` – 180° roll, then 90° pitch.
- `ROTATION_CUSTOM = 100` – reserved for user‑defined rotations.

See `rotation.h` for the complete list.

### Core Functions

#### `rotate_3f()`
```c
void rotate_3f(enum Rotation rot, float &x, float &y, float &z);
```
Applies a rotation to a 3‑element float vector **in‑place**. Uses optimised switch‑case when available, otherwise falls back to a full matrix multiplication.

#### `rotate_3i()`
```c
void rotate_3i(enum Rotation rot, int16_t &x, int16_t &y, int16_t &z);
```
Same as `rotate_3f()` but for `int16_t` vectors. The result is clamped to the range of `int16_t`.

#### `get_rot_matrix()`
```c
matrix::Dcmf get_rot_matrix(enum Rotation rot);
```
Returns the 3×3 direction cosine matrix (rotation matrix) for the given rotation.

#### `get_rot_quaternion()`
```c
matrix::Quatf get_rot_quaternion(enum Rotation rot);
```
Returns the unit quaternion representing the rotation.

## Usage Example

```c
#include "rotation.h"

// Example: rotate a magnetometer reading from board frame to airframe frame
int16_t mag_x = 150, mag_y = 30, mag_z = -200;
enum Rotation board_orientation = ROTATION_YAW_180;

rotate_3i(board_orientation, mag_x, mag_y, mag_z);
// mag_x, mag_y, mag_z now represent the vector in airframe coordinates
```

For floating‑point vectors (e.g., accelerometer in m/s²):

```c
float accel[3] = {9.81, 0.0, 0.0};
rotate_3f(ROTATION_ROLL_90, accel[0], accel[1], accel[2]);
```

## Optimised vs. Fallback Path

- The library first tries a **switch‑case** that directly performs sign changes and swaps – these are very fast and involve no floating‑point math.
- If the requested rotation is not among the optimised cases, the function falls back to a full matrix multiplication using `get_rot_matrix()`. This is slower but still accurate.
- The fallback is only used for rotations that are **not** explicitly handled in the `rotate_3()` template switch.

## Lookup Table

The `rot_lookup` table (defined in `rotation.cpp`) maps each `Rotation` enum to integer roll, pitch, and yaw angles in degrees. This table is used by `get_rot_matrix()` and `get_rot_quaternion()`.

## Dependencies and Notes

- All functions are reentrant and thread‑safe.
- The implementation depends on the extracted `PX4_mathlib` and `PX4_matrix` headers, but no longer requires PX4 platform headers.
- For custom rotations not listed in the enum, you must manually construct a rotation matrix or quaternion using `ROTATION_CUSTOM` and your own Euler angles.
- Invalid values (including `ROTATION_CUSTOM`, which has no stored angles) produce an identity matrix/quaternion; vector rotation leaves the input unchanged.

## Standalone CMake Build

The CMake target expects `PX4_conversion`, `PX4_mathlib`, and `PX4_matrix` to be sibling directories by default:

```powershell
cmake -S PX4_conversion -B build/PX4_conversion
cmake --build build/PX4_conversion
```

If the dependency repositories are elsewhere, pass their locations with `-DPX4_CONVERSION_MATHLIB_DIR=...` and `-DPX4_CONVERSION_MATRIX_DIR=...` when configuring. Consumers can link the target as `PX4::conversion`.

## License

This library is released under the BSD 3‑Clause License (see copyright header in source files).
```
