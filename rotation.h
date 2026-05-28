/****************************************************************************
 *
 *   Copyright (C) 2013-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rotation.h
 * @brief Vector rotation library for sensor and board orientation handling.
 *
 * This library provides functions to apply a predefined rotation (e.g., board
 * mounting orientation) to 3D vectors. It supports integer and floating-point
 * vectors and optimizes common rotations with switch‑case logic. For complex
 * or custom rotations a full rotation matrix or quaternion can be obtained.
 *
 * The rotations are defined as Euler angles (roll, pitch, yaw) applied in the
 * order ZYX (yaw, pitch, roll). The library is primarily used to transform
 * sensor data from the board frame to the airframe frame.
 *
 * @note All angles are in degrees in the lookup table, but internally converted
 *       to radians when matrix/ quaternion functions are used.
 *
 * @see Rotation
 */

// ################################################################################

#pragma once

#include <stdint.h>

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>

/**
 * @enum Rotation
 * @brief Predefined board mounting rotations.
 *
 * Each enumeration value corresponds to a specific combination of roll, pitch,
 * and yaw rotations that map a vector from the board (sensor) frame to the
 * airframe frame. The naming convention describes the required rotation of the
 * board to align with the airframe axes.
 *
 * For example, ROTATION_YAW_90 means the board is rotated 90° around the yaw
 * axis relative to the airframe. ROTATION_ROLL_180_PITCH_90 indicates a roll
 * of 180° followed by a pitch of 90°.
 *
 * @note ROTATION_CUSTOM (value 100) is reserved for user‑defined Euler angles
 *       handled outside this enumeration.
 */
enum Rotation : uint8_t {
	ROTATION_NONE                = 0,
	ROTATION_YAW_45              = 1,
	ROTATION_YAW_90              = 2,
	ROTATION_YAW_135             = 3,
	ROTATION_YAW_180             = 4,
	ROTATION_YAW_225             = 5,
	ROTATION_YAW_270             = 6,
	ROTATION_YAW_315             = 7,
	ROTATION_ROLL_180            = 8,
	ROTATION_ROLL_180_YAW_45     = 9,
	ROTATION_ROLL_180_YAW_90     = 10,
	ROTATION_ROLL_180_YAW_135    = 11,
	ROTATION_PITCH_180           = 12,
	ROTATION_ROLL_180_YAW_225    = 13,
	ROTATION_ROLL_180_YAW_270    = 14,
	ROTATION_ROLL_180_YAW_315    = 15,
	ROTATION_ROLL_90             = 16,
	ROTATION_ROLL_90_YAW_45      = 17,
	ROTATION_ROLL_90_YAW_90      = 18,
	ROTATION_ROLL_90_YAW_135     = 19,
	ROTATION_ROLL_270            = 20,
	ROTATION_ROLL_270_YAW_45     = 21,
	ROTATION_ROLL_270_YAW_90     = 22,
	ROTATION_ROLL_270_YAW_135    = 23,
	ROTATION_PITCH_90            = 24,
	ROTATION_PITCH_270           = 25,
	ROTATION_PITCH_180_YAW_90    = 26,
	ROTATION_PITCH_180_YAW_270   = 27,
	ROTATION_ROLL_90_PITCH_90    = 28,
	ROTATION_ROLL_180_PITCH_90   = 29,
	ROTATION_ROLL_270_PITCH_90   = 30,
	ROTATION_ROLL_90_PITCH_180   = 31,
	ROTATION_ROLL_270_PITCH_180  = 32,
	ROTATION_ROLL_90_PITCH_270   = 33,
	ROTATION_ROLL_180_PITCH_270  = 34,
	ROTATION_ROLL_270_PITCH_270  = 35,
	ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
	ROTATION_ROLL_90_YAW_270          = 37,
	ROTATION_ROLL_90_PITCH_68_YAW_293 = 38,
	ROTATION_PITCH_315                = 39,
	ROTATION_ROLL_90_PITCH_315        = 40,
	ROTATION_MAX,								///< Sentinel: number of predefined rotations

	// Rotation Enum reserved for custom rotation using Euler Angles
	ROTATION_CUSTOM                  = 100		///< Custom rotation, not covered by lookup table
};

/**
 * @struct rot_lookup_t
 * @brief Lookup table entry mapping a Rotation enum to Euler angles.
 *
 * The angles are in degrees and represent the rotation order roll → pitch → yaw
 * (X‑Y‑Z). The same order is used when constructing the rotation matrix or
 * quaternion.
 */
struct rot_lookup_t {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
};

/**
 * @var rot_lookup
 * @brief Static lookup table that converts each Rotation enum to Euler angles.
 *
 * Indexed by the Rotation enum values (0 … ROTATION_MAX‑1). The table is
 * defined in the accompanying rotation.cpp file.
 */
static constexpr rot_lookup_t rot_lookup[ROTATION_MAX] = {
	{  0,   0,   0 },
	{  0,   0,  45 },
	{  0,   0,  90 },
	{  0,   0, 135 },
	{  0,   0, 180 },
	{  0,   0, 225 },
	{  0,   0, 270 },
	{  0,   0, 315 },
	{180,   0,   0 },
	{180,   0,  45 },
	{180,   0,  90 },
	{180,   0, 135 },
	{  0, 180,   0 },
	{180,   0, 225 },
	{180,   0, 270 },
	{180,   0, 315 },
	{ 90,   0,   0 },
	{ 90,   0,  45 },
	{ 90,   0,  90 },
	{ 90,   0, 135 },
	{270,   0,   0 },
	{270,   0,  45 },
	{270,   0,  90 },
	{270,   0, 135 },
	{  0,  90,   0 },
	{  0, 270,   0 },
	{  0, 180,  90 },
	{  0, 180, 270 },
	{ 90,  90,   0 },
	{180,  90,   0 },
	{270,  90,   0 },
	{ 90, 180,   0 },
	{270, 180,   0 },
	{ 90, 270,   0 },
	{180, 270,   0 },
	{270, 270,   0 },
	{ 90, 180,  90 },
	{ 90,   0, 270 },
	{ 90,  68, 293 },
	{  0, 315,   0 },
	{ 90, 315,   0 },
};

/**
 * @fn get_rot_matrix
 * @brief Get the rotation matrix (direction cosine matrix) for a given rotation.
 *
 * The matrix is constructed from the Euler angles stored in the lookup table
 * using the ZYX (yaw, pitch, roll) order.
 *
 * @param rot The predefined rotation identifier.
 * @return matrix::Dcmf 3×3 rotation matrix.
 */
__EXPORT matrix::Dcmf get_rot_matrix(enum Rotation rot);

/**
 * @fn get_rot_quaternion
 * @brief Get the rotation quaternion for a given rotation.
 *
 * The quaternion is derived from the same Euler angles as get_rot_matrix().
 *
 * @param rot The predefined rotation identifier.
 * @return matrix::Quatf Unit quaternion representing the rotation.
 */
__EXPORT matrix::Quatf get_rot_quaternion(enum Rotation rot);

/**
 * @fn rotate_3i
 * @brief Apply a rotation to a 3‑element integer vector (in‑place).
 *
 * The function first attempts an optimised switch‑case for common rotations.
 * If the rotation is not handled there, it falls back to the full matrix
 * multiplication and clamps the result to the range of int16_t.
 *
 * @param rot The rotation to apply.
 * @param x Reference to the X component (modified).
 * @param y Reference to the Y component (modified).
 * @param z Reference to the Z component (modified).
 */
__EXPORT void rotate_3i(enum Rotation rot, int16_t &x, int16_t &y, int16_t &z);

/**
 * @fn rotate_3f
 * @brief Apply a rotation to a 3‑element float vector (in‑place).
 *
 * The function first attempts an optimised switch‑case for common rotations.
 * If the rotation is not handled there, it falls back to the full matrix
 * multiplication.
 *
 * @param rot The rotation to apply.
 * @param x Reference to the X component (modified).
 * @param y Reference to the Y component (modified).
 * @param z Reference to the Z component (modified).
 */
__EXPORT void rotate_3f(enum Rotation rot, float &x, float &y, float &z);

/**
 * @fn rotate_3
 * @brief Optimised in‑place rotation for common cases (template).
 *
 * This template function is called internally by rotate_3i and rotate_3f.
 * It returns true if the rotation was handled by the optimised switch‑case,
 * false otherwise (fallback to matrix multiplication). The fallback is
 * implemented in the caller functions.
 *
 * @tparam T Numeric type (int16_t or float).
 * @param rot Rotation to apply.
 * @param x Reference to X component.
 * @param y Reference to Y component.
 * @param z Reference to Z component.
 * @return true if the rotation was handled by the switch‑case, false otherwise.
 */
template<typename T>
static bool rotate_3(enum Rotation rot, T &x, T &y, T &z)
{
	switch (rot) {
	case ROTATION_NONE:
		return true;

	case ROTATION_YAW_90: {
			T tmp = x;
			x = math::negate(y);
			y = tmp;
		}

		return true;

	case ROTATION_YAW_180: {
			x = math::negate(x);
			y = math::negate(y);
		}

		return true;

	case ROTATION_YAW_270: {
			T tmp = x;
			x = y;
			y = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_180: {
			y = math::negate(y);
			z = math::negate(z);
		}

		return true;

	case ROTATION_ROLL_180_YAW_90:

	// FALLTHROUGH
	case ROTATION_PITCH_180_YAW_270: {
			T tmp = x;
			x = y;
			y = tmp;
			z = math::negate(z);
		}

		return true;

	case ROTATION_PITCH_180: {
			x = math::negate(x);
			z = math::negate(z);
		}

		return true;

	case ROTATION_ROLL_180_YAW_270:

	// FALLTHROUGH
	case ROTATION_PITCH_180_YAW_90: {
			T tmp = x;
			x = math::negate(y);
			y = math::negate(tmp);
			z = math::negate(z);
		}

		return true;

	case ROTATION_ROLL_90: {
			T tmp = z;
			z = y;
			y = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_90_YAW_90: {
			T tmp = x;
			x = z;
			z = y;
			y = tmp;
		}

		return true;

	case ROTATION_ROLL_270: {
			T tmp = z;
			z = math::negate(y);
			y = tmp;
		}

		return true;

	case ROTATION_ROLL_270_YAW_90: {
			T tmp = x;
			x = math::negate(z);
			z = math::negate(y);
			y = tmp;
		}

		return true;

	case ROTATION_PITCH_90: {
			T tmp = z;
			z = math::negate(x);
			x = tmp;
		}

		return true;

	case ROTATION_PITCH_270: {
			T tmp = z;
			z = x;
			x = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_180_PITCH_270: {
			T tmp = z;
			z = x;
			x = tmp;
			y = math::negate(y);
		}

		return true;

	case ROTATION_ROLL_90_YAW_270: {
			T tmp = x;
			x = math::negate(z);
			z = y;
			y = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_90_PITCH_90: {
			T tmp = x;
			x = y;
			y = math::negate(z);
			z = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_180_PITCH_90: {
			T tmp = x;
			x = math::negate(z);
			y = math::negate(y);
			z = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_270_PITCH_90: {
			T tmp = x;
			x = math::negate(y);
			y = z;
			z = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_90_PITCH_180: {
			T tmp = y;
			x = math::negate(x);
			y = math::negate(z);
			z = math::negate(tmp);
		}

		return true;

	case ROTATION_ROLL_270_PITCH_180: {
			T tmp = y;
			x = math::negate(x);
			y = z;
			z = tmp;
		}

		return true;

	case ROTATION_ROLL_90_PITCH_270: {
			T tmp = x;
			x = math::negate(y);
			y = math::negate(z);
			z = tmp;
		}

		return true;

	case ROTATION_ROLL_270_PITCH_270: {
			T tmp = x;
			x = y;
			y = z;
			z = tmp;
		}

		return true;

	case ROTATION_ROLL_90_PITCH_180_YAW_90: {
			T tmp = x;
			x = z;
			z = math::negate(y);
			y = math::negate(tmp);
		}

		return true;

	default:
		break;
	}

	return false;
}
