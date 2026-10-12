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
 * @file rotation.cpp
 *
 * Vector rotation library
 */

#include "rotation.h"

namespace
{
bool is_predefined_rotation(Rotation rotation)
{
	return static_cast<unsigned int>(rotation) < static_cast<unsigned int>(ROTATION_MAX);
}
}

PX4_CONVERSION_EXPORT matrix::Dcmf
get_rot_matrix(enum Rotation rot)
{
	if (!is_predefined_rotation(rot)) {
		return matrix::Dcmf{matrix::Eulerf{0.f, 0.f, 0.f}};
	}

	return matrix::Dcmf{matrix::Eulerf{
			math::radians((float)rot_lookup[rot].roll),
			math::radians((float)rot_lookup[rot].pitch),
			math::radians((float)rot_lookup[rot].yaw)}};
}

PX4_CONVERSION_EXPORT matrix::Quatf
get_rot_quaternion(enum Rotation rot)
{
	if (!is_predefined_rotation(rot)) {
		return matrix::Quatf{matrix::Eulerf{0.f, 0.f, 0.f}};
	}

	return matrix::Quatf{matrix::Eulerf{
			math::radians((float)rot_lookup[rot].roll),
			math::radians((float)rot_lookup[rot].pitch),
			math::radians((float)rot_lookup[rot].yaw)}};
}

PX4_CONVERSION_EXPORT void
rotate_3i(enum Rotation rot, int16_t &x, int16_t &y, int16_t &z)
{
	if (!rotate_3(rot, x, y, z)) {
		// otherwise use full rotation matrix for valid rotations
		if (is_predefined_rotation(rot)) {
			const matrix::Vector3f r{get_rot_matrix(rot) *matrix::Vector3f{(float)x, (float)y, (float)z}};
			x = math::constrain(roundf(r(0)), (float)INT16_MIN, (float)INT16_MAX);
			y = math::constrain(roundf(r(1)), (float)INT16_MIN, (float)INT16_MAX);
			z = math::constrain(roundf(r(2)), (float)INT16_MIN, (float)INT16_MAX);
		}
	}
}

PX4_CONVERSION_EXPORT void
rotate_3f(enum Rotation rot, float &x, float &y, float &z)
{
	if (!rotate_3(rot, x, y, z)) {
		// otherwise use full rotation matrix for valid rotations
		if (is_predefined_rotation(rot)) {
			const matrix::Vector3f r{get_rot_matrix(rot) *matrix::Vector3f{x, y, z}};
			x = r(0);
			y = r(1);
			z = r(2);
		}
	}
}
