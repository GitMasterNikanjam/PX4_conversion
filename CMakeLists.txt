cmake_minimum_required(VERSION 3.16)

project(PX4_conversion LANGUAGES CXX)

set(PX4_CONVERSION_MATHLIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../PX4_mathlib" CACHE PATH
	"Path to the extracted PX4_mathlib repository")
set(PX4_CONVERSION_MATRIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../PX4_matrix" CACHE PATH
	"Path to the extracted PX4_matrix repository")

if(NOT EXISTS "${PX4_CONVERSION_MATHLIB_DIR}/mathlib.h")
	message(FATAL_ERROR
		"PX4_mathlib was not found at '${PX4_CONVERSION_MATHLIB_DIR}'. "
		"Set PX4_CONVERSION_MATHLIB_DIR to the extracted library path.")
endif()

if(NOT EXISTS "${PX4_CONVERSION_MATRIX_DIR}/matrix/math.hpp")
	message(FATAL_ERROR
		"PX4_matrix was not found at '${PX4_CONVERSION_MATRIX_DIR}'. "
		"Set PX4_CONVERSION_MATRIX_DIR to the extracted library path.")
endif()

add_library(px4_conversion rotation.cpp)
add_library(PX4::conversion ALIAS px4_conversion)

target_compile_features(px4_conversion PUBLIC cxx_std_17)

target_include_directories(px4_conversion
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
		$<BUILD_INTERFACE:${PX4_CONVERSION_MATHLIB_DIR}>
		$<BUILD_INTERFACE:${PX4_CONVERSION_MATRIX_DIR}>
)
