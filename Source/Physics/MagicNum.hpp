#pragma once

#ifndef _MAGICNUM_HPP_

#define _MAGICNUM_HPP_

#define MAX_DEPTH 0.001f
#define MAX_DISTANCE 50.f
#define MIN_DISTANCE 5.f
#define MIN_DELTA_ANGLE 0.001f
#define MIN_FLOAT 1e-06

// property
#define MASS 1.f
#define LINEAR_FRICTION_COEFFICIENT 0.2f
#define IMPULSE_COEFFICIENT 0.5f
#define DEPTH_COEFFICIENT 0.5f

// right hand coordinator
#define normalized_X Eigen::Vector3f(1, 0, 0)
#define normalized_Y Eigen::Vector3f(0, 1, 0)
#define normalized_Z Eigen::Vector3f(0, 0, 1)
#define Gravity 9.80f
#define Acc_Gravity (-Gravity * normalized_Z * 100)

// impact point set
#define POINT_NUM 4

// iterations
#define ITERATION_NUM 10

#define LOG(msg) std::cerr << "(" << __LINE__ << "): " << msg << std::endl

#endif
