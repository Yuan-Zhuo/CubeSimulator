#pragma once

#ifndef _PHYSICSFORMULA_HPP_

#define _PHYSICSFORMULA_HPP_

#include "Eigen/Eigen/Dense"
// #include "Eigen/Dense"
#include <limits>

#include "PhysicsState.hpp"

using namespace std;
using namespace Eigen;

// declaration

inline void cal_cube_inertia(float mass,
                             Vector3f& box_extent,
                             Matrix3f* inertia);

inline void cal_actor_location(Vector3f& relative_vec,
                               Vector3f& origin,
                               Quaternionf& quat,
                               Vector3f* location);

inline void cal_gravity(DynamicState* dynamic_state, float delta_time);

inline void cal_actor_state(ActorState* actor_state,
                            DynamicState* dynamic_state,
                            float delta_time);

inline void update_dynamic(Vector3f impulse,
                           Vector3f angular_reciprocal_inertia,
                           float mass,
                           DynamicState* dynamic_state);

inline void cal_point_collision(ActorProperty* actor_property,
                                ActorState* actor_state,
                                ImpactPoint* impact_point,
                                float delta_time,
                                DynamicState* dynamic_state);

// definition
inline float cal_1d_inertia(float a, float b, float mass) {
    return 1.f / 12.f * mass * (pow(a, 2.f), pow(b, 2.f));
}

inline void cal_cube_inertia(float mass,
                             Vector3f& box_extent,
                             Matrix3f* inertia) {
    if (!inertia)
        return;
    *inertia << cal_1d_inertia(box_extent(1) * 2, box_extent(2) * 2, mass), 0,
        0, 0, cal_1d_inertia(box_extent(0) * 2, box_extent(2) * 2, mass), 0, 0,
        0, cal_1d_inertia(box_extent(0) * 2, box_extent(1) * 2, mass);
}

inline void cal_actor_location(ActorState* actor_state, Vector3f* location) {
    if (!location || !actor_state)
        return;

    *location = (actor_state->origin) +
                (actor_state->quat) * (actor_state->relative_vec);
}

inline void cal_point_location(ActorState* actor_state,
                               Vector3f relative_vec,
                               Vector3f* location) {
    if (!location || !actor_state)
        return;

    *location = (actor_state->origin) + (actor_state->quat) * relative_vec;
}

inline void cal_gravity(DynamicState* dynamic_state, float delta_time) {
    dynamic_state->linear_vel += (Acc_Gravity * delta_time);
}

inline void update_dynamic(Vector3f impulse,
                           Vector3f angular_reciprocal_inertia,
                           float mass,
                           DynamicState* dynamic_state) {
    dynamic_state->linear_vel += (impulse / mass);
    dynamic_state->angular_vel += (impulse.norm() * angular_reciprocal_inertia);
}

inline void set_quat(float angle, Vector3f axis, Quaternionf* quat) {
    const float a = angle * 0.5f;
    const float s = (float)sin(a);
    const float c = (float)cos(a);
    quat->w() = c;
    quat->x() = axis.x() * s;
    quat->y() = axis.y() * s;
    quat->z() = axis.z() * s;
}

inline void cal_actor_state(ActorState* actor_state,
                            DynamicState* dynamic_state,

                            float delta_time) {
    // move
    actor_state->origin += dynamic_state->linear_vel * delta_time;
    // rotate
    float angular_vel_norm = dynamic_state->angular_vel.norm();
    float angular_delta_angle = angular_vel_norm * delta_time;
    Vector3f angular_vel_axis =
        angular_delta_angle > MIN_DELTA_ANGLE
            ? dynamic_state->angular_vel / angular_vel_norm
            : normalized_Z;
    Quaternionf delta_quat;
    set_quat(angular_delta_angle, angular_vel_axis, &delta_quat);
    actor_state->quat = delta_quat * actor_state->quat;
}

inline void cal_deepest_point(ActorProperty* actor_property,
                              ActorState* actor_state,
                              Vector3f* hit_point) {
    int min_z = numeric_limits<int>::max();
    Vector3f relative_vec;
    Vector3f location;

    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
                relative_vec.x() = pow(-1, x) * actor_property->box_extent(0);
                relative_vec.y() = pow(-1, y) * actor_property->box_extent(1);
                relative_vec.z() = pow(-1, z) * actor_property->box_extent(2);
                cal_point_location(actor_state, relative_vec, &location);
                if (location.z() < min_z) {
                    *hit_point = location;
                    min_z = location.z();
                }
            }
        }
    }
}

inline void cal_point_collision(ActorProperty* actor_property,
                                ActorState* actor_state,
                                ImpactPoint* impact_point,
                                float delta_time,
                                DynamicState* dynamic_state) {
    // init
    Vector3f normal = normalized_Z;
    Vector3f* hit_point = &impact_point->hit_point;
    Vector3f* origin = &actor_state->origin;
    Vector3f relative_location = *hit_point - *origin;

    Vector3f combine_velocity =
        dynamic_state->linear_vel +
        dynamic_state->angular_vel.cross(relative_location);

    // vertical
    float depth = 0.f;
    if (impact_point->depth > MAX_DEPTH)
        depth = DEPTH_COEFFICIENT / delta_time *
                max(0.0f, impact_point->depth - MAX_DEPTH);

    float vertical_impulse_numerator =
        -((1.f + actor_property->impulse_coefficient) *
              (combine_velocity).dot(normal) -
          depth);

    Vector3f vertical_angular_reciprocal_inertia =
        actor_property->inertia.inverse() * (relative_location.cross(normal));

    float vertical_impulse_denominator =
        (1.f / actor_property->mass) +
        (vertical_angular_reciprocal_inertia.cross(relative_location))
            .dot(normal);

    float vertical_impulse_val =
        vertical_impulse_numerator / vertical_impulse_denominator;

    // adjust impulse
    float total_impulse = impact_point->impluse_vertical.norm();
    float adjust_impulse = max(total_impulse + vertical_impulse_val, 0.f);
    vertical_impulse_val = adjust_impulse - total_impulse;

    update_dynamic(vertical_impulse_val * normal,
                   vertical_angular_reciprocal_inertia, actor_property->mass,
                   dynamic_state);

    // horizontal
    Vector3f horizontal_velocity =
        -combine_velocity - ((-combine_velocity).dot(normal) * normal);

    Vector3f tangent = horizontal_velocity.normalized();
    Vector3f horizontal_angular_reciprocal_inertia =
        actor_property->inertia.inverse() * (relative_location.cross(tangent));

    float horizontal_impulse_numerator = -1.f * horizontal_velocity.norm();

    float horizontal_impulse_denominator =
        1 / actor_property->mass +
        tangent.dot(
            horizontal_angular_reciprocal_inertia.cross(relative_location));
    // impulse adjust
    Vector3f horizontal_impulse =
        actor_property->linear_friction_coefficient *
        (horizontal_impulse_numerator / horizontal_impulse_denominator) *
        tangent;

    update_dynamic(horizontal_impulse, horizontal_angular_reciprocal_inertia,
                   actor_property->mass, dynamic_state);

    // feedback
    impact_point->set_cache(adjust_impulse * normal, horizontal_impulse,
                            vertical_angular_reciprocal_inertia,
                            horizontal_angular_reciprocal_inertia);
}

#endif
