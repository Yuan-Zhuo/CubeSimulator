#pragma once

#ifndef _PHYSICSSIM_HPP_

#define _PHYSICSSIM_HPP_

#include <iostream>

// #include "Eigen/Dense"
#include "Eigen/Eigen/Dense"
#include "MagicNum.hpp"
#include "PhysicsFormula.hpp"
#include "PhysicsState.hpp"

using namespace std;
using namespace Eigen;

class CubePhysics {
   private:
    // property
    ActorProperty* actor_property;

    // status
    ActorState* actor_state;
    DynamicState* dynamic_state;

    // impact_point
    ImpactPointSet impact_point_set;

    // delta_time
    float delta_time;

    // file
   public:
    CubePhysics() = delete;

    CubePhysics(float mass_,
                Vector3f box_extent_,
                Vector3f origin_,
                Vector3f location_,
                Quaternionf quat_) {
        // actor_property
        Matrix3f inertia;
        cal_cube_inertia(mass_, box_extent_, &inertia);
        actor_property =
            new ActorProperty(mass_, box_extent_, LINEAR_FRICTION_COEFFICIENT,
                              IMPULSE_COEFFICIENT, inertia);

        // status
        actor_state = new ActorState(origin_, location_ - origin_, quat_);
        dynamic_state = new DynamicState();

        delta_time = 0.f;
    }

    Vector3f get_box_extent() { return actor_property->box_extent; }

    Vector3f get_origin() { return actor_state->origin; }

    Vector3f get_location() {
        Vector3f location;
        cal_actor_location(actor_state, &location);
        return location;
    }

    void set_delta_time(float delta_time_) { delta_time = delta_time_; }

    Quaternionf get_quat() { return actor_state->quat; }

    void simulate_actor_state_origin(Vector3f* origin) {
        *origin = actor_state->origin + dynamic_state->linear_vel * delta_time;
    }

    void update_actor_state() {
        cal_actor_state(actor_state, dynamic_state, delta_time);
    }

    void update_hit_point_set() {
        impact_point_set.update_point(actor_state->origin);
    }

    void resolve_gravity() { cal_gravity(dynamic_state, delta_time); }

    void resolve_hit_point() {
        Vector3f hit_point;
        cal_deepest_point(actor_property, actor_state, &hit_point);
        impact_point_set.apply_point(hit_point, actor_state->origin);
    }

    void resolve_hit_point(Vector3f hit_point) {
        impact_point_set.apply_point(hit_point, actor_state->origin);
    }

    void warm_start() {
        for (int i = 0; i < ITERATION_NUM; i++) {
            for (uint32_t j = 0; j < POINT_NUM; j++) {
                ImpactPoint* point = impact_point_set.get_point(j);
                if (point && (point->cache)) {
                    update_dynamic(point->impluse_vertical,
                                   point->angular_inertia_vertical,
                                   actor_property->mass, dynamic_state);
                    update_dynamic(point->impluse_horizontal,
                                   point->angular_inertia_horizontal,
                                   actor_property->mass, dynamic_state);
                }
            }
        }
    }

    void resolve_collision() {
        for (int i = 0; i < ITERATION_NUM; i++) {
            for (uint32_t j = 0; j < POINT_NUM; j++) {
                ImpactPoint* point = impact_point_set.get_point(j);
                if (point) {
                    cal_point_collision(actor_property, actor_state, point,
                                        delta_time, dynamic_state);
                }
            }
        }
    }
};

#endif
