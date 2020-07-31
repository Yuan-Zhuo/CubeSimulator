#pragma once

#ifndef _PHYSICSSTATE_HPP_

#define _PHYSICSSTATE_HPP_

#include <iostream>
#include <limits>

#include "Eigen/Eigen/Dense"
// #include "Eigen/Dense"
#include "MagicNum.hpp"

using namespace std;
using namespace Eigen;

struct ActorProperty {
    float mass;
    Vector3f box_extent;

    float linear_friction_coefficient;
    float impulse_coefficient;
    Matrix3f inertia;

    ActorProperty(float mass_,
                  Vector3f box_extent_,
                  float linear_friction_coefficient_,
                  float impulse_coefficient_,
                  Matrix3f inertia_)
        : mass(mass_),
          box_extent(box_extent_),
          linear_friction_coefficient(linear_friction_coefficient_),
          impulse_coefficient(impulse_coefficient_),
          inertia(inertia_) {}
};

struct ActorState {
    Vector3f origin;
    Vector3f relative_vec;
    Quaternionf quat;

    ActorState(Vector3f origin_, Vector3f relative_vec_, Quaternionf quat_)
        : origin(origin_), relative_vec(relative_vec_), quat(quat_) {}
};

struct DynamicState {
    // velocity
    Vector3f linear_vel;
    Vector3f angular_vel;

    DynamicState() {
        linear_vel.setZero(3);
        angular_vel.setZero(3);
    }
};

struct ImpactPoint {
    // basic info
    Vector3f hit_point;
    Vector3f relative_location;
    float depth;
    // cache
    bool cache;
    Vector3f impluse_vertical;
    Vector3f impluse_horizontal;
    Vector3f angular_inertia_vertical;
    Vector3f angular_inertia_horizontal;

    void set_basic(Vector3f hit_point_, Vector3f origin, float depth_) {
        cache = false;
        hit_point = hit_point_;
        depth = depth_;
        relative_location = hit_point_ - origin;
        impluse_vertical.setZero();
        impluse_horizontal.setZero();
        angular_inertia_vertical.setZero();
        angular_inertia_horizontal.setZero();
    }

    void set_cache(Vector3f impluse_vertical_,
                   Vector3f impluse_horizontal_,
                   Vector3f angular_inertia_vertical_,
                   Vector3f angular_inertia_horizontal_) {
        cache = true;
        impluse_vertical = impluse_vertical_;
        impluse_horizontal = impluse_horizontal_;
        angular_inertia_vertical = angular_inertia_vertical_;
        angular_inertia_horizontal = angular_inertia_horizontal_;
    }
};

class ImpactPointSet {
   private:
    ImpactPoint point_set[POINT_NUM];
    uint32_t size;
    uint32_t cnt;
    bool floor;
    float base;

    uint32_t get_remove_idx(uint32_t deepest_idx, Vector3f& hit_point) {
        if (POINT_NUM != 4)
            return -1;

        float area[4];
        for (uint32_t i = 0; i < 4; i++) {
            Vector3f vec1 = hit_point - point_set[(i + 1) % 4].hit_point;
            Vector3f vec2 = point_set[(i + 2) % 4].hit_point -
                            point_set[(i + 3) % 4].hit_point;
            area[i] = vec1.cross(vec2).norm();
        }
        area[deepest_idx] = 0.f;

        uint32_t idx = deepest_idx;
        float max_area = 0.f;
        for (uint32_t i = 0; i < 4; i++) {
            if (max_area <= area[i]) {
                idx = i;
                max_area = area[i];
            }
        }
        return idx;
    }

    void remove_element(uint32_t idx) {
        for (uint32_t i = idx; i < size - 1; i++) {
            point_set[i] = point_set[i + 1];
        }
        size--;
    }

    void update_set(ImpactPoint& point) {
        if (!floor) {
            base = point.hit_point.z();
            floor = true;
        }

        if (size < POINT_NUM) {
            point_set[size++] = point;
            return;
        }

        float max_depth = numeric_limits<float>::max();
        uint32_t deepest_idx = -1;
        for (uint32_t i = 0; i < POINT_NUM; i++) {
            if (point_set[i].depth < max_depth) {
                deepest_idx = i;
                max_depth = point_set[i].depth;
            }
        }

        uint32_t idx = get_remove_idx(deepest_idx, point.hit_point);
        if (idx == -1)
            return;
        point_set[idx] = point;
    }

   public:
    ImpactPointSet() {
        size = 0;
        floor = false;
        base = 0;
        cnt = 0;
    }

    void apply_point(Vector3f hit_point_, Vector3f origin_) {
        ImpactPoint point;
        point.set_basic(hit_point_, origin_, base - hit_point_.z());
        update_set(point);
        cnt++;
    }

    void update_point(Vector3f origin) {
        for (uint32_t i = 0; i < size; i++) {
            Vector3f cur_hit_point = origin + point_set[i].relative_location;
            Vector3f pre_origin =
                point_set[i].hit_point - point_set[i].relative_location;
            bool still_collision = ((base - cur_hit_point.z()) > 0);
            bool still_close = ((pre_origin - origin).norm() < MIN_DISTANCE);
            if (!still_collision || !still_close)
                remove_element(i);
        }
    }

    ImpactPoint* get_point(uint32_t idx) {
        if (idx >= size)
            return nullptr;
        return &point_set[idx];
    }

    uint32_t get_size() { return size; }

    uint32_t get_cnt() { return cnt; }
};

#endif
