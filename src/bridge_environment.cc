// BSD 3-Clause License

// Copyright (c) 2019, The University of North Carolina at Chapel Hill
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Mengyu Fu

#include <iostream>
#include <fstream>
#include <random>

#include "bridge_environment.h"
#include "io_utils.h"

namespace drone {

BridgeEnvironment::BridgeEnvironment(const Idx seed) {
    srand(seed);

    std::cout << "Loading obj model..." << std::endl;

    io::Vec3s pos;
    io::Vec2s texture;
    io::Vec3s normal;
    io::Indices v_i;
    io::Indices t_i;
    io::Indices n_i;

    io::LoadObjModel(kBridgeStructureFilename, pos, texture, normal, v_i, t_i, n_i);

    for (const auto& p : pos) {
        raw_vertices_.emplace_back(p[0], p[1], p[2]);
    }

    for (const auto i : v_i) {
        vertex_idx_.emplace_back(i);
    }

    InitializeTargets();
    InitializeObstaclePointCloud();

    // io::WriteJSPtCloud("../data/test.js", obstacles_, 0.5, IdxPoint(255, 255, 255));
}

void BridgeEnvironment::AddTargetPoint(const Vec3& p) {
    vertices_.push_back(p);
    nn_.insert(InspectPoint(global_idx_++, p));
    nn_obstacles_.insert(InspectPoint(obstacle_idx_++, p));

    if (global_idx_ == 1) {
        min_x_ = p[0];
        max_x_ = p[0];
        min_y_ = p[1];
        max_y_ = p[1];
        min_z_ = p[2];
        max_z_ = p[2];
    }
    else {
        min_x_ = fmin(min_x_, p[0]);
        max_x_ = fmax(max_x_, p[0]);
        min_y_ = fmin(min_y_, p[1]);
        max_y_ = fmax(max_y_, p[1]);
        min_z_ = fmin(min_z_, p[2]);
        max_z_ = fmax(max_z_, p[2]);
    }
}

void BridgeEnvironment::AddObstacleOnlyPoint(const Vec3& p) {
    nn_obstacles_.insert(InspectPoint(obstacle_idx_++, p));
}

std::optional<NodePair> BridgeEnvironment::NearestTarget(const Vec3& p) const {
    return nn_.nearest(p);
}

NodeSet BridgeEnvironment::NearestTargetsInSphere(const Vec3& p, const RealNum r) const {
    std::vector<NodePair> nbh;
    nn_.nearest(nbh, p, MAX_COVERAGE_SIZE, r);

    NodeSet nodes(nbh.begin(), nbh.end());
    return nodes;
}

NodeSet BridgeEnvironment::NearestObstaclesInShpere(const Vec3& p, const RealNum r) const {
    std::vector<NodePair> nbh;
    nn_obstacles_.nearest(nbh, p, MAX_COVERAGE_SIZE, r);

    NodeSet nodes(nbh.begin(), nbh.end());
    return nodes;
}

bool BridgeEnvironment::IsCollisionFree(const Vec3& p, const RealNum r) const {
    std::vector<NodePair> nbh;
    nn_obstacles_.nearest(nbh, p, obstacle_idx_, r);

    return (nbh.size() == 0);
}

void BridgeEnvironment::InitializeTargets() {
    for (const auto& v : raw_vertices_) {
        // Avoid duplicating target points.
        const auto neartest = nn_.nearest(v)->first.point;

        if ((neartest - v).norm() < 1e-3) {
            continue;
        }

        AddTargetPoint(v);
    }

    std::cout << "Number of targets: " << global_idx_ << std::endl;
}

void BridgeEnvironment::InitializeObstaclePointCloud(const RealNum unit_area) {
    for (auto i = 0; i < vertex_idx_.size(); i += 3) {
        auto p0 = raw_vertices_[vertex_idx_[i]];
        auto p1 = raw_vertices_[vertex_idx_[i+1]];
        auto p2 = raw_vertices_[vertex_idx_[i+2]];

        faces_.emplace_back(vertex_idx_[i], vertex_idx_[i+1], vertex_idx_[i+2]);

        auto area = TriangleArea((p1 - p0).norm(), (p2 - p1).norm(), (p0 - p2).norm());

        if (area < unit_area) {
            continue;
        }

        Idx num_points = std::floor(area / unit_area);

        for (auto j = 0; j < num_points; ++j) {
            auto alpha = RandomNum(0, 1);
            auto beta = RandomNum(0, 1 - alpha);
            auto gamma = 1 - alpha - beta;
            auto new_point = alpha * p0 + beta * p1 + gamma * p2;
            AddObstacleOnlyPoint(new_point);
        }
    }

    std::cout << "Number of obstacles: " << obstacle_idx_ << std::endl;
}

RealNum BridgeEnvironment::TriangleArea(const RealNum a, const RealNum b, const RealNum c) const {
    auto s = 0.5*(a + b + c);
    return sqrt(s*(s-a)*(s-b)*(s-c));
}

RealNum BridgeEnvironment::RandomNum(const RealNum min, const RealNum max) const {
    return min + ((RealNum)rand()/RAND_MAX) * (max - min);
}

RealNum BridgeEnvironment::EnvironmentBoundary(const Idx dim, const bool request_min) {
    if (dim == 0) {
        if (request_min) {
            return min_x_;
        }

        return max_x_;
    }
    else if (dim == 1) {
        if (request_min) {
            return min_y_;
        }

        return max_y_;
    }

    if (request_min) {
        return min_z_;
    }

    return max_z_;
}

SizeType BridgeEnvironment::NumTargets() const {
    return global_idx_;
}

// std::vector<SizeType> BridgeEnvironment::GetVisiblePointIndices(const Vec3& pos, const Vec3& tang,
//         const RealNum fov_in_rad, const RealNum min_dof, const RealNum max_dof) const {
//     std::vector<SizeType> visible_points;

//     // Compute points in valid range.
//     auto large_set = NearestTargetsInSphere(pos, max_dof);
//     auto small_set = NearestTargetsInSphere(pos, min_dof);

//     for (auto& p : small_set) {
//         large_set.erase(p);
//     }

//     auto obstacle_set = NearestObstaclesInShpere(pos, max_dof);

//     std::vector<Vec3> rays;

//     for (auto& node : obstacle_set) {
//         auto p = node.first.point;
//         auto camera_to_point = p - pos;
//         auto angle = std::acos(camera_to_point.normalized().dot(tang.normalized()));

//         if (angle > 0.5*fov_in_rad) {
//             continue;
//         }

//         bool visible = true;

//         for (auto& other : rays) {
//             auto angle_diff = std::acos(camera_to_point.normalized().dot(other.normalized()));

//             if (fabs(angle_diff) < 2.0/180.0 * M_PI) {
//                 visible = false;
//                 break;
//             }
//         }

//         if (visible && node.first.idx < global_idx_) {
//             visible_points.push_back(node.first.idx);
//         }

//         rays.push_back(camera_to_point);
//     }

//     return visible_points;
// }

std::vector<SizeType> BridgeEnvironment::GetVisiblePointIndices(const Vec3& pos, const Vec3& tang,
        const RealNum fov_in_rad, const RealNum min_dof, const RealNum max_dof) const {
    std::vector<SizeType> visible_points;

    // Compute points in valid range.
    auto large_set = NearestTargetsInSphere(pos, max_dof);
    auto small_set = NearestTargetsInSphere(pos, min_dof);

    for (const auto& p : small_set) {
        large_set.erase(p);
    }

    // Ray shooting to every point
    for (const auto& node : large_set) {
        const auto p = node.first.point;
        const auto camera_to_point = (p - pos);
        const auto camera_to_point_normalized = camera_to_point.normalized();
        const auto angle = std::acos(camera_to_point_normalized.dot(tang.normalized()));

        if (angle > 0.5*fov_in_rad) {
            continue;
        }

        bool visible = true;
        const auto dist = camera_to_point.norm();
        RealNum t, u, v;

        for (const auto& f : faces_) {
            const auto& v0 = raw_vertices_[f[0]];
            const auto& v1 = raw_vertices_[f[1]];
            const auto& v2 = raw_vertices_[f[2]];

            if (RayTriangleIntersect(pos, camera_to_point_normalized, v0, v1, v2, &t, &u, &v)
                    && t < dist - EPS) {
                visible = false;
                break;
            }
        }

        if (visible && node.first.idx < global_idx_) {
            visible_points.push_back(node.first.idx);
        }
    }

    return visible_points;
}

bool BridgeEnvironment::RayTriangleIntersect(const Vec3& org, const Vec3& dir, const Vec3& v0,
        const Vec3& v1, const Vec3& v2, RealNum* t, RealNum* u, RealNum* v) const {
    const auto v0v1 = v1 - v0;
    const auto v0v2 = v2 - v0;
    const auto pvec = dir.cross(v0v2);
    RealNum det = v0v1.dot(pvec);

    if (std::fabs(det) < EPS) {
        return false;
    }

    RealNum inv_det = 1.0/det;

    const auto tvec = org - v0;
    *u = tvec.dot(pvec) * inv_det;

    if (*u < 0 || *u > 1) {
        return false;
    }

    const auto qvec = tvec.cross(v0v1);
    *v = dir.dot(qvec) * inv_det;

    if (*v < 0 || *u + *v > 1) {
        return false;
    }

    *t = v0v2.dot(qvec) * inv_det;
    return true;
}

bool BridgeEnvironment::IfCorrectDirection(const Vec3& pos, const Vec3& tang,
        const RealNum fov_in_rad, const RealNum dist) const {
    auto obstacle_set = NearestObstaclesInShpere(pos, dist);

    for (auto& node : obstacle_set) {
        auto p = node.first.point;
        auto camera_to_point = p - pos;
        auto angle = std::acos(camera_to_point.normalized().dot(tang.normalized()));

        if (angle < 0.5*fov_in_rad && node.first.idx < global_idx_) {
            return true;
        }
    }

    return false;
}

std::vector<Vec3> BridgeEnvironment::IndicesToPoints(const std::vector<Idx>& indices) const {
    std::vector<Vec3> points;

    for (auto& i : indices) {
        points.push_back(vertices_[i]);
    }

    return points;
}

}
