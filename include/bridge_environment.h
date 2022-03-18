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

#ifndef BRIDGE_ENVIRONMENT_H
#define BRIDGE_ENVIRONMENT_H

#include <set>

#include "global_common.h"
#include "nigh/lp_space.hpp"
#include "nigh/kdtree_batch.hpp"

namespace drone {

namespace nigh = unc::robotics::nigh;

const String kBridgeStructureFilename = "../data/bridge/bridge_small.obj";

struct InspectPoint {
    SizeType idx;
    Vec3 point;
    InspectPoint(const SizeType i, const Vec3& p)
        : idx(i), point(p) {
    }

};

struct InspectPointKey {
    const Vec3& operator() (const InspectPoint& node) const {
        return node.point;
    }
};

using NodePair = std::pair<InspectPoint, RealNum>;
struct Cmp {
    bool operator() (const NodePair n1, const NodePair n2) const {
        // smaller key comes first
        return n1.second < n2.second;
    }
};
using NodeSet = std::set<NodePair, Cmp>;

class BridgeEnvironment {
  public:
    BridgeEnvironment(const Idx seed=1);
    ~BridgeEnvironment() = default;

    void AddTargetPoint(const Vec3& p);
    void AddObstacleOnlyPoint(const Vec3& p);

    std::optional<NodePair> NearestTarget(const Vec3& p) const;
    NodeSet NearestTargetsInSphere(const Vec3& p, const RealNum r) const;
    NodeSet NearestObstaclesInShpere(const Vec3& p, const RealNum r) const;
    bool IsCollisionFree(const Vec3& p, const RealNum r) const;
    RealNum EnvironmentBoundary(const Idx dim, const bool request_min=true);

    SizeType NumTargets() const;
    std::vector<SizeType> GetVisiblePointIndices(const Vec3& pos, const Vec3& tang,
            const RealNum fov_in_rad,  const RealNum min_dof, const RealNum max_dof) const;
    bool IfCorrectDirection(const Vec3& pos, const Vec3& tang, const RealNum fov_in_rad,
                            const RealNum dist) const;
    std::vector<Vec3> IndicesToPoints(const std::vector<Idx>& indices) const;

  private:
    SizeType global_idx_{0};
    SizeType obstacle_idx_{0};
    nigh::Nigh<InspectPoint, nigh::L2Space<RealNum, 3>, InspectPointKey, nigh::Concurrent, nigh::KDTreeBatch<>>
            nn_;
    nigh::Nigh<InspectPoint, nigh::L2Space<RealNum, 3>, InspectPointKey, nigh::Concurrent, nigh::KDTreeBatch<>>
            nn_obstacles_;
    std::vector<Vec3> raw_vertices_;
    std::vector<Vec3> vertices_;
    std::vector<SizeType> vertex_idx_;
    std::vector<IdxPoint> faces_;
    RealNum min_x_;
    RealNum max_x_;
    RealNum min_y_;
    RealNum max_y_;
    RealNum min_z_;
    RealNum max_z_;

    void InitializeTargets();
    void InitializeObstaclePointCloud(const RealNum unit_area=0.2);
    RealNum TriangleArea(const RealNum a, const RealNum b, const RealNum c) const;
    RealNum RandomNum(const RealNum min, const RealNum max) const;
    bool RayTriangleIntersect(const Vec3& org, const Vec3& dir, const Vec3& v0, const Vec3& v1,
                              const Vec3& v2, RealNum* t, RealNum* u, RealNum* v) const;
};

}

#endif // BRIDGE_ENVIRONMENT_H
