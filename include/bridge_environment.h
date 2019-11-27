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
    bool RayTriangleIntersect(const Vec3& org, const Vec3& dir, const Vec3& v0, const Vec3& v1, const Vec3& v2, RealNum* t, RealNum* u, RealNum* v) const;
};

}

#endif // BRIDGE_ENVIRONMENT_H
