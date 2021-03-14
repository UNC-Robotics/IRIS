#ifndef CT_ANATOMY_H
#define CT_ANATOMY_H

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stack>

#include "global_common.h"

namespace crisp {

const String kPluralEffusionAnatomyFileName = "../data/pleural_effusion/environment.txt";
const String kEntryLinesFileName = "../data/pleural_effusion/snarelines.txt";
const String kInspectionTargetsFileName = "../data/pleural_effusion/targets.txt";
const IdxPoint kFreePoint = IdxPoint(330, 248, 260);

class CTAnatomy {
  public:
    CTAnatomy() = default;
    CTAnatomy(const String file_env);
    ~CTAnatomy();

    void SetFreePoint(const IdxPoint& p);
    void SetTargetPoints(const String file_target_points);

    void ClearObstacles();
    void ClearTargets();
    void RemoveSmallConnectedComponents(const SizeType size);

    std::vector<IdxPoint> AllTargets() const;
    IdxPoint Target(Idx i) const;
    bool IsObstacle(const IdxPoint& p) const;
    bool IsTarget(const IdxPoint& p) const;
    bool IsTarget(const Idx x, const Idx y, const Idx z) const;
    Idx TargetIndex(const IdxPoint& p) const;
    Idx TargetIndex(const Idx x, const Idx y, const Idx z) const;
    IdxPoint IndexRanges() const ;
    Vec3 VoxelSize() const;
    SizeType NumTargets() const;

    Vec3 VoxelToWorld(const IdxPoint& point) const;
    IdxPoint WorldToVoxel(const Vec3& pos, Idx dimension=0, int sign=0) const;
    Idx CleanUpRegion(const Vec3& center, const Idx radius);
    bool WithinRange(const IdxPoint& p) const;

  private:
    String file_env_;
    String file_target_points_;
    IdxPoint free_point_;
    bool targets_initialized_{false};
    IdxPoint index_range_;
    Vec3 voxel_size_;
    Vec3 world_origin_;

    BoolArray3 obstacle_mask_;
    IdxArray3 target_mask_;
    std::vector<IdxPoint> targets_;

    void ReadEnvFile();
    bool ReadTargetFile();
    void WriteTargetFile() const;
    void AddObstacle(const IdxPoint& p);
    void AddObstacle(const Idx x, const Idx y, const Idx z);
    void AddTarget(const IdxPoint& p);
    void AddTarget(const Idx x, const Idx y, const Idx z);
    bool RemoveObstacle(const IdxPoint& p);
    bool RemoveObstacle(const Idx x, const Idx y, const Idx z);
    bool RemoveTarget(const IdxPoint& p);
    bool RemoveTarget(const Idx x, const Idx y, const Idx z);
    std::vector<IdxPoint> ConnectedComponent(const IdxPoint& source, BoolArray3* mask) const;
    std::vector<IdxPoint> ValidNeighbors(const IdxPoint& p) const;
    void GenerateSurfacePointsAsTargets();

}; // CTAnatomy

}

#endif // CT_ANATOMY_H
