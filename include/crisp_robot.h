#ifndef CRISP_ROBOT_H
#define CRISP_ROBOT_H

#include <cmath>

#include "crisp_kinematics/RodProperties.h"

#include "ct_anatomy.h"
#include "geo_shapes.h"
#include "robot.h"
#include "visibility_set.h"

#define NUM_TUBES 2
#define AUGMENTED_DIMENSION 28

namespace crisp {

const Idx kCameraIndex = 0;
const Idx kSnareIndex = 1;
const RealNum kCameraTubeRelativeInsertionPoint = 0.0862173;
const RealNum kSnareTubeRelativeInsertionPoint = 0.0387414;
const RealNum kRelativeGraspLocation = -0.00107589;
const RealNum kTubeOuterDiameter = 1.016e-3;
const RealNum kTubeInnerDiameter = 0.8382e-3;
const RealNum kCameraTubeLength = 0.32;
const RealNum kSnareTubeLength = 0.15;

const RealNum kSensorFOV = M_PI/3;
const RealNum kStableThreshold = 0.001;

using KinVec = Eigen::Matrix<RealNum, AUGMENTED_DIMENSION, 1>;
void CheckIndex(Idx i);
Eigen::Vector4d QuatToVec(const Quat& q);

class CRISPDesign {
public:
    CRISPDesign();
    ~CRISPDesign() = default;

    void SetEntryLines(const String file_name);
    Vec3 ComputeEntryPoint(const RealNum relative_position) const;
    Vec3 EntryPoint(Idx i) const;
    Vec3& EntryPoint(Idx i);
    RealNum GraspLocation() const;
    RealNum& GraspLocation();
    RealNum TubeOuterDiameter(Idx i) const;
    RealNum& TubeOuterDiameter(Idx i);
    RealNum TubeInnerDiameter(Idx i) const;
    RealNum& TubeInnerDiameter(Idx i);
    RealNum TubeLength(Idx i) const;
    RealNum& TubeLength(Idx i);
    RealNum MinInsertion(Idx i) const;
    RealNum MaxInsertion(Idx i) const;

private:
    RealNum total_len_;
    std::vector<RealNum> entry_line_lens_;
    std::vector<Vec3> entry_lines_;
    std::vector<Vec3> entry_points_;
    RealNum grasp_location_;
    std::vector<RealNum> outer_diameters_;
    std::vector<RealNum> inner_diameters_;
    std::vector<RealNum> lengths_;
};

class CRISPShape {
public:
    CRISPShape();
    ~CRISPShape() = default;

    geo::Cylinder ShapeSegment(Idx tube_idx, Idx segment_idx) const;
    geo::Cylinder& ShapeSegment(Idx tube_idx, Idx segment_idx);
    void AddSegment(Idx tube_idx, const geo::Cylinder& segment);
    Idx NumSegments(Idx tube_idx) const;
    void DeepCopy(const std::shared_ptr<CRISPShape>& other);

private:
    std::vector<std::vector<geo::Cylinder>> segments_;
};

class CRISPConfig {
public:
    CRISPConfig();
    ~CRISPConfig() = default;

    RealNum TubeInsertion(Idx i) const;
    RealNum& TubeInsertion(Idx i);
    Quat TubeOrientation(Idx i) const;
    Quat& TubeOrientation(Idx i);
    Vec3 TipTranslation() const;
    Vec3& TipTranslation();
    Vec3 TipTangent() const;
    Vec3& TipTangent();
    KinVec KinematicState() const;
    KinVec& KinematicState();
    RealNum Stability() const;
    RealNum& Stability();
    bool IsValid() const;
    bool& IsValid();
    std::shared_ptr<CRISPShape> Shape() const;
    std::shared_ptr<CRISPShape>& Shape();
    void Print(std::ostream &out) const;
    void DeepCopy(const std::shared_ptr<CRISPConfig>& other);
    
private:
    std::vector<RealNum> insertions_;
    std::vector<Quat> orientations_;
    Vec3 tip_translation_;
    Vec3 tip_tangent_;
    KinVec kinematic_state_;
    RealNum stability_;
    bool is_valid_;
    std::shared_ptr<CRISPShape> shape_{nullptr};
};

class CRISPRobot : public Robot {
public:
    CRISPRobot() = default;
    ~CRISPRobot() = default;

    void Initialize();
    void ComputeShape();
    std::shared_ptr<CRISPDesign> Design() const;
    void SetControlInput(const RealNum, const RealNum, const Quat, const Quat);
    std::shared_ptr<CRISPConfig> Config() const;
    std::shared_ptr<CRISPConfig> StartConfig() const;
    void SaveStartConfig();
    void SetAffineToSegmentation(const Affine& affine);
    void SetKinematicStateSeed(const KinVec &v);
    void UnableKinematicStateSeed();

private:
    bool initialized_{false};
    bool use_seed_{false};
    std::shared_ptr<CRISPDesign> design_{nullptr};
    std::shared_ptr<CRISPConfig> config_{nullptr};
    std::shared_ptr<CRISPConfig> start_config_{nullptr};
    std::vector<RodProperties> rod_properties_;
    Affine to_segmentation_{Affine(Mat4::Identity())};
    KinVec kinematics_seed_;
};

class CRISPCollisionDetector{
    using EnvPtr = std::shared_ptr<CTAnatomy>;
public:
    CRISPCollisionDetector(const EnvPtr env);
    ~CRISPCollisionDetector();

    bool IsObstacle(const Vec3& p) const;
    bool Collides(const CRISPConfig& config) const;
    bool Collides(const std::shared_ptr<CRISPConfig> config) const;
    std::vector<Vec3> InCollisionPoints(const CRISPConfig &config) const;
    std::vector<IdxPoint> VisiblePoints(const CRISPConfig &config, const RealNum fov_in_rad=kSensorFOV);
    std::vector<Idx> VisiblePointIndices(const CRISPConfig &config, const RealNum fov_in_rad=kSensorFOV);
    void ComputeVisSetForConfiguration(const CRISPConfig& config, VisibilitySet* vis_set, const RealNum fov_in_rad=kSensorFOV) const;
    void ComputeVisSetForConfiguration(const Vec3& tip_trans, const Vec3& tip_tang, VisibilitySet* vis_set, const RealNum fov_in_rad=kSensorFOV) const;
    EnvPtr Environment() const;

private:
    EnvPtr env_;
    BoolArray3 seen_;
    geo::Cube BoundingCube(const geo::Cylinder& cylinder) const;
    bool InsideCylinder(const Vec3& p, const geo::Cylinder& cylinder) const;
    void SetupSeenArray();
    IdxPoint NearestVisiblePoint(const Vec3& pos, const Vec3& unit_dir, const RealNum upper_bound=1.0) const;
};

}

#endif // CRISP_ROBOT_H
