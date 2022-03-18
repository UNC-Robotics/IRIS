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

// / VU's code
#include "crisp_kinematics/State.h"
#include "crisp_kinematics/CosseratRod.h"
#include "crisp_kinematics/Entities.h"
#include "crisp_kinematics/DerivativeEntities.h"
#include "crisp_kinematics/ConstraintEntities.h"
#include "crisp_kinematics/ParallelCosseratRod.h"
#include "crisp_kinematics/Observer.h"
// end of VU's code

#include "crisp_robot.h"

namespace crisp {

void CheckIndex(Idx i) {
    if (i >= NUM_TUBES) {
        std::cerr << "Exceeding maximum number of tubes" << std::endl;
        exit(1);
    }
}

Eigen::Vector4d QuatToVec(const Quat& q) {
    return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

CRISPDesign::CRISPDesign() {
    entry_lines_.resize(0);
    entry_line_lens_.resize(0);
    entry_points_ = std::vector<Vec3>(2, Vec3(0, 0, 0));
    grasp_location_ = 0;
    outer_diameters_ = std::vector<RealNum>(2, 0);
    inner_diameters_ = std::vector<RealNum>(2, 0);
    lengths_ = std::vector<RealNum>(2, 0);
}

void CRISPDesign::SetEntryLines(const String file_name) {
    std::ifstream fin;
    fin.open(file_name);

    if (!fin.is_open()) {
        std::cerr << "Unable to open file: " << file_name << std::endl;
        exit(1);
    }

    String line;

    while (getline(fin, line)) {
        std::istringstream sin(line);
        String field;
        Vec3 point;
        Idx d = 0;

        while (getline(sin, field, ',')) {
            if (d > 1) {
                point[d-2] = std::stof(field.c_str());
            }

            d++;
        }

        entry_lines_.push_back(point);
    }

    fin.close();

    total_len_ = 0;

    for (Idx i = 0; i < entry_lines_.size(); i += 2) {
        RealNum l = (entry_lines_[i] - entry_lines_[i+1]).norm();
        entry_line_lens_.push_back(l);
        total_len_ += l;
    }

    std::cout << "File read: " << file_name << std::endl;
    std::cout << "Total entry line length: " << total_len_ << std::endl;
}

Vec3 CRISPDesign::ComputeEntryPoint(const RealNum relative_position) const {
    if (entry_lines_.empty()) {
        std::cerr << "No available entry lines." << std::endl;
        exit(1);
    }

    if (relative_position > total_len_) {
        std::cerr << "Exceeding maximum entry line length." << std::endl;
        exit(1);
    }

    Vec3 entry_point;

    RealNum rp = relative_position;

    for (Idx j = 0; j < entry_lines_.size(); j += 2) {
        RealNum l = entry_line_lens_[j/2];

        if (rp < l) {
            entry_point = rp/l*(entry_lines_[j+1] - entry_lines_[j]) + entry_lines_[j];
            break;
        }
        else {
            rp -= l;
        }
    }

    std::cout << "Entry point: " << entry_point.transpose() << std::endl;

    return entry_point;
}

Vec3 CRISPDesign::EntryPoint(Idx i) const {
    CheckIndex(i);
    return entry_points_[i];
}

Vec3& CRISPDesign::EntryPoint(Idx i) {
    CheckIndex(i);
    return entry_points_[i];
}

RealNum CRISPDesign::GraspLocation() const {
    // Relative to the tip of camera tube.
    return grasp_location_;
}

RealNum& CRISPDesign::GraspLocation() {
    return grasp_location_;
}

RealNum CRISPDesign::TubeOuterDiameter(Idx i) const {
    CheckIndex(i);
    return outer_diameters_[i];
}

RealNum& CRISPDesign::TubeOuterDiameter(Idx i) {
    CheckIndex(i);
    return outer_diameters_[i];
}

RealNum CRISPDesign::TubeInnerDiameter(Idx i) const {
    CheckIndex(i);
    return inner_diameters_[i];
}

RealNum& CRISPDesign::TubeInnerDiameter(Idx i) {
    CheckIndex(i);
    return inner_diameters_[i];
}

RealNum CRISPDesign::TubeLength(Idx i) const {
    CheckIndex(i);
    return lengths_[i];
}

RealNum& CRISPDesign::TubeLength(Idx i) {
    CheckIndex(i);
    return lengths_[i];
}

RealNum CRISPDesign::MinInsertion(Idx i) const {
    CheckIndex(i);

    if (i == 0) {
        return -grasp_location_;
    }

    return 0;
}

RealNum CRISPDesign::MaxInsertion(Idx i) const {
    CheckIndex(i);
    return lengths_[i];
}

CRISPShape::CRISPShape() {
    segments_.resize(NUM_TUBES);
}

geo::Cylinder CRISPShape::ShapeSegment(Idx tube_idx, Idx segment_idx) const {
    CheckIndex(tube_idx);

    auto& segments = segments_[tube_idx];

    if (segment_idx < segments.size()) {
        return segments[segment_idx];
    }
    else {
        std::cerr << "Exceeding maximum number of segments." << std::endl;
        exit(1);
    }

}

geo::Cylinder& CRISPShape::ShapeSegment(Idx tube_idx, Idx segment_idx) {
    CheckIndex(tube_idx);

    auto& segments = segments_[tube_idx];

    if (segment_idx < segments.size()) {
        return segments[segment_idx];
    }
    else {
        std::cerr << "Exceeding maximum number of segments." << std::endl;
        exit(1);
    }
}

void CRISPShape::AddSegment(Idx tube_idx, const geo::Cylinder& segment) {
    CheckIndex(tube_idx);
    segments_[tube_idx].emplace_back(segment);
}

Idx CRISPShape::NumSegments(Idx tube_idx) const {
    CheckIndex(tube_idx);
    return segments_[tube_idx].size();
}

void CRISPShape::DeepCopy(const std::shared_ptr<CRISPShape>& other) {
    segments_.resize(NUM_TUBES);

    for (Idx i = 0; i < NUM_TUBES; ++i) {
        segments_[i].clear();
        auto n = other->NumSegments(i);

        for (Idx j = 0; j < n; ++j) {
            this->AddSegment(i, other->ShapeSegment(i, j));
        }
    }
}

CRISPConfig::CRISPConfig() {
    insertions_ = std::vector<RealNum>(2, 0);
    orientations_ = std::vector<Quat>(2, Quat(Mat3::Identity()));
    tip_translation_ = Vec3(0, 0, 0);
    tip_tangent_ = Vec3(0, 0, 0);
    stability_ = 0;
    is_valid_ = true;
}

RealNum CRISPConfig::TubeInsertion(Idx i) const {
    CheckIndex(i);
    return insertions_[i];
}

RealNum& CRISPConfig::TubeInsertion(Idx i) {
    CheckIndex(i);
    return insertions_[i];
}

Quat CRISPConfig::TubeOrientation(Idx i) const {
    CheckIndex(i);
    return orientations_[i];
}

Quat& CRISPConfig::TubeOrientation(Idx i) {
    CheckIndex(i);
    return orientations_[i];
}

Vec3 CRISPConfig::TipTranslation() const {
    return tip_translation_;
}

Vec3& CRISPConfig::TipTranslation() {
    return tip_translation_;
}

Vec3 CRISPConfig::TipTangent() const {
    return tip_tangent_;
}

Vec3& CRISPConfig::TipTangent() {
    return tip_tangent_;
}

KinVec CRISPConfig::KinematicState() const {
    return kinematic_state_;
}

KinVec& CRISPConfig::KinematicState() {
    return kinematic_state_;
}

RealNum CRISPConfig::Stability() const {
    return stability_;
}

RealNum& CRISPConfig::Stability() {
    return stability_;
}

bool CRISPConfig::IsValid() const {
    return is_valid_;
}

bool& CRISPConfig::IsValid() {
    return is_valid_;
}

std::shared_ptr<CRISPShape> CRISPConfig::Shape() const {
    return shape_;
}

std::shared_ptr<CRISPShape>& CRISPConfig::Shape() {
    return shape_;
}

void CRISPConfig::DeepCopy(const std::shared_ptr<CRISPConfig>& other) {
    for (Idx i = 0; i < NUM_TUBES; ++i) {
        this->TubeInsertion(i) = other->TubeInsertion(i);
        this->TubeOrientation(i) = other->TubeOrientation(i);
    }

    this->TipTranslation() = other->TipTranslation();
    this->TipTangent() = other->TipTangent();
    this->KinematicState() = other->KinematicState();
    this->Stability() = other->Stability();
    this->IsValid() = other->IsValid();

    if (!shape_) {
        shape_.reset(new CRISPShape());
    }

    this->Shape()->DeepCopy(other->Shape());
}

void CRISPConfig::Print(std::ostream& out) const {
    for (Idx i = 0; i < NUM_TUBES; ++i) {
        out << "Tube " << i << std::endl;
        out << "insertion = " << insertions_[i] << std::endl;
        out << "orientation = " << orientations_[i].w() << " "
            << orientations_[i].vec().transpose() << std::endl;
    }

    out << "Tip translation = " << tip_translation_.transpose() << std::endl;
    out << "Tip tangent = " << tip_tangent_.transpose() << std::endl;

    out << "Kinematic state = ";

    for (Idx i = 0; i < AUGMENTED_DIMENSION; ++i) {
        out << kinematic_state_[i] << " ";
    }

    out << std::endl;

    out << "Stability = " << stability_ << std::endl;
    out << "If valid = " << is_valid_ << std::endl;
}


void CRISPRobot::Initialize() {
    // Initialize robot design.
    design_.reset(new CRISPDesign());
    design_->SetEntryLines(kEntryLinesFileName);
    design_->EntryPoint(kCameraIndex)
        = design_->ComputeEntryPoint(kCameraTubeRelativeInsertionPoint);
    design_->EntryPoint(kSnareIndex)
        = design_->ComputeEntryPoint(kSnareTubeRelativeInsertionPoint);
    design_->GraspLocation() = kRelativeGraspLocation;
    design_->TubeOuterDiameter(kCameraIndex) = kTubeOuterDiameter;
    design_->TubeOuterDiameter(kSnareIndex) = kTubeOuterDiameter;
    design_->TubeInnerDiameter(kCameraIndex) = kTubeInnerDiameter;
    design_->TubeInnerDiameter(kSnareIndex) = kTubeInnerDiameter;
    design_->TubeLength(kCameraIndex) = kCameraTubeLength;
    design_->TubeLength(kSnareIndex) = kSnareTubeLength;

    // Initialize rod properties.
    for (Idx i = 0; i < NUM_TUBES; ++i) {
        rod_properties_.emplace_back(design_->TubeInnerDiameter(i), design_->TubeOuterDiameter(i));
    }

    std::cout << "CRISP robot initialized!" << std::endl;
    initialized_ = true;
}

void CRISPRobot::SetControlInput(const RealNum camera_tube_insertion,
                                 const RealNum snare_tube_insertion,
                                 const Quat camer_tube_orientation,
                                 const Quat snare_tube_orientation) {
    if (!config_) {
        config_.reset(new CRISPConfig());
    }

    config_->TubeInsertion(kCameraIndex) = camera_tube_insertion;
    config_->TubeInsertion(kSnareIndex) = snare_tube_insertion;
    config_->TubeOrientation(kCameraIndex) = camer_tube_orientation;
    config_->TubeOrientation(kSnareIndex) = snare_tube_orientation;
}

std::shared_ptr<CRISPConfig> CRISPRobot::Config() const {
    return config_;
}

void CRISPRobot::SaveStartConfig() {
    start_config_.reset(new CRISPConfig());

    start_config_->DeepCopy(config_);
}

std::shared_ptr<CRISPConfig> CRISPRobot::StartConfig() const {
    return start_config_;
}

void CRISPRobot::ComputeShape() {
    if (!initialized_) {
        std::cerr << "Robot design is not initialized." << std::endl;
        exit(1);
    }

    if (!config_) {
        std::cerr << "No valid configuration." << std::endl;
        exit(1);
    }

    auto tool_rod_properties = rod_properties_[kCameraIndex];
    auto snare_rod_properties = rod_properties_[kSnareIndex];

    Eigen::Vector3d tool_rod_p0 = design_->EntryPoint(kCameraIndex).cast<double>();
    Eigen::Vector3d snare_rod_p0 = design_->EntryPoint(kSnareIndex).cast<double>();

    double grasp_location = (double) design_->GraspLocation();

    RealNum tool_rod_insertion = config_->TubeInsertion(kCameraIndex);
    RealNum snare_rod_insertion = config_->TubeInsertion(kSnareIndex);
    Quat tool_rod_quat = config_->TubeOrientation(kCameraIndex);
    auto tool_rod_q0 = QuatToVec(tool_rod_quat);
    Quat snare_rod_quat = config_->TubeOrientation(kSnareIndex);
    auto snare_rod_q0 = QuatToVec(snare_rod_quat);


    /**************
    start of VU's code
    **************/

    // snare rod
    BasePoseConstraint snare_pose_constraint(snare_rod_p0, snare_rod_q0);
    LengthConstraint snare_length_constraint(snare_rod_insertion);
    VariableLengthRod<decltype(snare_pose_constraint),
                      decltype(snare_length_constraint)> snare_rod(snare_rod_properties, snare_pose_constraint,
                              snare_length_constraint);

    // tool rod
    FullGraspConstraint< decltype(snare_rod) > grasp_constraint(snare_rod,
            grasp_location); // the location is an offset from the tip
    LoadFreeConstraint load_free_constraint;
    LengthConstraint tool_length_constraint(tool_rod_insertion);
    BasePoseConstraint tool_pose_constraint(tool_rod_p0, tool_rod_q0);
    VariableLengthRod<decltype(grasp_constraint),
                      decltype(load_free_constraint),
                      decltype(tool_pose_constraint),
                      decltype(tool_length_constraint)> tool_rod(tool_rod_properties, grasp_constraint,
                              load_free_constraint, tool_pose_constraint, tool_length_constraint);

    // Total system.
    System<decltype(tool_rod)> system(tool_rod);

    if (AUGMENTED_DIMENSION != system.state_dimension) {
        std::cerr << "System state dimension is " << system.state_dimension
                  << ", which is not equal to pre-defined AUGMENTED_DIMENSION = " << AUGMENTED_DIMENSION << std::endl;
        exit(1);
    }

    // Just a typedef to make the following code more readable.
    using ParallelRods = System<decltype(tool_rod)>;

    // Create the initial state.
    ParallelRods::StateVector initial_state_vector =
        ParallelRods::StateVector::Zero(); // initialize to 0 at first

    // This code initializes the state vector for forward kinematics.
    // Initialize the portion of the state corresponding to snare rod.
    snare_rod.interpret(initial_state_vector).position() = snare_pose_constraint.position();
    snare_rod.interpret(initial_state_vector).quaternion() = snare_pose_constraint.quaternion();

    if (use_seed_) {
        snare_rod.interpret(initial_state_vector).moment() = Eigen::Vector3d(kinematics_seed_[21],
                kinematics_seed_[22],
                kinematics_seed_[23]);

        snare_rod.interpret(initial_state_vector).force() = Eigen::Vector3d(kinematics_seed_[24],
                kinematics_seed_[25],
                kinematics_seed_[26]);
    }
    else {
        snare_rod.interpret(initial_state_vector).moment() = Eigen::Vector3d(0, 0,
                0); //Eigen::Vector3d(-0.009, 0.002, -0.004);
        snare_rod.interpret(initial_state_vector).force() = Eigen::Vector3d(0, 0,
                0); //Eigen::Vector3d(-0.011, -0.115, 0.046);
    }

    snare_rod.interpret(initial_state_vector).length() = snare_length_constraint.length();

    // Initialize the portion of the state corresponding to tool rod.
    tool_rod.interpret(initial_state_vector).position() =  tool_pose_constraint.position();
    tool_rod.interpret(initial_state_vector).quaternion() = tool_pose_constraint.quaternion();

    if (use_seed_) {
        tool_rod.interpret(initial_state_vector).moment() = Eigen::Vector3d(kinematics_seed_[7],
                kinematics_seed_[8],
                kinematics_seed_[9]);
        tool_rod.interpret(initial_state_vector).force() = Eigen::Vector3d(kinematics_seed_[10],
                kinematics_seed_[11],
                kinematics_seed_[12]);
    }
    else {
        tool_rod.interpret(initial_state_vector).moment() = Eigen::Vector3d(0, 0,
                0); //Eigen::Vector3d(-0.013, 0.0, 0.004); // initial guess
        tool_rod.interpret(initial_state_vector).force() = Eigen::Vector3d(0, 0,
                0); //Eigen::Vector3d(0.011, 0.115, -0.046); // initial guess
    }

    tool_rod.interpret(initial_state_vector).length() = tool_length_constraint.length();

    // Solve the system whether it is forward or inverse kinematics.
    ParallelRods::StateVector solution_state_vector;
    int result = system.solve(initial_state_vector, solution_state_vector);

    // Integrate the system again after it's solved to print out the rod positions at a higher spatial resolution.
    ObserverList<decltype(system), decltype(tool_rod)> tool_rod_observers(tool_rod, 30);
    ObserverList<decltype(system), decltype(snare_rod)> snare_rod_observers(snare_rod, 30);
    System<decltype(tool_rod)>::Solution solution;
    system.initialize<decltype(snare_rod_observers), decltype(tool_rod_observers)>(solution,
            solution_state_vector, snare_rod_observers, tool_rod_observers);
    system.integrate(solution);

    double residual = solution.error();

    /**************
    end of VU's code
    **************/


    // Save state vector.
    KinVec result_vec;

    for (Idx i = 0; i < AUGMENTED_DIMENSION; ++i) {
        result_vec[i] = solution_state_vector(i);
    }

    config_->KinematicState() = result_vec;
    config_->Stability() = (RealNum)residual;
    config_->IsValid() = (result == -1)? false : true;

    // save robot shape
    config_->Shape().reset(new CRISPShape());

    for (Idx i = 0; i < tool_rod_observers.size() - 1; ++i) {
        auto cylinder = geo::Cylinder(to_segmentation_*tool_rod_observers(
                                          i).state().position().cast<RealNum>(),
                                      to_segmentation_*tool_rod_observers(i+1).state().position().cast<RealNum>(),
                                      design_->TubeOuterDiameter(kCameraIndex));
        config_->Shape()->AddSegment(kCameraIndex, cylinder);

        cylinder = geo::Cylinder(to_segmentation_*snare_rod_observers(i).state().position().cast<RealNum>(),
                                 to_segmentation_*snare_rod_observers(i+1).state().position().cast<RealNum>(),
                                 design_->TubeOuterDiameter(kSnareIndex));

        config_->Shape()->AddSegment(kSnareIndex, cylinder);
    }

    SizeType tool_cylinder_num = config_->Shape()->NumSegments(kCameraIndex);

    Vec3 tip_translation = config_->Shape()->ShapeSegment(kCameraIndex, tool_cylinder_num - 1).p2;
    Vec3 tip_tangent = (tip_translation - config_->Shape()->ShapeSegment(kCameraIndex,
                        tool_cylinder_num - 2).p2).normalized();

    config_->TipTranslation() = tip_translation;
    config_->TipTangent() = tip_tangent;
}

std::shared_ptr<CRISPDesign> CRISPRobot::Design() const {
    return design_;
}

void CRISPRobot::SetAffineToSegmentation(const Affine& affine) {
    to_segmentation_ = affine;
}

void CRISPRobot::SetKinematicStateSeed(const KinVec& v) {
    kinematics_seed_ = v;
    use_seed_ = true;
}

void CRISPRobot::UnableKinematicStateSeed() {
    use_seed_ = false;
}

CRISPCollisionDetector::CRISPCollisionDetector(const EnvPtr env) {
    env_ = env;
    SetupSeenArray();
}

CRISPCollisionDetector::~CRISPCollisionDetector() {
    seen_.resize(boost::extents[0][0][0]);
}

void CRISPCollisionDetector::SetupSeenArray() {
    if (seen_.shape()[0] == 0) {
        IdxPoint range = env_->IndexRanges();
        seen_.resize(boost::extents[range[0]][range[1]][range[2]]);
    }

    for (Idx x = 0; x < seen_.shape()[0]; ++x) {
        for (Idx y = 0; y < seen_.shape()[1]; ++y) {
            for (Idx z = 0; z < seen_.shape()[2]; ++z) {
                seen_[x][y][z] = false;
            }
        }
    }
}

bool CRISPCollisionDetector::IsObstacle(const Vec3& p) const {
    IdxPoint v = env_->WorldToVoxel(p);

    return env_->IsObstacle(v);
}

bool CRISPCollisionDetector::Collides(const CRISPConfig& config) const {
    auto shape = config.Shape();

    if (shape == nullptr) {
        std::cerr << "Shape of the robot is not valid!" << std::endl;
        exit(1);
    }

    for (Idx t = 0; t < NUM_TUBES; ++t) {
        Idx num_cylinders = shape->NumSegments(t);

        for (Idx c = 0; c < num_cylinders; ++c) {
            geo::Cube cube = BoundingCube(shape->ShapeSegment(t, c));

            IdxPoint min_index = env_->WorldToVoxel(cube.low);
            IdxPoint max_index = env_->WorldToVoxel(cube.high);

            for (Idx x = min_index[0]; x < max_index[0]; ++x) {
                for (Idx y = min_index[1]; y < max_index[1]; ++y) {
                    for (Idx z = min_index[2]; z < max_index[2]; ++z) {
                        IdxPoint voxel(x, y, z);

                        if (env_->IsObstacle(voxel) &&
                                InsideCylinder(env_->VoxelToWorld(voxel), shape->ShapeSegment(t, c))) {
                            return true;
                        }
                    }
                }
            }
        }
    }

    return false;
}

bool CRISPCollisionDetector::Collides(const std::shared_ptr<CRISPConfig> config) const {
    auto pointer = config.get();
    return Collides(*pointer);
}

std::vector<Vec3> CRISPCollisionDetector::InCollisionPoints(const CRISPConfig& config) const {
    std::vector<Vec3> points;
    auto shape = config.Shape();

    for (Idx t = 0; t < NUM_TUBES; ++t) {
        Idx num_cylinders = shape->NumSegments(t);

        for (Idx c = 0; c < num_cylinders; ++c) {
            geo::Cube cube = BoundingCube(shape->ShapeSegment(t, c));

            IdxPoint min_index = env_->WorldToVoxel(cube.low);
            IdxPoint max_index = env_->WorldToVoxel(cube.high);

            for (Idx x = min_index[0]; x < max_index[0]; ++x) {
                for (Idx y = min_index[1]; y < max_index[1]; ++y) {
                    for (Idx z = min_index[2]; z < max_index[2]; ++z) {
                        IdxPoint voxel(x, y, z);

                        if (env_->IsObstacle(voxel) &&
                                InsideCylinder(env_->VoxelToWorld(voxel), shape->ShapeSegment(t, c))) {
                            points.push_back(env_->VoxelToWorld(voxel));
                        }
                    }
                }
            }
        }
    }

    return points;
}

geo::Cube CRISPCollisionDetector::BoundingCube(const geo::Cylinder& cylinder) const {
    geo::Cube cube;

    for (Idx i = 0; i < 3; ++i) {
        cube.low[i] = fmin(cylinder.p1[i] - cylinder.r, cylinder.p2[i] - cylinder.r);
        cube.high[i] = fmax(cylinder.p1[i] + cylinder.r, cylinder.p2[i] + cylinder.r);
    }

    return cube;
}

bool CRISPCollisionDetector::InsideCylinder(const Vec3& p, const geo::Cylinder& cylinder) const {
    Vec3 line = cylinder.p2 - cylinder.p1;
    RealNum len = line.dot(p - cylinder.p1) / line.norm();
    Vec3 projection = len*(line.normalized());

    if (len > 0 && len < line.norm()) {
        return ( (cylinder.p1 + projection - p).norm() < cylinder.r );
    }

    return false;
}

std::vector<IdxPoint> CRISPCollisionDetector::VisiblePoints(const CRISPConfig& config,
        const RealNum fov_in_rad) {
    SetupSeenArray();

    std::vector<IdxPoint> points;
    Vec3 tip_dir = config.TipTangent().normalized();

    for (Idx i = 0; i < env_->NumTargets(); ++i) {
        IdxPoint p = env_->Target(i);

        if (!seen_[p[0]][p[1]][p[2]]) {
            Vec3 link_dir = (env_->VoxelToWorld(p) - config.TipTranslation()).normalized();

            if (acos(tip_dir.dot(link_dir)) < 0.5*fov_in_rad) {
                IdxPoint nearest_point = NearestVisiblePoint(config.TipTranslation(), link_dir);

                if (env_->IsTarget(nearest_point)
                        && !seen_[nearest_point[0]][nearest_point[1]][nearest_point[2]]) {

                    points.push_back(nearest_point);
                    seen_[nearest_point[0]][nearest_point[1]][nearest_point[2]] = true;
                }
            }
        }
    }

    return points;
}

std::vector<Idx> CRISPCollisionDetector::VisiblePointIndices(const CRISPConfig& config,
        const RealNum fov_in_rad) {
    SetupSeenArray();

    std::vector<Idx> points;
    Vec3 tip_dir = config.TipTangent().normalized();

    for (Idx i = 0; i < env_->NumTargets(); ++i) {
        IdxPoint p = env_->Target(i);

        if (!seen_[p[0]][p[1]][p[2]]) {
            Vec3 link_dir = (env_->VoxelToWorld(p) - config.TipTranslation()).normalized();

            if (acos(tip_dir.dot(link_dir)) < 0.5*fov_in_rad) {
                IdxPoint nearest_point = NearestVisiblePoint(config.TipTranslation(), link_dir);

                if (env_->IsTarget(nearest_point)
                        && !seen_[nearest_point[0]][nearest_point[1]][nearest_point[2]]) {

                    points.push_back(env_->TargetIndex(nearest_point));
                    seen_[nearest_point[0]][nearest_point[1]][nearest_point[2]] = true;
                }
            }
        }
    }

    return points;
}

IdxPoint CRISPCollisionDetector::NearestVisiblePoint(const Vec3& tip_trans, const Vec3& unit_dir,
        const RealNum upper_bound) const {
    RealNum min_dist = R_INF;
    Vec3 point;
    RealNum dist;

    // initialize p
    IdxPoint p = env_->WorldToVoxel(tip_trans);

    if (!env_->WithinRange(p)) {
        std::cerr << "Tip position is out of range!" << std::endl;
        exit(1);
    }

    Vec3 voxel_size = env_->VoxelSize();

    // go in x, y, z directions
    for (Idx d = 0; d < 3; ++d) {
        if (fabs(unit_dir[d]) > EPS) {
            point = tip_trans;
            dist = 0;

            // general step size
            int sign = (unit_dir[d] > 0)? 1 : -1;
            RealNum first_step;
            RealNum step = voxel_size[d] / fabs(unit_dir[d]);

            // special first step
            if (sign > 0) {
                first_step = (voxel_size[d] - fmod(point[d] + 0.5*voxel_size[d], voxel_size[d])) / unit_dir[d];
            }
            else {
                first_step = (-fmod(point[d] + 0.5*voxel_size[d], voxel_size[d])) / unit_dir[d];
            }

            point = point + first_step*unit_dir;
            dist += first_step;

            while(dist < min_dist && dist < upper_bound) {
                IdxPoint p_idx = env_->WorldToVoxel(point, d, sign);

                if (!env_->WithinRange(p_idx)) {
                    break;
                }

                if (env_->IsObstacle(p_idx)) {
                    if (dist < min_dist) {
                        min_dist = dist;
                        p = p_idx;
                        break;
                    }
                }

                point += step*unit_dir;
                dist += step;
            }
        }
    }

    return p;
}

void CRISPCollisionDetector::ComputeVisSetForConfiguration(const CRISPConfig& config,
        VisibilitySet* vis_set, const RealNum fov_in_rad) const {
    ComputeVisSetForConfiguration(config.TipTranslation(), config.TipTangent(), vis_set, fov_in_rad);
}

void CRISPCollisionDetector::ComputeVisSetForConfiguration(const Vec3& tip_trans,
        const Vec3& tip_tang, VisibilitySet* vis_set, const RealNum fov_in_rad) const {
    vis_set->Clear();

    Vec3 unit_tang = tip_tang.normalized();

    // examine all target points
    for (Idx i = 0; i < env_->NumTargets(); ++i) {
        IdxPoint p = env_->Target(i);

        if (!vis_set->At(i)) {
            Vec3 unit_dir = (env_->VoxelToWorld(p) - tip_trans).normalized();
            RealNum upper_bound = (env_->VoxelToWorld(p) - tip_trans).norm();

            // test if within fov
            if (acos(unit_tang.dot(unit_dir)) < 0.5*fov_in_rad) {
                // be caureful, p is changed
                p = NearestVisiblePoint(tip_trans, unit_dir, upper_bound);

                if ( env_->IsTarget(p) ) {
                    vis_set->Insert(env_->TargetIndex(p));
                }
            }
        }
    }
}

CRISPCollisionDetector::EnvPtr CRISPCollisionDetector::Environment() const {
    return env_;
}

}
