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

#include "drone_robot.h"

namespace drone {

DroneConfig::DroneConfig(const Vec3& pos, const RealNum yaw, const RealNum camera_angle) :
    pos_(pos), yaw_(yaw), camera_angle_(camera_angle) {

}

DroneConfig::DroneConfig(const std::shared_ptr<DroneConfig>& other) {
    pos_ = other->Position();
    yaw_ = other->Yaw();
    camera_angle_ = other->CameraAngle();
}

Vec3 DroneConfig::Position() const {
    return pos_;
}

Vec3& DroneConfig::Position() {
    return pos_;
}

RealNum DroneConfig::Yaw() const {
    return yaw_;
}

RealNum& DroneConfig::Yaw() {
    return yaw_;
}

RealNum DroneConfig::CameraAngle() const {
    return camera_angle_;
}

RealNum& DroneConfig::CameraAngle() {
    return camera_angle_;
}

bool DroneConfig::ValidYaw(const RealNum yaw) const {
    if (yaw > kMinYaw && yaw < kMaxYaw) {
        return true;
    }

    return false;
}

bool DroneConfig::ValidCameraAngle(const RealNum camera_angle) const {
    if (camera_angle > kMinCameraAngle && camera_angle < kMaxCameraAngle ) {
        return true;
    }

    return false;
}

void DroneConfig::Print(std::ostream& out) const {
    out << pos_.transpose() << ", ";
    out << yaw_ << ", " << camera_angle_ << std::endl;
}

DroneRobot::DroneRobot(const RealNum height, const RealNum width, const RealNum camera_offset) :
    height_(height), width_(width), camera_offset_(camera_offset) {
    auto half_height = 0.5*height;
    auto half_width = 0.5*width;
    sphere_radius_ = fmax(sqrt(half_width*half_width + half_width*half_width + half_height*half_height),
                          camera_offset);
}

void DroneRobot::Initialize() {
    if (fov_ < 1e-6 || max_dof_ < 1e-6) {
        std::cerr << "Field of view and/or depth of field is not properly set." << std::endl;
        exit(1);
    }
    else {
        std::cout << "Drone robot initialized." << std::endl;
    }
}

void DroneRobot::ComputeShape() {
    if (!config_) {
        std::cerr << "No valid configuration!" << std::endl;
        exit(1);
    }

    camera_pos_ = config_->Position();
    camera_pos_[2] += camera_offset_;

    auto yaw_mat = Eigen::AngleAxis<RealNum>(config_->Yaw(), Vec3::UnitZ());
    auto camera_mat = Eigen::AngleAxis<RealNum>(config_->CameraAngle(), Vec3::UnitY());
    camera_tang_ = yaw_mat * camera_mat * Vec3::UnitX();
}

void DroneRobot::SetCameraParameters(const RealNum fov, const RealNum min_dof,
                                     const RealNum max_dof) {
    fov_ = fov;
    min_dof_ = min_dof;
    max_dof_ = max_dof;
}

RealNum DroneRobot::Height() const {
    return height_;
}

RealNum DroneRobot::Width() const {
    return width_;
}

RealNum DroneRobot::SphereRadius() const {
    return sphere_radius_;
}

RealNum DroneRobot::FOV() const {
    return fov_;
}

RealNum DroneRobot::MinDOF() const {
    return min_dof_;
}

RealNum DroneRobot::MaxDOF() const {
    return max_dof_;
}

std::shared_ptr<DroneConfig> DroneRobot::Config() const {
    return config_;
}

std::shared_ptr<DroneConfig> DroneRobot::StartConfig() const {
    return start_config_;
}

void DroneRobot::SaveStartConfig() {
    start_config_.reset(new DroneConfig(config_));
}

void DroneRobot::SetConfig(const Vec3& pos, const RealNum yaw, const RealNum camera_angle) {
    if (!config_) {
        config_.reset(new DroneConfig(pos, yaw, camera_angle));
    }
    else {
        config_->Position() = pos;
        config_->Yaw() = yaw;
        config_->CameraAngle() = camera_angle;
    }
}

Vec3 DroneRobot::CameraPos() const {
    return camera_pos_;
}

Vec3 DroneRobot::CameraTangent() const {
    return camera_tang_;
}

}

