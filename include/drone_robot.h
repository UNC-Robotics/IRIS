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

#ifndef DRONE_ROBOT_H
#define DRONE_ROBOT_H

#include <cmath>

#include "geo_shapes.h"
#include "robot.h"

namespace drone {

const RealNum kMinYaw = -M_PI;
const RealNum kMaxYaw = M_PI;
const RealNum kMinCameraAngle = 0.0;
const RealNum kMaxCameraAngle = 0.5 * M_PI;

class DroneConfig {
  public:
    DroneConfig() = default;
    DroneConfig(const Vec3& pos, const RealNum yaw, const RealNum camera_angle);
    DroneConfig(const std::shared_ptr<DroneConfig>& other);
    ~DroneConfig() = default;

    Vec3 Position() const;
    Vec3& Position();
    RealNum Yaw() const;
    RealNum& Yaw();
    RealNum CameraAngle() const;
    RealNum& CameraAngle();

    bool ValidYaw(const RealNum yaw) const;
    bool ValidCameraAngle(const RealNum camera_angle) const;

    void Print(std::ostream& out) const;

  private:
    Vec3 pos_;
    RealNum yaw_;
    RealNum camera_angle_;
};

class DroneRobot : public Robot {
  public:
    DroneRobot(const RealNum height, const RealNum width, const RealNum camera_offset);
    ~DroneRobot() = default;

    void Initialize();
    void ComputeShape();

    void SetCameraParameters(const RealNum fov, const RealNum min_dof, const RealNum max_dof);

    RealNum Height() const;
    RealNum Width() const;
    RealNum SphereRadius() const;
    RealNum FOV() const;
    RealNum MinDOF() const;
    RealNum MaxDOF() const;
    std::shared_ptr<DroneConfig> Config() const;
    std::shared_ptr<DroneConfig> StartConfig() const;
    void SaveStartConfig();
    void SetConfig(const Vec3& pos, const RealNum yaw, const RealNum camera_angle);
    Vec3 CameraPos() const;
    Vec3 CameraTangent() const;

  private:
    RealNum height_;
    RealNum width_;
    RealNum camera_offset_;  // relative to robot center
    RealNum sphere_radius_;
    RealNum fov_{0.0};
    RealNum min_dof_{0.0};
    RealNum max_dof_{0.0};

    std::shared_ptr<DroneConfig> config_{nullptr};
    std::shared_ptr<DroneConfig> start_config_{nullptr};

    Vec3 camera_pos_;
    Vec3 camera_tang_;
};

}

#endif // DRONE_ROBOT_H
