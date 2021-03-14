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
