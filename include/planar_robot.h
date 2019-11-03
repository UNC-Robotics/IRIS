#ifndef PLANAR_ROBOT_H
#define PLANAR_ROBOT_H

#include "geo_shapes.h"
#include "robot.h"
#include "planar_environment.h"

namespace planar {

class PlanarRobot : public Robot {
public:
    PlanarRobot(const Vec2& origin, const std::vector<RealNum>& links, const std::vector<Vec2>& bounds);
    ~PlanarRobot() = default;

    void Initialize();
    void ComputeShape();

    void SetCameraFOV(const RealNum fov);

    Vec2 Origin() const;
    Vec2& Origin();
    RealNum LinkLength(Idx i) const;
    Vec2 Bounds(Idx i) const;
    Idx NumLinks() const;
    std::vector<Vec2> Shape() const;
    RealNum FOV() const;

    void SetConfig(const std::vector<RealNum>& config);
    void SaveStartConfig();
    std::vector<RealNum> Config() const;
    std::vector<RealNum> StartConfig() const;
    void PrintConfig() const;

private:
    Vec2 origin_;
    Idx num_links_;
    RealNum fov_{0.0};
    std::vector<RealNum> links_;
    std::vector<Vec2> bounds_;
    std::vector<RealNum> config_;
    std::vector<RealNum> start_config_;
    std::vector<Vec2> shape_;
};

}

#endif // PLANAR_ROBOT_H
