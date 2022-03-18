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
