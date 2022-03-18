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
#include <cmath>

#include "planar_robot.h"

namespace planar {

PlanarRobot::PlanarRobot(const Vec2& origin, const std::vector<RealNum>& links,
                         const std::vector<Vec2>& bounds)
    : origin_(origin), links_(links), bounds_(bounds) {
    if (links.size() != bounds.size()) {
        std::cerr << "Link lengths and angle bounds must have the same size." << std::endl;
        exit(1);
    }

    num_links_ = links.size();
}

void PlanarRobot::Initialize() {
    if (fov_ < 1e-6) {
        std::cerr << "Field of view is not properly set." << std::endl;
        exit(1);
    }
    else {
        std::cout << "Planar robot initialized." << std::endl;
    }
}

void PlanarRobot::ComputeShape() {
    if (config_.size() != num_links_) {
        std::cerr << "Incorrect DOF." << std::endl;
        exit(1);
    }

    shape_.clear();
    shape_.push_back(origin_);

    RealNum phi, x, y;
    phi = 0;
    x = origin_[0];
    y = origin_[1];

    for (Idx i = 0; i < num_links_; ++i) {
        phi += config_[i];
        x += links_[i] * std::cos(phi);
        y += links_[i] * std::sin(phi);

        shape_.emplace_back(x, y);
    }
}

void PlanarRobot::SetCameraFOV(const RealNum fov) {
    fov_ = fov;
}

Vec2 PlanarRobot::Origin() const {
    return origin_;
}

Vec2& PlanarRobot::Origin() {
    return origin_;
}

RealNum PlanarRobot::LinkLength(Idx i) const {
    if (i >= num_links_) {
        std::cerr << "Exceeding maximum number of links." << std::endl;
        exit(1);
    }

    return links_[i];
}

Vec2 PlanarRobot::Bounds(Idx i) const {
    if (i >= num_links_) {
        std::cerr << "Exceeding maximum number of links." << std::endl;
        exit(1);
    }

    return bounds_[i];
}

Idx PlanarRobot::NumLinks() const {
    return num_links_;
}

std::vector<Vec2> PlanarRobot::Shape() const {
    return shape_;
}

RealNum PlanarRobot::FOV() const {
    return fov_;
}


void PlanarRobot::SetConfig(const std::vector<RealNum>& config) {
    config_ = config;
}

void PlanarRobot::SaveStartConfig() {
    start_config_ = config_;
}

std::vector<RealNum> PlanarRobot::Config() const {
    return config_;
}

std::vector<RealNum> PlanarRobot::StartConfig() const {
    return start_config_;
}

void PlanarRobot::PrintConfig() const {
    for (Idx i = 0; i < num_links_; ++i) {
        std::cout << config_[i] << " ";
    }

    std::cout << std::endl;
}


}