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

#ifndef PLANAR_ENVIRONMENT_
#define PLANAR_ENVIRONMENT_

#include <vector>

#include "geo_shapes.h"

namespace planar {

using Point = Vec2;
using Line = geo::Line2D;
using Rectangle = geo::Rectangle;

class PlanarEnvironment {
  public:
    PlanarEnvironment(const RealNum width, RealNum length, const Idx num_points_per_edge=100,
                      Idx rand_seed=1);
    ~PlanarEnvironment() = default;

    void RandomObstacles(const Idx num_rects,const RealNum max_size,
                         const bool clear_previous_obstacles=true);
    bool IsCollisionFree(const std::vector<Line> links, bool show_details=false) const;
    bool IsCollisionFree(const std::vector<Vec2> shape, bool show_details=false) const;
    SizeType NumTargets() const;
    std::vector<Point> GetVisiblePoints(const std::vector<Vec2>& shape, const RealNum fov_in_rad) const;
    std::vector<Idx> GetVisiblePointIndices(const std::vector<Vec2>& shape,
                                            const RealNum fov_in_rad) const;

  private:
    RealNum xmin_, xmax_, ymin_, ymax_;
    std::vector<Rectangle> obstacles_;
    std::vector<Point> targets_;
    RealNum RandomNum(const RealNum min, const RealNum max) const;
    bool Intersecting(const Line& l1, const Line& l2) const;

};

}

#endif // PLANAR_ENVIRONMENT_