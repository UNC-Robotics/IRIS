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

#ifndef GEO_SHAPES_H_
#define GEO_SHAPES_H_

#include "global_common.h"

namespace geo {
struct Cylinder {
    Cylinder() = default;
    Cylinder(Vec3 start, Vec3 end, RealNum radius) {
        p1 = start;
        p2 = end;
        r = radius;
    }
    Cylinder(const Cylinder& other) {
        this->p1 = other.p1;
        this->p2 = other.p2;
        this->r = other.r;
    }

    Vec3 p1, p2;
    RealNum r;
};

struct Cube {
    Vec3 low, high;
};

struct Line2D {
    Vec2 p1, p2;

    Line2D(const Vec2& point1, const Vec2& point2) {
        p1 = point1;
        p2 = point2;
    }
};

struct Rectangle {
    Vec2 center;
    RealNum width, length;

    Rectangle(const Vec2& c, const RealNum w, const RealNum l) {
        center = c;
        width = w;
        length = l;
    }

    Vec2 LowerLeft() const {
        return center + 0.5*Vec2(-width, -length);
    }

    Vec2 UpperLeft() const {
        return center + 0.5*Vec2(-width, length);
    }

    Vec2 LowerRight() const {
        return center + 0.5*Vec2(width, -length);
    }

    Vec2 UpperRight() const {
        return center + 0.5*Vec2(width, length);
    }
};
}

#endif // GEO_SHAPES_H_
