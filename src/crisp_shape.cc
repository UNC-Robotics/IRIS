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

#include "crisp_shape.h"

CrispShape::CrispShape(const Idx n) {
    points_.resize(n);
    radii_.resize(n);
}

void CrispShape::CheckIndex(const Idx tube_index) const {
    if (tube_index > radii_.size() - 1) {
        std::cerr << "Exceeding tube index!" << std::endl;
        exit(1);
    }
}

void CrispShape::SetRadius(const Idx tube_index, const RealNum r) {
    CheckIndex(tube_index);

    radii_[tube_index] = r;
}

void CrispShape::AddCenterPoint(const Idx tube_index, const Vec3& p) {
    CheckIndex(tube_index);

    points_[tube_index].push_back(p);
}

void CrispShape::AddCylinder(const Idx tube_index, const Vec3& p1, const Vec3& p2,
                             const RealNum r) {
    CheckIndex(tube_index);

    SizeType n = points_[tube_index].size();

    if (n == 0) {
        points_[tube_index].push_back(p1);
        points_[tube_index].push_back(p2);
        SetRadius(tube_index, r);
    }
    else {
        if (points_[tube_index][n-1][0] != p1[0]
                || points_[tube_index][n-1][1] != p1[1]
                || points_[tube_index][n-1][2] != p1[2]) {
            std::cerr << "Cylinders are not connected!" << std::endl;
            exit(1);
        }

        points_[tube_index].push_back(p2);

        if (abs(r - radii_[tube_index]) > EPS) {
            std::cerr << "Tube radius changed!" << std::endl;
        }
    }
}

geo::Cylinder CrispShape::GetCylinder(const Idx tube_index, const Idx cylinder_index) const {
    CheckIndex(tube_index);

    if (points_[tube_index].size() > 0 && cylinder_index < points_[tube_index].size()-1) {
        geo::Cylinder cyl;
        cyl.p1 = points_[tube_index][cylinder_index];
        cyl.p2 = points_[tube_index][cylinder_index+1];
        cyl.r = radii_[tube_index];
        return cyl;
    }

    std::cerr << "Exceeding cylinder index!" << std::endl;
    exit(1);
}

SizeType CrispShape::NumTubes() const {
    return radii_.size();
}

SizeType CrispShape::NumCylinders(const Idx tube_index) const {
    CheckIndex(tube_index);

    SizeType n = points_[tube_index].size();

    if (n > 0) {
        return n - 1;
    }

    return 0;
}
