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

void CrispShape::AddCylinder(const Idx tube_index, const Vec3& p1, const Vec3& p2, const RealNum r) {
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
