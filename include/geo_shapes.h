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
