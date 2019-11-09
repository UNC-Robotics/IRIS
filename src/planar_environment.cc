#include "planar_environment.h"

#include <iostream>
#include <random>

namespace planar {

PlanarEnvironment::PlanarEnvironment(const RealNum width, RealNum length, const Idx num_points_per_edge, Idx rand_seed):
    xmin_(0),
    xmax_(width),
    ymin_(0),
    ymax_(length) {

    srand(rand_seed);

    assert(width > 0 && length > 0);
    RealNum dx = width/num_points_per_edge;
    RealNum dy = length/num_points_per_edge;

    RealNum x, y;
    for (Idx i = 0; i < num_points_per_edge; ++i) {
        x = xmin_ + i*dx;
        y = ymin_ + i*dy;

        targets_.push_back(Point(x, ymin_));
        targets_.push_back(Point(x, ymax_));
        targets_.push_back(Point(xmin_, y));
        targets_.push_back(Point(xmax_, y));
    }
}

void PlanarEnvironment::RandomObstacles(const Idx num_rects,const RealNum max_size, const bool clear_previous_obstacles) {
    if (clear_previous_obstacles) {
        obstacles_.clear();
    }

    while (obstacles_.size() < num_rects) {
        RealNum width = RandomNum(0, max_size);
        RealNum length = RandomNum(0, max_size);
        RealNum x = RandomNum(xmin_ + 0.5*width, xmax_ - 0.5*width);
        RealNum y = RandomNum(ymin_ + 0.5*length, ymax_ - 0.5*length);

        obstacles_.emplace_back(Vec2(x, y), width, length);
    }
}

bool PlanarEnvironment::IsCollisionFree(const std::vector<Line> links, bool show_details) const {
    // Self collision.
    for (unsigned i = 0; i < links.size(); ++i) {
        auto l0 = links[i];

        // Environment bounds.
        if (std::min(l0.p1[0], l0.p2[0]) < xmin_ || std::max(l0.p1[0], l0.p2[0]) > xmax_ ||
            std::min(l0.p1[1], l0.p2[1]) < ymin_ || std::max(l0.p1[1], l0.p2[0]) > xmax_) {
            if (show_details) {
                std::cout << "Intersecting with environment boundary." << std::endl;
            }
            return false;
        }

        for (unsigned j = i + 2; j < links.size(); ++j) {
            auto l1 = links[j];
            if (Intersecting(Line(l0.p1, l0.p2), Line(l1.p1, l1.p2))) {
                if (show_details) {
                    std::cout << "Self collision." << std::endl;
                }
                return false;
            }
        }
    }

    // Obstacles.
    for (auto l : links) {
        for (auto obs: obstacles_) {
            // p1 is inside obstacle
            auto dist1 = l.p1 - obs.center;
            if (std::abs(dist1[0]) < 0.5*obs.width && std::abs(dist1[1]) < 0.5*obs.length) {
                if (show_details) {
                    std::cout << "p1" << std::endl;
                }
                return false;
            }

            // p2 is inside obstacle
            auto dist2 = l.p2 - obs.center;
            if (std::abs(dist2[0]) < 0.5*obs.width && std::abs(dist2[1]) < 0.5*obs.length) {
                if (show_details) {
                    std::cout << "p2" << std::endl;
                }
                return false;
            }

            // link intersects rectangle
            if (Intersecting(l, Line(obs.UpperLeft(), obs.UpperRight()))) {
                if (show_details) {
                    std::cout << "1" << std::endl;
                }
                return false;
            }
            if (Intersecting(l, Line(obs.UpperLeft(), obs.LowerLeft()))) {
                if (show_details) {
                    std::cout << "2" << std::endl;
                }
                return false;
            }
            if (Intersecting(l, Line(obs.UpperRight(), obs.LowerRight()))) {
                if (show_details) {
                    std::cout << "3" << std::endl;
                }
                return false;
            }
            if (Intersecting(l, Line(obs.LowerLeft(), obs.LowerRight()))) {
                if (show_details) {
                    std::cout << "4" << std::endl;
                }
                return false;
            }
        }
    }


    return true;
}

bool PlanarEnvironment::IsCollisionFree(const std::vector<Vec2> shape, bool show_details) const {
    std::vector<Line> links;
    for (Idx i = 0; i < shape.size() - 1; ++i) {
        links.emplace_back(shape[i], shape[i + 1]);
    }

    return IsCollisionFree(links, show_details);
}

SizeType PlanarEnvironment::NumTargets() const {
    return targets_.size();
}

std::vector<Point> PlanarEnvironment::GetVisiblePoints(const std::vector<Vec2>& shape, const RealNum fov_in_rad) const {
    auto dim = shape.size() - 1;
    Vec2 tip_pos = shape[dim];
    Vec2 tip_tang = shape[dim] - shape[dim - 1];

    std::vector<Point> visible_points;

    for (Idx i = 0; i < targets_.size(); ++i) {
        auto p = targets_[i];
        auto tip_to_point = p - tip_pos;
        auto angle = std::acos(tip_to_point.normalized().dot(tip_tang.normalized()));

        if (angle < 0.5*fov_in_rad) {
            bool visible = true;
            auto vis_line = Line(tip_pos, p);

            // Self occculusion.
            for (Idx j = 0; j < dim - 1; ++j) {
                if (Intersecting(vis_line, Line(shape[j], shape[j+1]))) {
                    visible = false;
                    break;
                }
            }

            if (!visible) {
                continue;
            }

            for (auto&& r : obstacles_) {
                if (Intersecting(vis_line, Line(r.LowerLeft(), r.LowerRight()))
                    || Intersecting(vis_line, Line(r.UpperLeft(), r.UpperRight()))
                    || Intersecting(vis_line, Line(r.LowerLeft(), r.UpperLeft()))
                    || Intersecting(vis_line, Line(r.LowerRight(), r.UpperRight()))) 
                {
                    visible = false;
                    break;
                }
            }

            if (visible) {
                visible_points.push_back(p);
            }
        }
    }

    return visible_points;
}

std::vector<Idx> PlanarEnvironment::GetVisiblePointIndices(const std::vector<Vec2>& shape, const RealNum fov_in_rad) const {
    auto dim = shape.size() - 1;
    Vec2 tip_pos = shape[dim];
    Vec2 tip_tang = shape[dim] - shape[dim - 1];

    std::vector<Idx> visible_points;

    for (Idx i = 0; i < targets_.size(); ++i) {
        auto p = targets_[i];
        auto tip_to_point = p - tip_pos;
        auto angle = std::acos(tip_to_point.normalized().dot(tip_tang.normalized()));

        if (angle < 0.5*fov_in_rad) {
            bool visible = true;
            auto vis_line = Line(tip_pos, p);

            // Self occculusion.
            for (Idx j = 0; j < dim - 1; ++j) {
                if (Intersecting(vis_line, Line(shape[j], shape[j+1]))) {
                    visible = false;
                    break;
                }
            }

            if (!visible) {
                continue;
            }

            for (auto&& r : obstacles_) {
                if (Intersecting(vis_line, Line(r.LowerLeft(), r.LowerRight()))
                    || Intersecting(vis_line, Line(r.UpperLeft(), r.UpperRight()))
                    || Intersecting(vis_line, Line(r.LowerLeft(), r.UpperLeft()))
                    || Intersecting(vis_line, Line(r.LowerRight(), r.UpperRight()))) 
                {
                    visible = false;
                    break;
                }
            }

            if (visible) {
                visible_points.push_back(i);
            }
        }
    }

    return visible_points;
}

RealNum PlanarEnvironment::RandomNum(const RealNum min, const RealNum max) const {
    return min + ((RealNum)rand()/RAND_MAX) * (max - min);
}


bool PlanarEnvironment::Intersecting(const Line& l1, const Line& l2) const {
    RealNum q = (l1.p1[1] - l2.p1[1])*(l2.p2[0] - l2.p1[0]) - (l1.p1[0] - l2.p1[0])*(l2.p2[1] - l2.p1[1]);
    RealNum d = (l1.p2[0] - l1.p1[0])*(l2.p2[1] - l2.p1[1]) - (l1.p2[1] - l1.p1[1])*(l2.p2[0] - l2.p1[0]);

    if (std::abs(d) < EPS) {
        return false;
    }

    RealNum r = q/d;
    RealNum s = ((l1.p1[1] - l2.p1[1])*(l1.p2[0] - l1.p1[0]) - (l1.p1[0] - l2.p1[0])*(l1.p2[1] - l1.p1[1]))/d;

    if ( r < 0 || r > 1 || s < 0 || s > 1 ) {
        return false;
    }

    return true;
}

}