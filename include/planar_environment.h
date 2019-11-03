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
    PlanarEnvironment(const RealNum width, RealNum length, const Idx num_points_per_edge=100, Idx rand_seed=1);
    ~PlanarEnvironment() = default;
    
    void RandomObstacles(const Idx num_rects,const RealNum max_size, const bool clear_previous_obstacles=true);
    bool IsCollisionFree(const std::vector<Line> links, bool show_details=false) const;
    bool IsCollisionFree(const std::vector<Vec2> shape, bool show_details=false) const;
    SizeType NumTargets() const;
    std::vector<Point> GetVisiblePoints(const std::vector<Vec2>& shape, const RealNum fov_in_rad) const;
    std::vector<Idx> GetVisiblePointIndices(const std::vector<Vec2>& shape, const RealNum fov_in_rad) const;

private:
	RealNum xmin_, xmax_, ymin_, ymax_;
    std::vector<Rectangle> obstacles_;
    std::vector<Point> targets_;
    RealNum RandomNum(const RealNum min, const RealNum max) const;
    bool Intersecting(const Line& l1, const Line& l2) const;

};

}

#endif // PLANAR_ENVIRONMENT_