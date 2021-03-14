#include "ct_anatomy.h"

namespace crisp {

CTAnatomy::CTAnatomy(const String file_env)
    : file_env_(file_env) {

    ReadEnvFile();
}

CTAnatomy::~CTAnatomy() {
    obstacle_mask_.resize(boost::extents[0][0][0]);
    target_mask_.resize(boost::extents[0][0][0]);
}

void CTAnatomy::SetFreePoint(const IdxPoint& p) {
    free_point_ = p;
}

void CTAnatomy::SetTargetPoints(const String file_target_points) {
    file_target_points_ = file_target_points;
    target_mask_.resize(boost::extents[index_range_[0]][index_range_[1]][index_range_[2]]);
    this->ClearTargets();

    if (!ReadTargetFile()) {
        this->GenerateSurfacePointsAsTargets();
        WriteTargetFile();
    }

    std::cout << "Totally " << this->NumTargets() << " targets to inspect." << std::endl;
    targets_initialized_ = true;
}

void CTAnatomy::ReadEnvFile() {
    std::ifstream fin;
    fin.open(file_env_);

    if (!fin.is_open()) {
        std::cerr << "File not opened: " << file_env_ << std::endl;
        exit(1);
    }

    String str;
    std::getline(fin, str);
    std::istringstream s1(str);
    RealNum x, y, z;
    s1 >> x >> y >> z;
    voxel_size_ << x/1000, y/1000, z/1000;

    std::cout << "Voxel size " << voxel_size_.transpose() << std::endl;

    std::getline(fin, str);
    std::istringstream s2(str);
    Idx i, j, k;
    s2 >> i >> j >> k;
    index_range_ << i, j, k;

    std::cout << "Space size " << index_range_.transpose() << std::endl;

    world_origin_ << 0.0, 0.0, 0.0;

    std::getline(fin, str);
    std::istringstream stream(str);

    obstacle_mask_.resize(boost::extents[i][j][k]);
    // target_mask_.resize(boost::extents[i][j][k]);

    this->ClearObstacles();
    /// this->ClearTargets();

    SizeType count = 0;
    bool occupied;

    for (Idx x = 0; x < i; ++x) {
        for (Idx y = 0; y < j; ++y) {
            for (Idx z = 0; z < k; ++z) {
                stream >> occupied;

                if (occupied) {
                    AddObstacle(x, y, z);
                    count++;
                }
            }
        }
    }

    fin.close();
    std::cout << "File read: " << file_env_ << std::endl;

    std::cout << "Totally " << count << " obstacles." << std::endl;
}

void CTAnatomy::ClearObstacles() {
    for (Idx x = 0; x < index_range_[0]; ++x) {
        for (Idx y = 0; y < index_range_[1]; ++y) {
            for (Idx z = 0; z < index_range_[2]; ++z) {
                obstacle_mask_[x][y][z] = false;
            }
        }
    }
}

void CTAnatomy::ClearTargets() {
    for (Idx x = 0; x < index_range_[0]; ++x) {
        for (Idx y = 0; y < index_range_[1]; ++y) {
            for (Idx z = 0; z < index_range_[2]; ++z) {
                target_mask_[x][y][z] = 0;
            }
        }
    }
}

void CTAnatomy::AddObstacle(Idx x, Idx y, Idx z) {
    if (!obstacle_mask_[x][y][z]) {
        obstacle_mask_[x][y][z] = true;
    }
}

void CTAnatomy::AddObstacle(const IdxPoint& p) {
    this->AddObstacle(p[0], p[1], p[2]);
}

bool CTAnatomy::RemoveObstacle(Idx x, Idx y, Idx z) {
    if (obstacle_mask_[x][y][z]) {
        obstacle_mask_[x][y][z] = false;
        return true;
    }

    return false;
}

bool CTAnatomy::RemoveObstacle(const IdxPoint& p) {
    return this->RemoveObstacle(p[0], p[1], p[2]);
}

void CTAnatomy::AddTarget(Idx x, Idx y, Idx z) {
    if (target_mask_[x][y][z] == 0) {
        target_mask_[x][y][z] = targets_.size() + 1;
        targets_.push_back(IdxPoint(x,y,z));
    }
}

void CTAnatomy::AddTarget(const IdxPoint& p) {
    this->AddTarget(p[0], p[1], p[2]);
}

bool CTAnatomy::RemoveTarget(const Idx x, const Idx y, const Idx z) {
    if (IsTarget(x, y, z)) {
        Idx index = TargetIndex(x, y, z);
        IdxPoint p = targets_[index];

        if (p[0] != x || p[1] != y || p[2] != z) {
            std::cerr << "Target at index " << index << " is not (" << p.transpose() << ")" << std::endl;
            exit(1);
        }

        target_mask_[x][y][z] = 0;

        if (index > 0 && index < targets_.size()) {
            targets_.erase(targets_.begin() + index);

            for (Idx x = 0; x < index_range_[0]; ++x) {
                for (Idx y = 0; y < index_range_[1]; ++y) {
                    for (Idx z = 0; z < index_range_[2]; ++z) {
                        if (IsTarget(x, y, z) && TargetIndex(x, y, z) > index) {
                            target_mask_[x][y][z]--;
                        }
                    }
                }
            }

            return true;
        }
        else {
            std::cerr << "Target vector and mask are inconsistent!" << std::endl;
            exit(1);
        }
    }

    return false;
}

bool CTAnatomy::RemoveTarget(const IdxPoint& p) {
    return this->RemoveTarget(p[0], p[1], p[2]);
}

std::vector<IdxPoint> CTAnatomy::AllTargets() const {
    return targets_;
}

IdxPoint CTAnatomy::Target(Idx i) const {
    if (i < targets_.size()) {
        IdxPoint p = targets_[i];

        if (IsTarget(p)) {
            return p;
        }

        std::cerr << "Target vector and mask are inconsistent!" << std::endl;
        exit(1);
    }

    std::cerr << "Exceeding target index!" << std::endl;
    exit(1);
}

bool CTAnatomy::IsObstacle(const IdxPoint& p) const {
    return obstacle_mask_[p[0]][p[1]][p[2]];
}

bool CTAnatomy::IsTarget(const IdxPoint& p) const {
    return target_mask_[p[0]][p[1]][p[2]] > 0;
}

bool CTAnatomy::IsTarget(const Idx x, const Idx y, const Idx z) const {
    return target_mask_[x][y][z] > 0;
}

Idx CTAnatomy::TargetIndex(const IdxPoint& p) const {
    return TargetIndex(p[0], p[1], p[2]);
}

Idx CTAnatomy::TargetIndex(const Idx x, const Idx y, const Idx z) const {
    if (target_mask_[x][y][z] > 0) {
        return target_mask_[x][y][z] - 1;
    }

    std::cerr << "Not a valid target!" << std::endl;
    exit(1);
}

IdxPoint CTAnatomy::IndexRanges() const {
    return index_range_;
}

Vec3 CTAnatomy::VoxelSize() const {
    return voxel_size_;
}

SizeType CTAnatomy::NumTargets() const {
    return targets_.size();
}

bool CTAnatomy::ReadTargetFile() {
    std::ifstream fin;
    fin.open(file_target_points_);

    if (!fin.is_open()) {
        std::cout << "File not exist: " << file_target_points_ << std::endl;
        return false;
    }

    String line;

    while(std::getline(fin, line)) {
        std::istringstream ss(line);
        Idx x, y, z;
        ss >> x >> y >> z;

        this->AddTarget(x, y, z);
    }

    fin.close();
    std::cout << "File read: " << file_target_points_ << std::endl;

    return true;
}

void CTAnatomy::WriteTargetFile() const {
    std::ofstream fout;
    fout.open(file_target_points_);

    if (!fout.is_open()) {
        std::cerr << "File not opened: " << file_target_points_ << std::endl;
        exit(1);
    }

    for (IdxPoint p : targets_) {
        fout << p[0] << " "
             << p[1] << " "
             << p[2] << " "
             << std::endl;
    }

    fout.close();
    std::cout << "File written: " << file_target_points_ << std::endl;
}

void CTAnatomy::RemoveSmallConnectedComponents(const SizeType size) {
    BoolArray3 mask(obstacle_mask_);

    for (Idx x = 0; x < index_range_[0]; ++x) {
        for (Idx y = 0; y < index_range_[1]; ++y) {
            for (Idx z = 0; z < index_range_[2]; ++z) {
                if (mask[x][y][z]) {
                    IdxPoint p (x, y, z);
                    std::vector<IdxPoint> connected_component = ConnectedComponent(p, &mask);

                    if (connected_component.size() > size) {
                        continue;
                    }

                    for (IdxPoint p : connected_component) {
                        this->RemoveObstacle(p);

                        if (targets_initialized_) {
                            this->RemoveTarget(p);
                        }
                    }
                }
            }
        }
    }

    std::cout << "Removed small connected components under size " << size  << "." << std::endl;
}

std::vector<IdxPoint> CTAnatomy::ConnectedComponent(const IdxPoint& source,
        BoolArray3* mask) const {
    std::vector<IdxPoint> cp;

    std::stack<IdxPoint> s;
    s.push(source);

    while (!s.empty()) {
        IdxPoint p = s.top();
        s.pop();
        cp.push_back(p);

        auto neighbors = ValidNeighbors(p);

        for (IdxPoint n : neighbors) {
            if ((*mask)[n[0]][n[1]][n[2]]) {
                s.push(n);
                (*mask)[n[0]][n[1]][n[2]] = false;
            }
        }
    }

    return cp;
}

std::vector<IdxPoint> CTAnatomy::ValidNeighbors(const IdxPoint& p) const {
    std::vector<IdxPoint> neighbors;

    IdxPoint neig;

    if (p[0] > 0) {
        neig << p[0] - 1, p[1], p[2];
        neighbors.push_back(neig);
    }

    if (p[0] < index_range_[0] - 1) {
        neig << p[0] + 1, p[1], p[2];
        neighbors.push_back(neig);
    }

    if (p[1] > 0) {
        neig << p[0], p[1] - 1, p[2];
        neighbors.push_back(neig);
    }

    if (p[1] < index_range_[1] - 1) {
        neig << p[0], p[1] + 1, p[2];
        neighbors.push_back(neig);
    }

    if (p[2] > 0) {
        neig << p[0], p[1], p[2] - 1;
        neighbors.push_back(neig);
    }

    if (p[2] < index_range_[2] - 1) {
        neig << p[0], p[1], p[2] + 1;
        neighbors.push_back(neig);
    }

    return neighbors;
}

void CTAnatomy::GenerateSurfacePointsAsTargets() {
    BoolArray3 processed(boost::extents[index_range_[0]][index_range_[1]][index_range_[2]]);

    std::stack<IdxPoint> s;
    s.push(free_point_);

    while(!s.empty()) {
        IdxPoint p = s.top();
        s.pop();

        std::vector<IdxPoint> neighbors = ValidNeighbors(p);

        for (IdxPoint n : neighbors) {
            if (!processed[n[0]][n[1]][n[2]]) {
                if (IsObstacle(n)) {
                    AddTarget(n);
                }
                else {
                    s.push(n);
                }

                processed[n[0]][n[1]][n[2]] = true;
            }
        }
    }
}

Vec3 CTAnatomy::VoxelToWorld(const IdxPoint& p) const {
    // assume (0,0,0) point in voxel space centers at world_origin_
    return Vec3(p[0]*voxel_size_[0] + world_origin_[0],
                p[1]*voxel_size_[1] + world_origin_[1],
                p[2]*voxel_size_[2] + world_origin_[2]);
}

IdxPoint CTAnatomy::WorldToVoxel(const Vec3& p, Idx dimension, int sign) const {
    Vec3 p_tmp = p;

    if (sign != 0) {
        p_tmp[dimension] += 0.5*sign * voxel_size_[dimension];
    }

    IdxPoint p_return;

    for (Idx i = 0; i < 3; ++i) {
        p_return[i] = floor( (p_tmp[i] + 0.5*voxel_size_[i]) / voxel_size_[i] );
    }

    return p_return;
}

Idx CTAnatomy::CleanUpRegion(const Vec3& center, const Idx radius) {
    Idx count = 0;

    IdxPoint center_voxel = WorldToVoxel(center);

    IdxPoint min_bound = center_voxel;
    IdxPoint max_bound = center_voxel;

    for (Idx i = 0; i < 3; ++i) {
        min_bound[i] -= radius;
        min_bound[i] = (min_bound[i] < 0)? 0 : min_bound[i];
        max_bound[i] += radius;
        max_bound[i] = (max_bound[i] > index_range_[i] - 1)? (index_range_[i] - 1) : max_bound[i];
    }

    for (Idx x = min_bound[0]; x <= max_bound[0]; ++x) {
        for (Idx y = min_bound[1]; y <= max_bound[1]; ++y) {
            for (Idx z = min_bound[2]; z <= max_bound[2]; ++z) {
                bool is_obstacle = this->RemoveObstacle(x, y, z);

                if (targets_initialized_ && IsTarget(x, y, z)) {
                    this->RemoveTarget(x, y, z);
                }

                if (is_obstacle) {
                    count++;
                }
            }
        }
    }

    std::cout << "Cleaned up region around " << center.transpose() << ", removed " << count <<
              " obstacles" << std::endl;

    if (targets_initialized_) {
        std::cout << "Remaining target points: " << targets_.size() << std::endl;
    }

    return count;
}

bool CTAnatomy::WithinRange(const IdxPoint& p) const {
    for (Idx i = 0; i < 3; ++i) {
        if (p[i] < 0 || p[i] > index_range_[i] - 1) {
            return false;
        }
    }

    return true;
}

}