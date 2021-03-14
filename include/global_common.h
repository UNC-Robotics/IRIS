#ifndef GLOBAL_COMMON_H_
#define GLOBAL_COMMON_H_

#include <chrono>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <boost/multi_array.hpp>
#include <Eigen/Dense>

#define DEBUG_MODE 0

#define REJECT_SAMPLING 1

#define USE_NODE_REUSE 1
#define KEEP_SUBSUMING_HISTORY 1
#define SAVE_PREDECESSOR 1

#define USE_CRISP 1
#define USE_PLANAR 0

#define USE_GHOST_DATA 1

#if USE_GHOST_DATA
#define USE_GHOST_COST_AS_KEY 1
#else
#define USE_GHOST_COST_AS_KEY 0
#endif

#define USE_HEURISTIC 0

#if USE_CRISP
#define DOWN_SAMPLING 1

#if DOWN_SAMPLING
#define MAX_COVERAGE_SIZE 4950
#else
#define MAX_COVERAGE_SIZE 49506 // 42039
#endif

#else

#define DOWN_SAMPLING 0

#if USE_PLANAR
#define MAX_COVERAGE_SIZE 400
#else
#define MAX_COVERAGE_SIZE 3817 // 14021 // 27384 // 3346
#endif // USE_PLANAR

#endif // USE_CRISP

#if USE_GHOST_DATA
#define P_DEFAULT 1.0
#define E_DEFUALT 0.95
#endif // USE_GHOST_DATA

#if USE_HEURISTIC
#define HEUR_BIAS 3.5
#define HEUR_PORTION 0.1
#endif // USE_HEURISTIC

// basic alias
using String = std::string;
using Idx = unsigned short;
using SizeType = std::size_t;
using RealNum = float;
using Rand = std::mt19937_64;
using RealUniformDist = std::uniform_real_distribution<RealNum>;
using RealNormalDist = std::normal_distribution<RealNum>;
using IntUniformDist = std::uniform_int_distribution<Idx>;

// Eigen alias
using IdxPoint = Eigen::Matrix<Idx, 3, 1, Eigen::ColMajor>;
using Vec2 = Eigen::Matrix<RealNum, 2, 1, Eigen::ColMajor>;
using Vec3 = Eigen::Matrix<RealNum, 3, 1, Eigen::ColMajor>;
using Vec4 = Eigen::Matrix<RealNum, 4, 1, Eigen::ColMajor>;
using Mat3 = Eigen::Matrix<RealNum, 3, 3, Eigen::ColMajor>;
using Mat4 = Eigen::Matrix<RealNum, 4, 4, Eigen::ColMajor>;
using Affine = Eigen::Transform<RealNum, 3, Eigen::Affine, Eigen::ColMajor>;
using Quat = Eigen::Quaternion<RealNum>;

// boost alias
using BoolArray2 = boost::multi_array<bool, 2>;
using IdxArray2 = boost::multi_array<Idx, 2>;
using BoolArray3 = boost::multi_array<bool, 3>;
using IdxArray3 = boost::multi_array<Idx, 3>;

// limits
const RealNum R_INF = std::numeric_limits<RealNum>::infinity();
const Idx I_INF = std::numeric_limits<Idx>::max();
const RealNum EPS = 1e-6;

// timing
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

#endif // GLOBAL_COMMON_H_
