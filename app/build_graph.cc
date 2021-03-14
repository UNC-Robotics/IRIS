#include <iostream>

#include "global_common.h"

#if USE_CRISP
#include "crisp_robot.h"
#include "crisp_planner.h"
#else
#if USE_PLANAR
#include "planar_robot.h"
#include "planar_planner.h"
#else
#include <cmath>
#include "drone_robot.h"
#include "drone_planner.h"
#endif // USE_PLANAR
#endif // USE_CRISP

int main(int argc, char** argv) {
#if USE_CRISP

    // CRISP robot.
    if (argc < 4) {
        std::cerr << "Usage:" << argv[0] << " seed num_vertex file_to_write" << std::endl;
        exit(1);
    }

    // Parse input.
    Idx seed = std::stoi(argv[1]);
    SizeType num_vertex = std::stoi(argv[2]);
    String file_to_write = argv[3];

    // Robot.
    auto robot = std::make_shared<crisp::CRISPRobot>();
    robot->Initialize();

    // Environment setup.
    SizeType clean_up_size = 13;
    SizeType entry_point_clean_up_radius = 3;
    auto env = std::make_shared<crisp::CTAnatomy>(crisp::kPluralEffusionAnatomyFileName);
    env->SetFreePoint(crisp::kFreePoint);
    env->RemoveSmallConnectedComponents(clean_up_size);
    env->SetTargetPoints(crisp::kInspectionTargetsFileName);
    env->CleanUpRegion(robot->Design()->EntryPoint(crisp::kCameraIndex), entry_point_clean_up_radius);
    env->CleanUpRegion(robot->Design()->EntryPoint(crisp::kSnareIndex), entry_point_clean_up_radius);
    auto detector = std::make_shared<crisp::CRISPCollisionDetector>(env);

    // Planner.
    auto planner = std::make_shared<crisp::CRISPPlanner>(robot, detector, seed);
    // If we want a fixed start configuration, use a fixed seed point here.
    planner->SampleStartConfig(1000, 1);
    //planner->SampleStartConfig(1000, seed);
    planner->BuildAndSaveInspectionGraph(file_to_write, num_vertex);

#else

#if USE_PLANAR

    // Planar robot.
    if (argc < 4) {
        std::cerr << "Usage:" << argv[0] << " seed num_vertex file_to_write [num_obstacles]" << std::endl;
        exit(1);
    }

    // parse input
    Idx seed = std::stoi(argv[1]);
    SizeType num_vertex = std::stoi(argv[2]);
    String file_to_write = argv[3];
    Idx num_obstacles = 10;

    if (argc > 4) {
        num_obstacles = std::stoi(argv[4]);
    }

    // Robot design.
    Vec2 origin(1, 0);
    unsigned num_links = 5;
    std::vector<RealNum> link_length{0.2, 0.1, 0.2, 0.3, 0.1};
    std::vector<Vec2> bounds(num_links, Vec2(-M_PI, M_PI));
    bounds[0] = Vec2(0, M_PI);
    RealNum fov = M_PI/2;

    auto robot = std::make_shared<planar::PlanarRobot>(origin, link_length, bounds);
    robot->SetCameraFOV(fov);
    robot->Initialize();

    // Environment setup.
    auto env = std::make_shared<planar::PlanarEnvironment>(2.0, 2.0, 100, seed);
    env->RandomObstacles(num_obstacles, 0.3, 0.1, origin);

    // Planner.
    auto planner = std::make_shared<planar::PlanarPlanner>(robot, env, seed);
    // If we want a fixed start configuration, use a fixed seed point here.
    planner->SampleStartConfig(1000, 1);
    //planner->SampleStartConfig(1000, seed);
    planner->BuildAndSaveInspectionGraph(file_to_write, num_vertex);

#else

    // Drone robot.
    if (argc < 4) {
        std::cerr << "Usage:" << argv[0] << " seed num_vertex file_to_write" << std::endl;
        exit(1);
    }

    // Parse input.
    Idx seed = std::stoi(argv[1]);
    SizeType num_vertex = std::stoi(argv[2]);
    String file_to_write = argv[3];

    // Robot.
    auto robot = std::make_shared<drone::DroneRobot>(0.196, 0.2895, -0.049);
    robot->SetCameraParameters(94.0/180 * M_PI, 0.2, 10.0);
    robot->Initialize();

    // Environment setup.
    auto env = std::make_shared<drone::BridgeEnvironment>();

    // Planner
    auto planner = std::make_shared<drone::DronePlanner>(robot, env, seed);
    // If we want a fixed start configuration, use a fixed seed point here.
    planner->SampleStartConfig(1000, 1);
    //planner->SampleStartConfig(1000, seed);
    planner->BuildAndSaveInspectionGraph(file_to_write, num_vertex);

#endif // USE_PLANAR
#endif // USE_CRISP

    return 0;
}
