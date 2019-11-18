#ifndef DRONE_PLANNER_H
#define DRONE_PLANNER_H

// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/base/SpaceInformation.h>
// #include <ompl/base/StateValidityChecker.h>
// #include <ompl/base/ScopedState.h>
// #include <ompl/base/ProblemDefinition.h>
// #include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/Planner.h>
// #include <ompl/base/PlannerStatus.h>
// #include <ompl/base/PlannerData.h>
// #include <ompl/base/StateStorage.h>
// #include <ompl/control/SpaceInformation.h>
// #include <ompl/control/spaces/RealVectorControlSpace.h>
// #include <ompl/control/PathControl.h>
// #include <ompl/geometric/PathGeometric.h>

#include "ompl/drone_state_space.h"
#include "ompl/infinite_goal.h"
#include "ompl/rrg.h"

#include "drone_robot.h"
#include "bridge_environment.h"
#include "inspection_graph.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace drone {

class DronePlanner {
public:
    using RobotPtr = std::shared_ptr<DroneRobot>;
    using EnvPtr = std::shared_ptr<BridgeEnvironment>;
    
    DronePlanner(const RobotPtr& robot, const EnvPtr& env, const Idx seed=1);
    ~DronePlanner() = default;
    
    void SampleStartConfig(const Idx max_iter=1000, const Idx seed=1);
    void SetParams(const RealNum step_size, const bool if_k_nearest);
    void BuildAndSaveInspectionGraph(const String file_name, const Idx target_size);

private:
	RobotPtr robot_;
    EnvPtr env_;
    Idx seed_;
    Rand rng_;
    RealUniformDist uni_;
    RealNum step_size_{3.0};
    bool k_nearest_{true};
    bool validate_all_edges_{true};
    SizeType incremental_step_{100};
    RealNum reject_ratio_{1.0};
    RealNum validation_distance_{10.0};

    ob::SpaceInformationPtr space_info_;

    void BuildRRGIncrementally(Inspection::Graph *graph, ob::PlannerPtr& planner, ob::PlannerData& tree_data, ob::PlannerData& graph_data);
    RealNum RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound);
    bool StateValid(const ob::State *state);
    SizeType RelativeTime(const TimePoint start) const;
    std::vector<RealNum> StateToConfig(const ob::State *state) const;
    std::vector<Vec2> StateToShape(const ob::State *state) const;
    void ComputeVisibilitySet(Inspection::VPtr vertex) const;
    bool CheckEdge(const ob::State *source, const ob::State *target) const;
};

}


#endif // DRONE_PLANNER_H
