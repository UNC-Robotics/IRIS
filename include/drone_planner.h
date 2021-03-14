#ifndef DRONE_PLANNER_H
#define DRONE_PLANNER_H

#include <ompl/base/Planner.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

#include "ompl/drone_state_space.h"
#include "ompl/infinite_goal.h"
#include "ompl/rrg.h"

#include "drone_robot.h"
#include "bridge_environment.h"
#include "inspection_graph.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#if REJECT_SAMPLING
#define REJECT_START_COVERAGE 0.0
#define MAX_REJECT_CHECK_RATIO 0.95
#define MIN_REJECT_CHECK_RATIO 0.95
#define REJECT_THRESHOLD 1000
#define COVERAGE_MIN_EXTEND 0.00
#endif

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
    RealNum validity_res_{0.1};
    bool k_nearest_{true};
    bool validate_all_edges_{true};
    SizeType incremental_step_{100};
    RealNum reject_ratio_{1.0};
    RealNum validation_distance_{10.0};
    SizeType num_targets_{0};

    ob::SpaceInformationPtr space_info_;

    void BuildRRGIncrementally(Inspection::Graph *graph, ob::PlannerPtr& planner, ob::PlannerData& tree_data, ob::PlannerData& graph_data);
    RealNum RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound);
    bool StateValid(const ob::State *state);
    SizeType RelativeTime(const TimePoint start) const;
    std::vector<RealNum> StateToConfig(const ob::State *state) const;
    std::vector<Vec2> StateToShape(const ob::State *state) const;
    void ComputeRobotVisibilitySet(VisibilitySet& vis_set) const;
    void ComputeVisibilitySet(Inspection::VPtr vertex) const;
    bool CheckEdge(const ob::State *source, const ob::State *target) const;

#if REJECT_SAMPLING
    VisibilitySet global_vis_set_;
    SizeType invalid_states_counter_{0};
    RealNum reject_check_ratio_{MAX_REJECT_CHECK_RATIO};
    RealNum coverage_min_extend_{COVERAGE_MIN_EXTEND};
#endif
};

}


#endif // DRONE_PLANNER_H
