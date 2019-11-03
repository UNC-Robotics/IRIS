#ifndef CRISP_PLANNER_H
#define CRISP_PLANNER_H

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <utility>

#include <boost/shared_ptr.hpp>

#include <ompl/control/SpaceInformation.h>
#include <ompl/config.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/PathControl.h>
#include <ompl/util/Console.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include "crisp_robot.h"
#include "inspection_graph.h"
#include "ompl/crisp_control_space.h"
#include "ompl/crisp_directed_control_sampler.h"
#include "ompl/crisp_state_space.h"
#include "ompl/infinite_goal.h"
#include "ompl/control_rrg.h"
#include "ompl/control_rrt.h"
#include "ompl/control_rrtstar.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace crisp {

class CRISPPlanner {
public:
    using RobotPtr = std::shared_ptr<CRISPRobot>;
    using CollisionDetectorPtr = std::shared_ptr<CRISPCollisionDetector>;

    CRISPPlanner() = default;
    CRISPPlanner(const RobotPtr robot, const CollisionDetectorPtr detector, const Idx seed=1);
    ~CRISPPlanner() = default;

    void SampleStartConfig(const Idx max_iter=1000, const Idx seed=1);
    void SetQuaternionDeviationPercent(const RealNum p);
    void Setup();
    void BuildAndSaveInspectionGraph(const String file_name, const SizeType target_size);

private:
    bool set_seed_{false};
    bool validate_all_edges_{false};
    Idx seed_;
    Rand rng_;
    RealUniformDist uni_;
    RobotPtr robot_{nullptr};
    CollisionDetectorPtr detector_{nullptr};
    RealNum quat_deviation_percent_{0.2};
    RealNum dx_{1e-3};
    RealNum stability_threshold_{1e-3};
    SizeType num_targets_{0};

    void SetSeed(const Idx seed);
    RealNum RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound);

    // ompl related members
    ob::StateSpacePtr state_space_;
    oc::ControlSpacePtr control_space_;
    oc::SpaceInformationPtr space_info_;
    ob::ProblemDefinitionPtr problem_def_;
    ob::GoalPtr goal_;

    oc::DirectedControlSamplerPtr AllocateCrispSampler(const oc::ControlSpace *cspace, const oc::SpaceInformation *si);
    bool StateValid(const ob::State *state);
    void Propagate(const ob::State *state, const oc::Control *control, const RealNum time, ob::State *result);

    void BuildRRGIncrementally(Inspection::Graph *graph, 
                            ob::PlannerPtr& planner, 
                            ob::PlannerData& tree_data, 
                            ob::PlannerData& graph_data, 
                            const Idx step);
    SizeType RelativeTime(const TimePoint start) const;
    void ComputeVisibilitySet(Inspection::VPtr vertex) const;
    bool CheckEdge(const ob::State *source, const ob::State *target, const RealNum dist) const;
};

}

#endif // CRISP_PLANNER_H
