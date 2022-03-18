// BSD 3-Clause License

// Copyright (c) 2019, The University of North Carolina at Chapel Hill
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Mengyu Fu

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
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

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

#if REJECT_SAMPLING
#define REJECT_START_COVERAGE 0.0
#define MAX_REJECT_CHECK_RATIO 0.9
#define MIN_REJECT_CHECK_RATIO 0.9
#define REJECT_THRESHOLD 100
#define COVERAGE_MIN_EXTEND 0.000
#endif

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

    oc::DirectedControlSamplerPtr AllocateCrispSampler(const oc::ControlSpace* cspace,
            const oc::SpaceInformation* si);
    bool StateValid(const ob::State* state);
    void Propagate(const ob::State* state, const oc::Control* control, const RealNum time,
                   ob::State* result);

    void BuildRRGIncrementally(Inspection::Graph* graph,
                               ob::PlannerPtr& planner,
                               ob::PlannerData& tree_data,
                               ob::PlannerData& graph_data,
                               const Idx step);
    SizeType RelativeTime(const TimePoint start) const;
    void ComputeVisibilitySet(Inspection::VPtr vertex) const;
    bool CheckEdge(const ob::State* source, const ob::State* target, const RealNum dist) const;

#if REJECT_SAMPLING
    VisibilitySet global_vis_set_;
    SizeType invalid_states_counter_{0};
    RealNum reject_check_ratio_{MAX_REJECT_CHECK_RATIO};
    RealNum coverage_min_extend_{COVERAGE_MIN_EXTEND};
#endif

};

}

#endif // CRISP_PLANNER_H
