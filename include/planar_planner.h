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

#ifndef PLANAR_PLANNER_H
#define PLANAR_PLANNER_H

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/StateStorage.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>

#include "planar_robot.h"
#include "planar_environment.h"
#include "ompl/infinite_goal.h"
#include "inspection_graph.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

namespace planar {

class PlanarPlanner {
  public:
    using RobotPtr = std::shared_ptr<PlanarRobot>;
    using EnvPtr = std::shared_ptr<PlanarEnvironment>;

    PlanarPlanner(const RobotPtr& robot, const EnvPtr& env, const Idx seed=1);
    ~PlanarPlanner() = default;

    void SampleStartConfig(const Idx max_iter=1000, const Idx seed=1);
    void SetParams(const RealNum step_size, const bool if_k_nearest);
    void BuildAndSaveInspectionGraph(const String file_name, const Idx target_size);

  private:
    RobotPtr robot_;
    EnvPtr env_;
    Idx seed_;
    Rand rng_;
    RealUniformDist uni_;
    RealNum step_size_{0.1};
    bool k_nearest_{true};
    bool validate_all_edges_{true};
    SizeType incremental_step_{100};

    ob::SpaceInformationPtr space_info_;

    void BuildRRGIncrementally(Inspection::Graph* graph, ob::PlannerPtr& planner,
                               ob::PlannerData& tree_data, ob::PlannerData& graph_data);
    RealNum RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound);
    bool StateValid(const ob::State* state);
    SizeType RelativeTime(const TimePoint start) const;
    std::vector<RealNum> StateToConfig(const ob::State* state) const;
    std::vector<Vec2> StateToShape(const ob::State* state) const;
    void ComputeVisibilitySet(Inspection::VPtr vertex) const;
    bool CheckEdge(const ob::State* source, const ob::State* target, const RealNum dist) const;

};

}


#endif // PLANAR_PLANNER_H
