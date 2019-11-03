#include <iostream>

#include "ompl/rrg.h"

#include "planar_planner.h"

namespace planar {

PlanarPlanner::PlanarPlanner(const RobotPtr& robot, const EnvPtr& env, const Idx seed)
 : robot_(robot), env_(env), seed_(seed)
{
    rng_.seed(seed_);
    uni_ = RealUniformDist(0, 1);
    // ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
}

void PlanarPlanner::SampleStartConfig(const Idx max_iter, const Idx seed) {
    Rand rng;
    rng.seed(seed);
    RealUniformDist uni(0, 1);

    std::vector<RealNum> config;
    for (Idx iter = 0; iter < max_iter; ++iter) {
        config.clear();
        for (Idx i = 0; i < robot_->NumLinks(); ++i) {
            auto bounds = robot_->Bounds(i);
            auto ang = uni(rng)*(bounds[1] - bounds[0]) + bounds[0];
            config.push_back(ang);
        }

        robot_->SetConfig(config);
        robot_->ComputeShape();

        if (env_->IsCollisionFree(robot_->Shape())) {
            robot_->SaveStartConfig();
            std::cout << "Found at " << iter << std::endl;
            std::cout << "Start config: ";
            robot_->PrintConfig();
            return;
        }
    }

    std::cout << "Fail to find valid start config." << std::endl;
    exit(1);
}

void PlanarPlanner::SetParams(const RealNum step_size, const bool if_k_nearest) {
    step_size_ = step_size;
    k_nearest_ = if_k_nearest;
}

void PlanarPlanner::BuildAndSaveInspectionGraph(const String file_name, const Idx target_size) {
    std::cout << "Prepare to build an inspection graph of size: " << target_size << std::endl;

    auto dim = robot_->NumLinks();

    // State space.
    ompl::RNG::setSeed(seed_);
    auto state_space_ = ob::StateSpacePtr(new ob::RealVectorStateSpace(dim));
    ob::RealVectorBounds bounds(dim);
    for (Idx i = 0; i < robot_->NumLinks(); ++i) {
        auto b = robot_->Bounds(i);
        bounds.setLow(i, b[0]);
        bounds.setHigh(i, b[1]);
    }
    state_space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Space info.
    space_info_.reset(new ob::SpaceInformation(state_space_));
    space_info_->setStateValidityCheckingResolution(0.01);
    using namespace std::placeholders;
    space_info_->setStateValidityChecker(std::bind(&PlanarPlanner::StateValid, this, _1));
    space_info_->setup();

    // Problem definition.
    auto problem_def = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(space_info_));
    ob::ScopedState<> start(state_space_);
    auto config = robot_->StartConfig();
    for (Idx i = 0; i < dim; ++i) {
        start->as<ob::RealVectorStateSpace::StateType>()->values[i] = config[i];
    }

    problem_def->addStartState(start);
    auto goal = ob::GoalPtr(new InfiniteGoal(space_info_));
    problem_def->setGoal(goal);
    auto obj = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(space_info_));
    problem_def->setOptimizationObjective(obj);

    // Planner.
    auto planner = ob::PlannerPtr(new og::RRG(space_info_));
    planner->as<og::RRG>()->setRange(step_size_);
    planner->as<og::RRG>()->setGoalBias(0.0);
    planner->as<og::RRG>()->setKNearest(k_nearest_);
    planner->setProblemDefinition(problem_def);
    planner->setup();

    // Build graph incrementally.
    auto num_targets = env_->NumTargets();
    Inspection::Graph *graph = new Inspection::Graph();
    
    ob::PlannerData tree_data(space_info_);
    ob::PlannerData graph_data(space_info_);

    while (graph->NumVertices() < target_size) {
        BuildRRGIncrementally(graph, planner, tree_data, graph_data);
        std::cout << "Covered targets: " << graph->NumTargetsCovered() 
            << ", " << graph->NumTargetsCovered()*(RealNum)100/num_targets << "%" << std::endl;
    }

    graph->Save(file_name, true, dim);

    delete graph;
}

void PlanarPlanner::BuildRRGIncrementally(Inspection::Graph *graph, 
                            ob::PlannerPtr& planner, 
                            ob::PlannerData& tree_data, 
                            ob::PlannerData& graph_data) 
{
    // use ompl planner
    const TimePoint start = Clock::now();
    ob::PlannerStatus solved;
    ob::IterationTerminationCondition ptc(incremental_step_);
    solved = planner->solve(ptc);
    planner->as<og::RRG>()->getPlannerData(tree_data);
    planner->as<og::RRG>()->getUncheckedEdges(graph_data);
    SizeType total_time_build = RelativeTime(start);

    SizeType avg_time_build = 0;
    Idx prev_size = graph->NumVertices();
    Idx current_size = tree_data.numVertices();
    if (current_size > prev_size) {
        avg_time_build = SizeType(total_time_build/(current_size - prev_size));
    }
    else {
        return;
    }

    // update inspection graph
    for (Idx i = prev_size; i < current_size; ++i) {
        std::cout << i << " " << std::flush;
        graph->AddVertex(i);
        auto vertex = graph->Vertex(i);
        vertex->state = space_info_->allocState();
        space_info_->copyState(vertex->state, tree_data.getVertex(i).getState());

        const TimePoint start = Clock::now();
        ComputeVisibilitySet(vertex);
        vertex->time_vis = RelativeTime(start);
        vertex->time_build = avg_time_build;

        graph->UpdateGlobalVisibility(vertex->vis);

        // tree edges
        std::vector<unsigned> edges;
        auto num_parent = tree_data.getIncomingEdges(i, edges);
        if (num_parent == 1) {
            Idx p = edges[0];
            Inspection::EPtr edge(new Inspection::Edge(p, i));
            edge->checked = true;
            edge->valid = true;
            edge->cost = space_info_->distance(tree_data.getVertex(p).getState(), tree_data.getVertex(i).getState());
            graph->AddEdge(edge);
        }
        else if (num_parent != 0){
            std::cerr << "More than one parent! Error!" << std::endl;
            exit(1);
        }

        // graph edges
        edges.clear();
        graph_data.getEdges(i, edges);
        for (auto && e : edges) {
            Inspection::EPtr edge(new Inspection::Edge(i, e));
            const ob::State *source = tree_data.getVertex(i).getState();
            const ob::State *target = tree_data.getVertex(e).getState();
            edge->cost = space_info_->distance(source, target);

            if (validate_all_edges_) {
                const TimePoint start = Clock::now();
                bool valid = this->CheckEdge(source, target, edge->cost);
                // edge->checked = true;
                edge->valid = valid;
                edge->time_forward_kinematics = RelativeTime(start);
            }

            graph->AddEdge(edge);
        }
    }
}

void PlanarPlanner::ComputeVisibilitySet(Inspection::VPtr vertex) const {
    auto shape = StateToShape(vertex->state);
    auto visible_points = env_->GetVisiblePointIndices(shape, robot_->FOV());
    auto& vis_set = vertex->vis;
    vis_set.Clear();

    for (auto p : visible_points) {
        vis_set.Insert(p);
    }
}

RealNum PlanarPlanner::RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound) {
    return uni_(rng_)*(higher_bound - lower_bound) + lower_bound;
}

bool PlanarPlanner::StateValid(const ob::State *state) {
    return env_->IsCollisionFree(StateToShape(state));
}

bool PlanarPlanner::CheckEdge(const ob::State *source, const ob::State *target, const RealNum dist) const {
    Idx num_steps = std::ceil(dist / step_size_);

    auto config0 = StateToConfig(source);
    auto config1 = StateToConfig(target);

    for (Idx i = 1; i < num_steps; ++i) {
        RealNum p = i / (RealNum)num_steps;
        std::vector<RealNum> config_mid;
        for (Idx j = 0; j < robot_->NumLinks(); ++j) {
            config_mid.push_back(config0[j]*p + config1[j]*(1 - p));
        }

        robot_->SetConfig(config_mid);
        robot_->ComputeShape();

        if (!env_->IsCollisionFree(robot_->Shape())) {
            return false;
        }
    }

    return true;
}

std::vector<RealNum> PlanarPlanner::StateToConfig(const ob::State *state) const {
    const auto s = state->as<ob::RealVectorStateSpace::StateType>();
    std::vector<RealNum> config;

    for (Idx i = 0; i < robot_->NumLinks(); ++i) {
        config.push_back(s->values[i]);
    }

    return config;
}

std::vector<Vec2> PlanarPlanner::StateToShape(const ob::State *state) const {
    auto config = StateToConfig(state);
    
    robot_->SetConfig(config);
    robot_->ComputeShape();

    return robot_->Shape();
}

SizeType PlanarPlanner::RelativeTime(const TimePoint start) const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
}

}
