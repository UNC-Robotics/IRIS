#include "drone_planner.h"

extern bool extern_validate_sample;

namespace drone {

DronePlanner::DronePlanner(const RobotPtr& robot, const EnvPtr& env, const Idx seed)
    : robot_(robot), env_(env), seed_(seed) {
    rng_.seed(seed_);
    uni_ = RealUniformDist(0, 1);

#if REJECT_SAMPLING
    global_vis_set_.Clear();
#endif

    // ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
}

void DronePlanner::SampleStartConfig(const Idx max_iter, const Idx seed) {
    Rand rng;
    rng.seed(seed);
    RealUniformDist uni(0, 1);

    Vec3 pos;
    RealNum yaw, camera_angle;

    for (auto i = 0; i < max_iter; ++i) {
        for (auto j = 0; j < 3; ++j) {
            auto lo = (j == 2)? env_->EnvironmentBoundary(j) : env_->EnvironmentBoundary(
                          j) - validation_distance_;
            auto hi = env_->EnvironmentBoundary(j, false) + validation_distance_;
            pos[j] = uni(rng)*(hi - lo) + lo;
        }

        yaw = uni(rng)*(kMaxYaw - kMinYaw) + kMinYaw;
        camera_angle = uni(rng)*(kMaxCameraAngle - kMinCameraAngle) + kMinCameraAngle;

        robot_->SetConfig(pos, yaw, camera_angle);
        robot_->ComputeShape();

        if (env_->IsCollisionFree(robot_->Config()->Position(), robot_->SphereRadius())
                && env_->IfCorrectDirection(robot_->CameraPos(), robot_->CameraTangent(), robot_->FOV(),
                                            validation_distance_)) {
            robot_->SaveStartConfig();
            std::cout << "Found at " << i << std::endl;
            std::cout << "Start config: ";
            robot_->Config()->Print(std::cout);
            return;
        }
    }

    std::cout << "Fail to find valid start config." << std::endl;
    exit(1);
}

void DronePlanner::SetParams(const RealNum step_size, const bool if_k_nearest) {
    step_size_ = step_size;
    k_nearest_ = if_k_nearest;
}

void DronePlanner::BuildAndSaveInspectionGraph(const String file_name, const Idx target_size) {
    std::cout << "Prepare to build an inspection graph of size: " << target_size << std::endl;

    // State space.
    ompl::RNG::setSeed(seed_);
    num_targets_ = env_->NumTargets();
    auto state_space_ = ob::StateSpacePtr(new DroneStateSpace());

    ob::RealVectorBounds bounds(5);

    for (auto i = 0; i < 3; ++i) {
        if (i < 2) {
            bounds.setLow(i, env_->EnvironmentBoundary(i) - validation_distance_);
        }
        else {
            bounds.setLow(i, env_->EnvironmentBoundary(i));
        }

        bounds.setHigh(i, env_->EnvironmentBoundary(i, false) + validation_distance_);
    }

    bounds.setLow(3, kMinYaw);
    bounds.setHigh(3, kMaxYaw);
    bounds.setLow(4, kMinCameraAngle);
    bounds.setHigh(4, kMaxCameraAngle);
    state_space_->as<DroneStateSpace>()->setBounds(bounds);

    // Space info.
    space_info_.reset(new ob::SpaceInformation(state_space_));
    using namespace std::placeholders;
    space_info_->setStateValidityChecker(std::bind(&DronePlanner::StateValid, this, _1));
    space_info_->setStateValidityCheckingResolution(validity_res_);
    space_info_->setup();

    // Problem definition.
    auto problem_def = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(space_info_));
    ob::State* start = space_info_->allocState();
    state_space_->as<DroneStateSpace>()->StateFromConfiguration(start, robot_->StartConfig());

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
    Inspection::Graph* graph = new Inspection::Graph();

    ob::PlannerData tree_data(space_info_);
    ob::PlannerData graph_data(space_info_);

    while (graph->NumVertices() < target_size) {
        BuildRRGIncrementally(graph, planner, tree_data, graph_data);
        std::cout << "Covered targets: " << graph->NumTargetsCovered()
                  << ", " << graph->NumTargetsCovered()*(RealNum)100/num_targets_ << "%" << std::endl;
    }

    graph->Save(file_name, true);

    delete graph;
}

void DronePlanner::BuildRRGIncrementally(Inspection::Graph* graph,
        ob::PlannerPtr& planner,
        ob::PlannerData& tree_data,
        ob::PlannerData& graph_data) {
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
#if REJECT_SAMPLING
        global_vis_set_.Insert(graph->GlobalVisibility());
#endif

        // tree edges
        std::vector<unsigned> edges;
        auto num_parent = tree_data.getIncomingEdges(i, edges);

        if (num_parent == 1) {
            Idx p = edges[0];
            Inspection::EPtr edge(new Inspection::Edge(p, i));
            edge->checked = true;
            edge->valid = true;
            edge->cost = space_info_->distance(tree_data.getVertex(p).getState(),
                                               tree_data.getVertex(i).getState());
            graph->AddEdge(edge);
        }
        else if (num_parent != 0) {
            std::cerr << "More than one parent! Error!" << std::endl;
            exit(1);
        }

        // graph edges
        edges.clear();
        graph_data.getEdges(i, edges);

        for (auto&& e : edges) {
            Inspection::EPtr edge(new Inspection::Edge(i, e));
            const ob::State* source = tree_data.getVertex(i).getState();
            const ob::State* target = tree_data.getVertex(e).getState();
            edge->cost = space_info_->distance(source, target);

            if (validate_all_edges_) {
                const TimePoint start = Clock::now();
                bool valid = this->CheckEdge(source, target);
                // edge->checked = true;
                edge->valid = valid;
                edge->time_forward_kinematics = RelativeTime(start);
            }

            graph->AddEdge(edge);
        }
    }
}

void DronePlanner::ComputeRobotVisibilitySet(VisibilitySet& vis_set) const {
    auto visible_points = env_->GetVisiblePointIndices(robot_->CameraPos(),
                          robot_->CameraTangent(),
                          robot_->FOV(),
                          robot_->MinDOF(),
                          robot_->MaxDOF());
    vis_set.Clear();

    for (auto p : visible_points) {
        vis_set.Insert(p);
    }
}

void DronePlanner::ComputeVisibilitySet(Inspection::VPtr vertex) const {
    const auto& s = vertex->state->as<DroneStateSpace::StateType>();
    robot_->SetConfig(s->Position(), s->Yaw(), s->CameraAngle());
    robot_->ComputeShape();
    this->ComputeRobotVisibilitySet(vertex->vis);
}

bool DronePlanner::StateValid(const ob::State* state) {
    const auto& s = state->as<DroneStateSpace::StateType>();

    auto collision_free = env_->IsCollisionFree(s->Position(), robot_->SphereRadius());

    if (!collision_free || !extern_validate_sample) {
        return collision_free;
    }

    extern_validate_sample = false;
    robot_->SetConfig(s->Position(), s->Yaw(), s->CameraAngle());
    robot_->ComputeShape();

    bool valid_direction = env_->IfCorrectDirection(robot_->CameraPos(), robot_->CameraTangent(),
                           robot_->FOV(),
                           validation_distance_);

    if (!valid_direction && RandomRealNumber(0, 1) < reject_ratio_) {
        // Camera is not facing the correct direction.
        return false;
    }

#if REJECT_SAMPLING
    bool valid = true;

    RealNum coverage = global_vis_set_.Size()/(RealNum)num_targets_;

    if (coverage > REJECT_START_COVERAGE) {
        if (RandomRealNumber(0, 1) < reject_check_ratio_) {
            VisibilitySet vis_set;
            this->ComputeRobotVisibilitySet(vis_set);
            vis_set.Insert(global_vis_set_);
            RealNum extend_ratio = (vis_set.Size() - global_vis_set_.Size())/(RealNum)num_targets_;
            valid = (extend_ratio > coverage_min_extend_);

            if (!valid) {
                invalid_states_counter_++;
            }
            else {
                invalid_states_counter_ = 0;
                reject_check_ratio_ *= 1.05;
                reject_check_ratio_ = std::fmin(reject_check_ratio_, MAX_REJECT_CHECK_RATIO);

                // coverage_min_extend_ *= 1.5;
                // coverage_min_extend_ = std::fmin(coverage_min_extend_, COVERAGE_MIN_EXTEND);

                global_vis_set_.Insert(vis_set);
            }
        }

        if (invalid_states_counter_ == REJECT_THRESHOLD) {
            // Perform less rejection.
            reject_check_ratio_ *= 0.95;
            reject_check_ratio_ = std::fmax(reject_check_ratio_, MIN_REJECT_CHECK_RATIO);

            // Set a lower bar for accepting a state.
            coverage_min_extend_ *= 0.5;

            // Reset counter
            invalid_states_counter_ = 0;
        }
    }

    return valid;
#endif

    return true;
}

bool DronePlanner::CheckEdge(const ob::State* source, const ob::State* target) const {
    const auto& s0 = source->as<DroneStateSpace::StateType>();
    const auto& s1 = target->as<DroneStateSpace::StateType>();
    Vec3 p0 = s0->Position();
    Vec3 p1 = s1->Position();

    Idx num_steps = std::ceil((p0 - p1).norm() / 0.5);

    for (Idx i = 1; i < num_steps; ++i) {
        RealNum p = i / (RealNum)num_steps;
        Vec3 config_mid = p0 * p + p1 * (1 - p);

        if (!env_->IsCollisionFree(config_mid, robot_->SphereRadius())) {
            return false;
        }
    }

    return true;
}


RealNum DronePlanner::RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound) {
    return uni_(rng_)*(higher_bound - lower_bound) + lower_bound;
}

SizeType DronePlanner::RelativeTime(const TimePoint start) const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
}

}
