#include "crisp_planner.h"

namespace crisp {

CRISPPlanner::CRISPPlanner(const RobotPtr robot, const CollisionDetectorPtr detector, const Idx seed)
    : robot_(robot), detector_(detector) {

    SetSeed(seed);

    // ompl
    state_space_.reset(new CrispStateSpace(NUM_TUBES));
    control_space_.reset(new CrispControlSpace(state_space_));
    space_info_.reset(new oc::SpaceInformation(state_space_, control_space_));
    goal_.reset(new InfiniteGoal(space_info_));

    // allocator for control sampler
    space_info_->setDirectedControlSamplerAllocator(
        [this](const oc::SpaceInformation *si) {
            return AllocateCrispSampler(control_space_.get(), si);
    });

    using namespace std::placeholders;

    // state validity checker
    // std::bind(function name, arguments), here placeholder is for true input parameter for function StateValid
    space_info_->setStateValidityChecker(std::bind(&CRISPPlanner::StateValid, this, _1));

    // propagator
    // std::placeholders::_1 to _4 are true inputs for function Propagate
    space_info_->setStatePropagator(std::bind(&CRISPPlanner::Propagate, this, _1, _2, _3, _4));

    space_info_->setup();
    // space_info_->printSettings(std::cout);
}

oc::DirectedControlSamplerPtr CRISPPlanner::AllocateCrispSampler(const oc::ControlSpace *cspace, const oc::SpaceInformation *si) {
    return oc::DirectedControlSamplerPtr(new CrispDirectedControlSampler(robot_, detector_, cspace, si, quat_deviation_percent_, rng_));
}

bool CRISPPlanner::StateValid(const ob::State *state) {
    // valid iff kinematics is solved and 
    return state->as<CrispStateSpace::StateType>()->IsValid();
}

void CRISPPlanner::Propagate(const ob::State *state, const oc::Control *control, const RealNum time, ob::State *result) {
    // don't need anything here
    std::cout << "Propagate function called..." << std::endl;
    exit(1);
}

void CRISPPlanner::SetSeed(const Idx seed) {
    if (!set_seed_) {
        seed_ = seed;
        rng_.seed(seed_);
        uni_ = RealUniformDist(0, 1);
        set_seed_ = true;

        ompl::RNG::setSeed(seed);

        std::cout << "Seed for planner set to: " << seed << std::endl;
    }
    else {
        std::cout << "Already have a seed: " << seed_ << std::endl;
    }
}

RealNum CRISPPlanner::RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound) {
    return uni_(rng_)*(higher_bound - lower_bound) + lower_bound;
}

// currently this is only for 2-tube robot
void CRISPPlanner::SampleStartConfig(const Idx max_iter, const Idx seed) {
    std::cout << "Sampling start configuration..." << std::endl;

    auto design = robot_->Design();

    Vec3 p0 = design->EntryPoint(0);
    Vec3 p1 = design->EntryPoint(1);
    Vec3 between_entries = p1 - p0;
    Vec3 center = (p0 + p1)/2;
    RealNum radius = between_entries.norm()/2;
    Rand rng;
    rng.seed(seed);
    RealNormalDist normal(0, 1);
    
    Vec3 p;
    bool find_valid_config = false;
    for (Idx i = 0; i < max_iter; ++i) {
        for (Idx d = 0; d < 3; d++) {
            p[d] = normal(rng);
        }
        p.normalize();
        p *= radius;
        p += center;
        
        RealNum tool_insertion = (p - p0).norm() - design->GraspLocation(); 
        RealNum snare_insertion = (p - p1).norm(); 

        // compute quaternion for tool tube
        Mat3 tool_rot;
        tool_rot.col(0) = (p1 - p).normalized(); // x
        tool_rot.col(2) = (p - p0).normalized(); // z
        tool_rot.col(1) = (tool_rot.col(2)).cross(tool_rot.col(0)); // y
        Quat tool_quat(tool_rot);

        // compute quaternion for snare tube
        Quat z_to_minus_x = Quat::FromTwoVectors(Vec3(0,0,1), Vec3(-1,0,0));
        Quat snare_quat = tool_quat * z_to_minus_x;

        robot_->SetControlInput(tool_insertion, snare_insertion, tool_quat, snare_quat);

        robot_->SetKinematicStateSeed(KinVec::Zero());
        robot_->ComputeShape();

        const auto& config = robot_->Config();
        bool in_collision = false;
        if (config->IsValid() && config->Stability() < kStableThreshold) {
            in_collision = detector_->Collides(config);
            if (!in_collision) {
                find_valid_config = true;
                break;
            }
        }

        std::cout << "Configuration solved: " << config->IsValid()
            << ", stability: " << config->Stability() 
            << ", collision: " << in_collision
            << ", retry..." << std::endl;
    }

    if (find_valid_config) {
        robot_->SaveStartConfig();
        std::cout << "Start configuration: " << std::endl;
        robot_->Config()->Print(std::cout);
        return;
    }

    std::cerr << "No valid start configuration found in " << max_iter << " iterations!" << std::endl;
    exit(1);
}

void CRISPPlanner::Setup() {
    if (robot_->Design() == nullptr) {
        std::cerr << "No design for robot!" << std::endl;
        exit(1);
    }
    if (detector_ == nullptr) {
        std::cerr << "No collision detector for robot!" << std::endl;
        exit(1);
    }
    if (robot_->StartConfig() == nullptr) {
        std::cerr << "No start configuration!" << std::endl;
        exit(1);
    }

    num_targets_ = detector_->Environment()->NumTargets();

    ob::State *start = space_info_->allocState();
    start->as<CrispStateSpace::StateType>()->SetState(robot_->StartConfig());
    space_info_->enforceBounds(start);

    problem_def_.reset(new ob::ProblemDefinition(space_info_));
    problem_def_->addStartState(start);
    problem_def_->setGoal(goal_);

    std::cout << "Planner setup." << std::endl;
}

void CRISPPlanner::BuildAndSaveInspectionGraph(const String file_name, const SizeType target_size) {
    std::cout << "Prepare to build an inspection graph of size: " << target_size << std::endl;
    Setup();
    ob::PlannerPtr planner(new oc::RRG(space_info_));
    planner->setProblemDefinition(problem_def_);
    planner->setup();
    ob::PlannerData tree_data(space_info_);
    ob::PlannerData graph_data(space_info_);

    Inspection::Graph *graph = new Inspection::Graph();
    
    validate_all_edges_ = true;
    SizeType incremental_step = 100;
    const TimePoint start = Clock::now();
    while (graph->NumVertices() < target_size) {
        BuildRRGIncrementally(graph, planner, tree_data, graph_data, incremental_step);
        std::cout << "Time elapsed: " << RelativeTime(start)/(RealNum)1000 
            << ", covered targets: " << graph->NumTargetsCovered() 
            << ", " << graph->NumTargetsCovered()*(RealNum)100/num_targets_ << "%" << std::endl;
    }

    graph->Save(file_name);

    delete graph;
}

void CRISPPlanner::BuildRRGIncrementally(Inspection::Graph *graph, 
                            ob::PlannerPtr& planner, 
                            ob::PlannerData& tree_data, 
                            ob::PlannerData& graph_data, 
                            const Idx step) {

    // use ompl planner
    const TimePoint start = Clock::now();
    ob::PlannerStatus solved;
    ob::IterationTerminationCondition ptc(step);
    solved = planner->solve(ptc);
    planner->as<oc::RRG>()->getPlannerData(tree_data);
    planner->as<oc::RRG>()->getUncheckedEdges(graph_data);
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

SizeType CRISPPlanner::RelativeTime(const TimePoint start) const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
}

void CRISPPlanner::ComputeVisibilitySet(Inspection::VPtr vertex) const {
    const CrispStateSpace::StateType *s = vertex->state->as<CrispStateSpace::StateType>();

    detector_->ComputeVisSetForConfiguration(s->TipTranslation(), s->TipTangent(), &(vertex->vis));
}

bool CRISPPlanner::CheckEdge(const ob::State *source, const ob::State *target, const RealNum dist) const {
    const Idx tool_i = 0;
    const Idx snare_i = 1;
    RealNum max_time = 1.0;
    const RealNum dt = dx_/dist;

    const CrispStateSpace::StateType *c_source = source->as<CrispStateSpace::StateType>();
    const CrispStateSpace::StateType *c_dest = target->as<CrispStateSpace::StateType>();

    // source
    Quat tool_quat_source = c_source->Quaternion(tool_i);
    RealNum tool_ins_source = c_source->Insertion(tool_i);
    Quat snare_quat_source = c_source->Quaternion(snare_i);
    RealNum snare_ins_source = c_source->Insertion(snare_i);
    KinVec vec = c_source->KinStateVector();

    // dest
    Quat tool_quat_dest = c_dest->Quaternion(tool_i);
    RealNum tool_ins_dest = c_dest->Insertion(tool_i);
    Quat snare_quat_dest = c_dest->Quaternion(snare_i);
    RealNum snare_ins_dest = c_dest->Insertion(snare_i);

    // 
    Quat tool_quat_mid, snare_quat_mid;
    RealNum tool_ins_mid, snare_ins_mid;

    bool last_iter = false;

    if (dt >= max_time) {
        last_iter = true;
    }

    for (RealNum t = 0; t < max_time || last_iter; t += dt) {
        if (last_iter) {
            t = max_time;
        }

        tool_ins_mid = t*(tool_ins_dest - tool_ins_source) + tool_ins_source;
        snare_ins_mid = t*(snare_ins_dest - snare_ins_source) + snare_ins_source;

        tool_quat_mid = tool_quat_source.slerp(t, tool_quat_dest);
        tool_quat_mid.normalize();
        snare_quat_mid = snare_quat_source.slerp(t, snare_quat_dest);
        snare_quat_mid.normalize();

        robot_->SetControlInput(tool_ins_mid, snare_ins_mid, tool_quat_mid, snare_quat_mid);
        robot_->SetKinematicStateSeed(vec);
        robot_->ComputeShape();

        const auto& config = robot_->Config();
        if (!config->IsValid() || config->Stability() > kStableThreshold) {
            return false;
        }

        if (detector_->Collides(config)) {
            return false;
        }

        vec = config->KinematicState();

        if (last_iter) {
            return true;
        }

        if (t + dt >= max_time) {
            last_iter = true;
        }
    }

    return false;
}

}
