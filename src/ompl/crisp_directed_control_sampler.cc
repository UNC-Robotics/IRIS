#include "ompl/crisp_directed_control_sampler.h"

CrispDirectedControlSampler::CrispDirectedControlSampler(const RobotPtr robot,
                                const DetectorPtr detector,
                                const oc::ControlSpace *cspace, 
                                const oc::SpaceInformation *si, 
                                const RealNum quat_deviation_percent, 
                                const Rand& rng):
    DirectedControlSampler(si),
    robot_(robot),
    detector_(detector),
    cspace_(cspace),
    // si_(si),
    quat_deviation_percent_(quat_deviation_percent),
    rng_(rng) {

    uni_ = RealUniformDist(0,1);
}

RealNum CrispDirectedControlSampler::RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound) {
    return uni_(rng_)*(higher_bound - lower_bound) + lower_bound;
}

void CrispDirectedControlSampler::SampleStateUniform(ob::State *state) {
    // const Idx tool_i = 0;
    // const Idx snare_i = 1;
    auto design = robot_->Design();

    const Vec3 e2 = Vec3::UnitZ();
    for (Idx i = 0; i < NUM_TUBES; ++i) {
        RealNum insertion = this->RandomRealNumber(design->MinInsertion(i), design->MaxInsertion(i));
        
        state->as<CrispStateSpace::StateType>()->SetInsertion(i, insertion);

        RealNum theta = this->RandomRealNumber(0, 2*M_PI);
        RealNum z = this->RandomRealNumber(quat_deviation_percent_, 1);
        RealNum r = sqrt(1 - pow(z,2));
        Vec3 v(r*cos(theta), r*sin(theta), z);
        RealNum roll = this->RandomRealNumber(min_roll_, max_roll_);
        Quat q = robot_->StartConfig()->TubeOrientation(i);
        q = q * (Quat::FromTwoVectors(e2, v) * Quat(Eigen::AngleAxisf(roll, e2)));

        state->as<CrispStateSpace::StateType>()->SetQuaternion(i, q);
    }

    // initialize as zero
    state->as<CrispStateSpace::StateType>()->SetTipTranslation(Vec3(0,0,0));
    state->as<CrispStateSpace::StateType>()->SetTipTangent(Vec3(0,0,0));
    state->as<CrispStateSpace::StateType>()->SetKinStateVector(crisp::KinVec::Zero());

    state->as<CrispStateSpace::StateType>()->SetStability(0.0);
    state->as<CrispStateSpace::StateType>()->SetValid(true);

    cspace_->getStateSpace()->enforceBounds(state);

}

unsigned CrispDirectedControlSampler::sampleTo(oc::Control *control, const oc::Control *prev, const ob::State *source, ob::State *dest) {
    RealNum dist = 0.2; //si_->distance(source, dest);
    return this->SampleToConsideringDistance(source, dest, dist);
}

unsigned CrispDirectedControlSampler::sampleTo(oc::Control *control, const ob::State *source, ob::State *dest) {
    RealNum dist = 0.2; //si_->distance(source, dest);
    return this->SampleToConsideringDistance(source, dest, dist);
}

unsigned CrispDirectedControlSampler::SampleToConsideringDistance(const ob::State *source, ob::State *dest, const RealNum dist) {
    const Idx tool_i = 0;
    const Idx snare_i = 1;
    RealNum max_time = this->RandomRealNumber(0, 1);
    const RealNum dt = dx_/dist;

    const CrispStateSpace::StateType *c_source = source->as<CrispStateSpace::StateType>();
    CrispStateSpace::StateType *c_dest = dest->as<CrispStateSpace::StateType>();

    // source
    Quat tool_quat_source = c_source->Quaternion(tool_i);
    RealNum tool_ins_source = c_source->Insertion(tool_i);
    Quat snare_quat_source = c_source->Quaternion(snare_i);
    RealNum snare_ins_source = c_source->Insertion(snare_i);
    crisp::KinVec vec = c_source->KinStateVector();

    // dest
    Quat tool_quat_dest = c_dest->Quaternion(tool_i);
    RealNum tool_ins_dest = c_dest->Insertion(tool_i);
    Quat snare_quat_dest = c_dest->Quaternion(snare_i);
    RealNum snare_ins_dest = c_dest->Insertion(snare_i);

    // 
    Quat tool_quat_mid, snare_quat_mid;
    RealNum tool_ins_mid, snare_ins_mid;

    auto prev_config = std::make_shared<crisp::CRISPConfig>();

    bool one_good = false;
    bool last_iter = false;

    if (dt >= max_time) {
        last_iter = true;
    }

    for (RealNum t = dt; t < max_time || last_iter; t += dt) {
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

        if (!config->IsValid()
                || config->Stability() > crisp::kStableThreshold 
                || detector_->Collides(config)) {
            if (!one_good) { return 0; }

            dest->as<CrispStateSpace::StateType>()->SetState(prev_config);
            cspace_->getStateSpace()->enforceBounds(dest);
            // if (!CheckState(dest)) {
                // std::cout << "middle" << std::endl;
                // getchar();
            // }

            return 1;
        }

        one_good = true;
        vec = config->KinematicState();
        prev_config = config;

        if (last_iter) {
            dest->as<CrispStateSpace::StateType>()->SetState(prev_config);
            cspace_->getStateSpace()->enforceBounds(dest);
            // if (!CheckState(dest)) {
                // std::cout << "last" << std::endl;
                // getchar();
            // }
            return 1;
        }

        if (t < max_time && t + dt >= max_time) {
            last_iter = true;
        }
    }

    return 0;
}

unsigned CrispDirectedControlSampler::SampleToConsideringDistanceWithTiming(const ob::State *source, ob::State *dest, const RealNum dist, unsigned *time_for_fk, unsigned *time_for_cd) {
    const Idx tool_i = 0;
    const Idx snare_i = 1;
    RealNum max_time = this->RandomRealNumber(0, 1);
    const RealNum dt = dx_/dist;

    const CrispStateSpace::StateType *c_source = source->as<CrispStateSpace::StateType>();
    CrispStateSpace::StateType *c_dest = dest->as<CrispStateSpace::StateType>();

    // source
    Quat tool_quat_source = c_source->Quaternion(tool_i);
    RealNum tool_ins_source = c_source->Insertion(tool_i);
    Quat snare_quat_source = c_source->Quaternion(snare_i);
    RealNum snare_ins_source = c_source->Insertion(snare_i);
    auto vec = c_source->KinStateVector();

    // dest
    Quat tool_quat_dest = c_dest->Quaternion(tool_i);
    RealNum tool_ins_dest = c_dest->Insertion(tool_i);
    Quat snare_quat_dest = c_dest->Quaternion(snare_i);
    RealNum snare_ins_dest = c_dest->Insertion(snare_i);

    // 
    Quat tool_quat_mid, snare_quat_mid;
    RealNum tool_ins_mid, snare_ins_mid;

    auto prev_config = std::make_shared<crisp::CRISPConfig>();

    bool one_good = false;
    bool last_iter = false;

    if (dt >= max_time) {
        last_iter = true;
    }

    for (RealNum t = dt; t < max_time || last_iter; t += dt) {
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

        const TimePoint start_fk = Clock::now();
        robot_->ComputeShape();
        *time_for_fk += RelativeTime(start_fk);
        const auto& config = robot_->Config();

        if (!config->IsValid()
                || config->Stability() > crisp::kStableThreshold ) {
                // || robot_->CollisionDetector()->Collides(config)) {
            if (!one_good) { return 0; }

            dest->as<CrispStateSpace::StateType>()->SetState(prev_config);
            cspace_->getStateSpace()->enforceBounds(dest);
            // if (!CheckState(dest)) {
                // std::cout << "middle" << std::endl;
                // getchar();
            // }

            return 1;
        }
        else {
            const TimePoint start_cd = Clock::now();
            bool in_collision = detector_->Collides(config);
            *time_for_cd += RelativeTime(start_cd);

            if (in_collision) {
                if (!one_good) { return 0; }

            dest->as<CrispStateSpace::StateType>()->SetState(prev_config);
            cspace_->getStateSpace()->enforceBounds(dest);
            return 1;
            }
        }

        one_good = true;
        vec = config->KinematicState();
        prev_config = config;

        if (last_iter) {
            dest->as<CrispStateSpace::StateType>()->SetState(prev_config);
            cspace_->getStateSpace()->enforceBounds(dest);
            // if (!CheckState(dest)) {
                // std::cout << "last" << std::endl;
                // getchar();
            // }
            return 1;
        }

        if (t < max_time && t + dt >= max_time) {
            last_iter = true;
        }
    }

    return 0;
}

bool CrispDirectedControlSampler::CheckState(const ob::State *state) const {
    const CrispStateSpace::StateType *s = state->as<CrispStateSpace::StateType>();

    robot_->SetControlInput(s->Insertion(crisp::kCameraIndex),
                            s->Insertion(crisp::kSnareIndex),
                            s->Quaternion(crisp::kCameraIndex),
                            s->Quaternion(crisp::kSnareIndex));

    robot_->SetKinematicStateSeed(s->KinStateVector());
    robot_->ComputeShape();
    const auto& config = robot_->Config();

    if (!config->IsValid() || config->Stability() > crisp::kStableThreshold) {
        return false;
    }

    return !(detector_->Collides(config));
}

bool CrispDirectedControlSampler::CheckMotion(const ob::State *start, const ob::State *goal, const RealNum dist) const {
    RealNum max_time = 1.0;
    const RealNum dt = dx_/dist;

    const CrispStateSpace::StateType *c_source = start->as<CrispStateSpace::StateType>();
    const CrispStateSpace::StateType *c_dest = goal->as<CrispStateSpace::StateType>();

    // source
    Quat tool_quat_source = c_source->Quaternion(crisp::kCameraIndex);
    RealNum tool_ins_source = c_source->Insertion(crisp::kCameraIndex);
    Quat snare_quat_source = c_source->Quaternion(crisp::kSnareIndex);
    RealNum snare_ins_source = c_source->Insertion(crisp::kSnareIndex);
    auto vec = c_source->KinStateVector();

    // dest
    Quat tool_quat_dest = c_dest->Quaternion(crisp::kCameraIndex);
    RealNum tool_ins_dest = c_dest->Insertion(crisp::kCameraIndex);
    Quat snare_quat_dest = c_dest->Quaternion(crisp::kSnareIndex);
    RealNum snare_ins_dest = c_dest->Insertion(crisp::kSnareIndex);

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

        if (!config->IsValid() || config->Stability() > crisp::kStableThreshold) {
            return false;
        }

        if (detector_->Collides(config)) {
            return false;
        }

        vec = config->KinematicState();

        if (last_iter) {
            return true;
        }

        if (t < max_time && t + dt >= max_time) {
            last_iter = true;
        }
    }

    return false;
}

unsigned CrispDirectedControlSampler::RelativeTime(const TimePoint start) const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
}