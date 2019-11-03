#include "ompl/drone_state_space.h"

DroneStateSpace::DroneStateSpace() : ob::CompoundStateSpace() 
{
	RealNum trans_weight = 0.6;
	RealNum yaw_weight = 0.2;
	RealNum camera_weight = 0.2;

	ob::StateSpacePtr trans(new ob::RealVectorStateSpace(3));
	this->addSubspace(trans, trans_weight);

	ob::StateSpacePtr yaw(new ob::RealVectorStateSpace(1));
	this->addSubspace(yaw, yaw_weight);

	ob::StateSpacePtr camera(new ob::RealVectorStateSpace(1));
	this->addSubspace(camera, camera_weight);
}

void DroneStateSpace::setBounds(const ob::RealVectorBounds& bounds) {
	ob::RealVectorBounds trans_bounds(3);
	for (auto i = 0; i < 3; ++i) {
		trans_bounds.setLow(i, bounds.low[i]);
		trans_bounds.setHigh(i, bounds.high[i]);
	}
	this->as<ob::RealVectorStateSpace>(0)->setBounds(trans_bounds);

	ob::RealVectorBounds yaw_bounds(1);
	yaw_bounds.setLow(bounds.low[3]);
	yaw_bounds.setHigh(bounds.high[3]);
	this->as<ob::RealVectorStateSpace>(1)->setBounds(yaw_bounds);

	ob::RealVectorBounds camera_bounds(1);
	camera_bounds.setLow(bounds.low[4]);
	camera_bounds.setHigh(bounds.high[4]);
	this->as<ob::RealVectorStateSpace>(2)->setBounds(camera_bounds);
}

ob::State* DroneStateSpace::allocState() const {
	StateType *state = new StateType();
	this->allocStateComponents(state);

	return state;
}

void DroneStateSpace::StateFromConfiguration(ob::State *state, const ConfigPtr config) {
	DroneStateSpace::StateType *s = state->as<DroneStateSpace::StateType>();

	s->SetPosition(config->Position());
	s->SetYaw(config->Yaw());
	s->SetCameraAngle(config->CameraAngle());
}

DroneStateSpace::StateType::StateType() :
ob::CompoundStateSpace::StateType() {
	// nothing for now
}

void DroneStateSpace::StateType::SetPosition(const Vec3& pos) {
    this->as<RealState>(0)->values[0] = pos[0];
    this->as<RealState>(0)->values[1] = pos[1];
    this->as<RealState>(0)->values[2] = pos[2];

}

void DroneStateSpace::StateType::SetYaw(const RealNum& yaw) {
    this->as<RealState>(1)->values[0] = yaw;
}

void DroneStateSpace::StateType::SetCameraAngle(const RealNum& angle) {
    this->as<RealState>(2)->values[0] = angle;
}

Vec3 DroneStateSpace::StateType::Position() const {
    Vec3 pos;
    pos[0] = this->as<RealState>(0)->values[0];
    pos[1] = this->as<RealState>(0)->values[1];
    pos[2] = this->as<RealState>(0)->values[2];

    return pos;
}

RealNum DroneStateSpace::StateType::Yaw() const {
    return this->as<RealState>(1)->values[0];
}

RealNum DroneStateSpace::StateType::CameraAngle() const {
    return this->as<RealState>(2)->values[0];
}

void DroneStateSpace::StateType::CopyToConfig(ConfigPtr config) const {
    config->Position() = this->Position();
    config->Yaw() = this->Yaw();
    config->CameraAngle() = this->CameraAngle();
}

void DroneStateSpace::StateType::DeepCopy(const StateType *other) {
    this->SetPosition(other->Position());
    this->SetYaw(other->Yaw());
    this->SetCameraAngle(other->CameraAngle());
}

void DroneStateSpace::StateType::PrintState(std::ostream &out) const {
    out << "pos: "
    << this->as<RealState>(0)->values[0] << ", "
    << this->as<RealState>(0)->values[1] << ", "
    << this->as<RealState>(0)->values[2]
    << std::endl;

    out << "yaw: "
    << this->as<RealState>(1)->values[0] 
    << std::endl;

    out << "camera angle: "
    << this->as<RealState>(2)->values[0] 
    << std::endl;
}
