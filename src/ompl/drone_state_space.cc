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
