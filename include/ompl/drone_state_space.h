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

#ifndef DRONE_STATE_SPACE_H_
#define DRONE_STATE_SPACE_H_

#include <iostream>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "global_common.h"
#include "drone_robot.h"

namespace ob = ompl::base;

class DroneStateSpace : public ob::CompoundStateSpace {
	using ConfigPtr = std::shared_ptr<drone::DroneConfig>;
public:
	DroneStateSpace();

	void setBounds(const ob::RealVectorBounds& bounds);
	virtual ob::State* allocState() const;
	void StateFromConfiguration(ob::State *state, const ConfigPtr config);

	class StateType : public ob::CompoundStateSpace::StateType {
	public:
		using RealState = ob::RealVectorStateSpace::StateType;

		StateType();

		void SetPosition(const Vec3& pos);
		void SetYaw(const RealNum& yaw);
		void SetCameraAngle(const RealNum& angle);

		Vec3 Position() const;
		RealNum Yaw() const;
		RealNum CameraAngle() const;

		void CopyToConfig(ConfigPtr config) const;

		void DeepCopy(const StateType *other);
		void PrintState(std::ostream &out) const;
	private:
	};

private:
};

#endif // DRONE_STATE_SPACE_H_