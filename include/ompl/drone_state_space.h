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