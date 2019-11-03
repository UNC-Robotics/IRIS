#ifndef CRISP_STATE_SPACE_H_
#define CRISP_STATE_SPACE_H_

#include <iostream>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

#include "global_common.h"
#include "crisp_robot.h"

namespace ob = ompl::base;

class CrispStateSpace : public ob::CompoundStateSpace {
	using ConfigPtr = std::shared_ptr<crisp::CRISPConfig>;
public:
	CrispStateSpace(const Idx num_tubes);

	virtual ob::State* allocState() const;
	void StateFromConfiguration(ob::State *state, const ConfigPtr config);
	void StateFromVector(ob::State *state, const std::vector<RealNum>& vector);

	class StateType : public ob::CompoundStateSpace::StateType {
	public:
		using SO3State = ob::SO3StateSpace::StateType;
		using RealState = ob::RealVectorStateSpace::StateType;
		using DiscreteState = ob::DiscreteStateSpace::StateType;

		StateType(const Idx num_tubes);

		void SetQuaternion(const Idx i, const Quat q);
		void SetInsertion(const Idx i, const RealNum insertion);
		void SetTipTranslation(const Vec3& trans);
		void SetTipTangent(const Vec3& tang);
		void SetKinStateVector(const crisp::KinVec& vector);
		void SetValid(const bool valid);
		void SetStability(const RealNum stable);
		void SetState(const ConfigPtr config);

		Quat Quaternion(const Idx i) const;
		RealNum Insertion(const Idx i) const;
		Vec3 TipTranslation() const;
		Vec3 TipTangent() const;
		crisp::KinVec KinStateVector() const;
		RealNum Stability()const;
		bool IsValid() const;
		void CopyToConfig(crisp::CRISPConfig *config) const;

		void DeepCopy(const StateType *other);
		void PrintState(std::ostream &out) const;
	private:
		const Idx num_tubes_;
	};

private:
	const Idx num_tubes_;
};

#endif // CRISP_STATE_SPACE_H_