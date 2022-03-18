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