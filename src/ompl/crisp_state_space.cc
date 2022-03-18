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

#include "ompl/crisp_state_space.h"

CrispStateSpace::CrispStateSpace(const Idx num_tubes) :
ob::CompoundStateSpace(), num_tubes_(num_tubes) 
{
	RealNum w = 1e-3;
	RealNum rot_weight = w/num_tubes;
	RealNum trans_weight = (1 - w)/num_tubes;
	RealNum none_weight = 0.0;

	for (Idx i = 0; i < num_tubes; ++i) {
		ob::StateSpacePtr rot_space(new ob::SO3StateSpace());
		this->addSubspace(rot_space, rot_weight);
	}

	for (Idx i = 0; i < num_tubes; ++i) {
		ob::StateSpacePtr trans_space(new ob::RealVectorStateSpace(1));
		this->addSubspace(trans_space, trans_weight);
	}

	ob::StateSpacePtr tip_trans(new ob::RealVectorStateSpace(3));
	this->addSubspace(tip_trans, none_weight);

	ob::StateSpacePtr tip_tang(new ob::RealVectorStateSpace(3));
	this->addSubspace(tip_tang, none_weight);

	ob::StateSpacePtr kin_state(new ob::RealVectorStateSpace(AUGMENTED_DIMENSION));
	this->addSubspace(kin_state, none_weight);

	ob::StateSpacePtr stability(new ob::RealVectorStateSpace(1));
	this->addSubspace(stability, none_weight);

	ob::StateSpacePtr valid(new ob::DiscreteStateSpace(0, 1));
	this->addSubspace(valid, none_weight);

	// bounds
	for (Idx i = 0; i < num_tubes; ++i) {
		ob::RealVectorBounds trans_bound(1);
		trans_bound.setHigh(0.5);
		trans_bound.setLow(0.0);
		this->as<ob::RealVectorStateSpace>(num_tubes + i)->setBounds(trans_bound);
	}

	ob::RealVectorBounds tip_trans_bound(3);
	tip_trans_bound.setHigh(1);
	tip_trans_bound.setLow(-1);
	this->as<ob::RealVectorStateSpace>(2*num_tubes)->setBounds(tip_trans_bound);

	ob::RealVectorBounds tip_tang_bound(3);
	tip_tang_bound.setHigh(1);
	tip_tang_bound.setLow(-1);
	this->as<ob::RealVectorStateSpace>(2*num_tubes + 1)->setBounds(tip_tang_bound);

	ob::RealVectorBounds kinematic_state_bound(AUGMENTED_DIMENSION);
	kinematic_state_bound.setHigh(10);
	kinematic_state_bound.setLow(-10);
	this->as<ob::RealVectorStateSpace>(2*num_tubes + 2)->setBounds(kinematic_state_bound);

	ob::RealVectorBounds stability_bound(1);
	stability_bound.setHigh(100);
	stability_bound.setLow(0);
	this->as<ob::RealVectorStateSpace>(2*num_tubes + 3)->setBounds(stability_bound);
}

ob::State* CrispStateSpace::allocState() const {
	StateType *state = new StateType(num_tubes_);
	this->allocStateComponents(state);

	return state;
}

void CrispStateSpace::StateFromConfiguration(ob::State *state, const ConfigPtr config) {
	CrispStateSpace::StateType *s = state->as<CrispStateSpace::StateType>();

	for (Idx i = 0; i < num_tubes_; ++i) {
		s->SetInsertion(i, config->TubeInsertion(i));
		s->SetQuaternion(i, config->TubeOrientation(i));
	}

	s->SetTipTranslation(config->TipTranslation());
	s->SetTipTangent(config->TipTangent());
	s->SetKinStateVector(config->KinematicState());
	s->SetStability(config->Stability());
	s->SetValid(true); // default at this time
}

void CrispStateSpace::StateFromVector(ob::State *state, const std::vector<RealNum>& vector) {
	CrispStateSpace::StateType *s = state->as<CrispStateSpace::StateType>();

	for (Idx i = 0; i < num_tubes_; ++i) {
		s->SetInsertion(i, vector[i]);
		Idx base = num_tubes_ + 4*i;
		s->SetQuaternion(i, Quat(vector[base], vector[base+1], vector[base+2], vector[base+3]));
	}

	Idx base = 5*num_tubes_;
	s->SetTipTranslation(Vec3(vector[base], vector[base+1], vector[base+2]));
	s->SetTipTangent(Vec3(vector[base+3], vector[base+4], vector[base+5]));

	base = 5*num_tubes_ + 6;
	crisp::KinVec kin_vec;
	for (Idx i = 0; i < AUGMENTED_DIMENSION; ++i) {
		kin_vec(i) = vector[base+i];
	}
	s->SetKinStateVector(kin_vec);
	s->SetStability(vector[5*num_tubes_ + 6 + AUGMENTED_DIMENSION]);
	s->SetValid(true); // default at this time
}

CrispStateSpace::StateType::StateType(const Idx num_tubes) :
ob::CompoundStateSpace::StateType(), num_tubes_(num_tubes) {
	// nothing for now
}

void CrispStateSpace::StateType::SetQuaternion(const Idx i, const Quat q) {
	this->as<SO3State>(i)->w = q.w();
	this->as<SO3State>(i)->x = q.x();
	this->as<SO3State>(i)->y = q.y();
	this->as<SO3State>(i)->z = q.z();
}

void CrispStateSpace::StateType::SetInsertion(const Idx i, const RealNum insertion) {
	this->as<RealState>(num_tubes_ + i)->values[0] = insertion;
}

void CrispStateSpace::StateType::SetTipTranslation(const Vec3& trans) {
	for (Idx i = 0; i < 3; ++i) {
		this->as<RealState>(2*num_tubes_)->values[i] = trans(i);
	}
}

void CrispStateSpace::StateType::SetTipTangent(const Vec3& tang) {
	for (Idx i = 0; i < 3; ++i) {
		this->as<RealState>(2*num_tubes_ + 1)->values[i] = tang(i);
	}
}

void CrispStateSpace::StateType::SetKinStateVector(const crisp::KinVec& vector) {
	for (Idx i = 0; i < AUGMENTED_DIMENSION; ++i) {
		this->as<RealState>(2*num_tubes_ + 2)->values[i] = vector(i);
	}
}

void CrispStateSpace::StateType::SetStability(const RealNum stable) {
	this->as<RealState>(2*num_tubes_ + 3)->values[0] = stable;
}

void CrispStateSpace::StateType::SetValid(const bool valid) {
	this->as<DiscreteState>(2*num_tubes_ + 4)->value = (valid)? 1 : 0;
}

void CrispStateSpace::StateType::SetState(const ConfigPtr config) {
	for (Idx i = 0; i < num_tubes_; ++i) {
		this->SetInsertion(i, config->TubeInsertion(i));
		this->SetQuaternion(i, config->TubeOrientation(i));
	}

	this->SetTipTranslation(config->TipTranslation());
	this->SetTipTangent(config->TipTangent());
	this->SetKinStateVector(config->KinematicState());
	this->SetStability(config->Stability());
	this->SetValid(config->IsValid()); // only valid state will be copied
}

Quat CrispStateSpace::StateType::Quaternion(const Idx i) const {
	Quat q;
	q.w() = this->as<SO3State>(i)->w;
	q.x() = this->as<SO3State>(i)->x;
	q.y() = this->as<SO3State>(i)->y;
	q.z() = this->as<SO3State>(i)->z;

	return q.normalized();
}

RealNum CrispStateSpace::StateType::Insertion(const Idx i) const {
	return this->as<RealState>(num_tubes_ + i)->values[0];
}

Vec3 CrispStateSpace::StateType::TipTranslation() const {
	Vec3 v;
	for (Idx i = 0; i < 3; ++i) {
		v(i) = this->as<RealState>(2*num_tubes_)->values[i];
	}

	return v;
}

Vec3 CrispStateSpace::StateType::TipTangent() const {
	Vec3 v;
	for (Idx i = 0; i < 3; ++i) {
		v(i) = this->as<RealState>(2*num_tubes_ + 1)->values[i];
	}

	return v;
}
crisp::KinVec CrispStateSpace::StateType::KinStateVector() const {
	crisp::KinVec v;
	for (Idx i = 0; i < AUGMENTED_DIMENSION; ++i) {
		v(i) = this->as<RealState>(2*num_tubes_ + 2)->values[i];
	}

	return v;
}

RealNum CrispStateSpace::StateType::Stability() const {
	return this->as<RealState>(2*num_tubes_ + 3)->values[0];
}

bool CrispStateSpace::StateType::IsValid() const {
	return (this->as<DiscreteState>(2*num_tubes_ + 4)->value == 1);
}

void CrispStateSpace::StateType::CopyToConfig(crisp::CRISPConfig *config) const {
	for (Idx i = 0; i < NUM_TUBES; ++i) {
		config->TubeInsertion(i) = this->Insertion(i);
		config->TubeOrientation(i) = this->Quaternion(i);
	}

	config->TipTranslation() = this->TipTranslation();
	config->TipTangent() = this->TipTangent();
	config->KinematicState() = this->KinStateVector();
	config->Stability() = this->Stability();
	config->IsValid() = this->IsValid();
}

void CrispStateSpace::StateType::DeepCopy(const CrispStateSpace::StateType *other) {
	for (Idx i = 0; i < num_tubes_; ++i) {
		this->SetQuaternion(i, other->Quaternion(i));
		this->SetInsertion(i, other->Insertion(i));
	}

	this->SetTipTranslation(other->TipTranslation());
	this->SetTipTangent(other->TipTangent());
	this->SetKinStateVector(other->KinStateVector());
	this->SetStability(other->Stability());
	this->SetValid(other->IsValid());
}

void CrispStateSpace::StateType::PrintState(std::ostream &out) const {
	for (Idx i = 0; i < num_tubes_; ++i) {
        out << "Tube " << i << std::endl;
        out << "insertion = " << this->Insertion(i) << std::endl;
        out << "orientation = " << this->Quaternion(i).w()
            << this->Quaternion(i).vec().transpose() << std::endl;
    }

    out << "Tip translation = " << this->TipTranslation().transpose() << std::endl;
    out << "Tip tangent = " << this->TipTangent().transpose() << std::endl;

    out << "Kinematic state = ";
    auto kin_vec = this->KinStateVector();
    for (Idx i = 0; i < AUGMENTED_DIMENSION; ++i) {
        out << kin_vec(i) << " ";
    }
    out << std::endl;

    out << "Stability = " << this->Stability() << std::endl;
    out << "If valid = " << this->IsValid() << std::endl;
}
