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

#ifndef CRISP_DIRECTED_CONTROL_SAMPLER_H_
#define CRISP_DIRECTED_CONTROL_SAMPLER_H_

#include <cmath>

#include <ompl/control/DirectedControlSampler.h>

#include "crisp_planner.h"
#include "ompl/crisp_control_space.h"
#include "ompl/crisp_state_space.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class CrispDirectedControlSampler : public oc::DirectedControlSampler {
public:
	using RobotPtr = std::shared_ptr<crisp::CRISPRobot>;
	using DetectorPtr = std::shared_ptr<crisp::CRISPCollisionDetector>;

	CrispDirectedControlSampler(const RobotPtr robot,
		                        const DetectorPtr detector,
		                        const oc::ControlSpace *cspace, 
		                        const oc::SpaceInformation *si, 
		                        const RealNum quat_deviation_percent, 
		                        const Rand& rng);

	void SampleStateUniform(ob::State *state);
	virtual unsigned sampleTo(oc::Control *control, const oc::Control *prev, const ob::State *source, ob::State *dest);
	virtual unsigned sampleTo(oc::Control *control, const ob::State *source, ob::State *dest);
	unsigned SampleToConsideringDistance(const ob::State *source, ob::State *dest, const RealNum dist);
	unsigned SampleToConsideringDistanceWithTiming(const ob::State *source, ob::State *dest, const RealNum dist, unsigned *time_for_fk, unsigned *time_for_cd);
	bool CheckState(const ob::State *state) const;
	bool CheckMotion(const ob::State *start, const ob::State *goal, const RealNum dist) const;

private:
	RobotPtr robot_;
	DetectorPtr detector_;
	const oc::ControlSpace *cspace_;
	// const oc::SpaceInformation *si_;
	const RealNum quat_deviation_percent_;
	Rand rng_;
	RealUniformDist uni_;
	RealNum min_roll_{-1};
	RealNum max_roll_{1};
	RealNum dx_{1e-3}; // 1e-3

	// void GenerateRandomControl(std::vector<RealNum> *insertions, std::vector<Quat> *quaternions) const;
	RealNum RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound);
	unsigned RelativeTime(const TimePoint start) const;

};

#endif // CRISP_DIRECTED_CONTROL_SAMPLER_H_