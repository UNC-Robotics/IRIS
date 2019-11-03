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