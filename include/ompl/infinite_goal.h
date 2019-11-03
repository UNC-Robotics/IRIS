#ifndef INFINITE_GOAL_H_
#define INFINITE_GOAL_H_

#include <ompl/base/goals/GoalRegion.h>

namespace ob = ompl::base;

class InfiniteGoal : public ob::GoalRegion {
public:
    InfiniteGoal(const ob::SpaceInformationPtr &si) :
    ompl::base::GoalRegion(si) {
        // nothing 
    }

    virtual double distanceGoal(const ob::State *si) const {
        return 1e9;
    }
};

#endif // INFINITE_GOAL_H_