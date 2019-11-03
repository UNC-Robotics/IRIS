#ifndef CRISP_CONTROL_SPACE_H_
#define CRISP_CONTROL_SPACE_H_

#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "global_common.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class CrispControlSpace : public oc::RealVectorControlSpace {
public:
    CrispControlSpace(const ob::StateSpacePtr& state_space):
    RealVectorControlSpace(state_space, 10) {
        
        // 2 * 4(quat) + 2 * 1(insert)
        ob::RealVectorBounds bounds(10);

        for (Idx i = 0; i < 8; ++i) { //rotational dofs
            bounds.low[i] = -1;
            bounds.high[i] = 1;
        }

        for (Idx i = 8; i < 10; ++i) {
            bounds.low[i] = -0.1;
            bounds.high[i] = 0.1;
        }

        this->setBounds(bounds);
    }

    virtual oc::Control* allocControl() const {
        // do we need delete?
        ControlType* control = new ControlType();
        control->values = new double[10];
        return control;
    }

    class ControlType : public oc::RealVectorControlSpace::ControlType {
    public:
        ControlType() = default;

        void SetInsertionVelocity(const Idx tube_index, const RealNum velocity) {
            this->values[8+tube_index] = velocity;
        }

        void SetRotationVelocity(const Idx tube_index, const Quat rotation) {
            this->values[4*tube_index] = 0;
            this->values[4*tube_index+1] = rotation.x();
            this->values[4*tube_index+2] = rotation.y();
            this->values[4*tube_index+3] = rotation.z();
        }
    };
};

#endif // CRISP_CONTROL_SPACE_H_