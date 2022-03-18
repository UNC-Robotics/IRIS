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