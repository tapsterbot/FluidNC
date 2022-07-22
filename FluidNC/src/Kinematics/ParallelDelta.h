// Copyright (c) 2021 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*

	This implements Parallel Delta Kinematics

*/

#include "Kinematics.h"
#include "Cartesian.h"

namespace Kinematics {
    class ParallelDelta : public Cartesian {
    public:
        ParallelDelta() = default;

        ParallelDelta(const ParallelDelta&)            = delete;
        ParallelDelta(ParallelDelta&&)                 = delete;
        ParallelDelta& operator=(const ParallelDelta&) = delete;
        ParallelDelta& operator=(ParallelDelta&&)      = delete;

        // Kinematic Interface
        virtual void init() override;
        // bool         kinematics_homing(AxisMask cycle_mask) override;
        // void         kinematics_post_homing() override;
        // bool         cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        // void         motors_to_cartesian(float* cartesian, float* motors, int n_axis) override;

        // Configuration handlers:
        void         validate() const override {}
        virtual void group(Configuration::HandlerBase& handler) override;
        void         afterParse() override {}

        // Name of the configurable. Must match the name registered in the cpp file.
        virtual const char* name() const override { return "ParallelDelta"; }

        ~ParallelDelta() {}

    private:
        //  Config items
        float _crank_length             = 70;  // The length of the crank arm on the motor
        float _base_triangle            = 45;
        float _linkage_length           = 100;
        float _end_effector_triangle_mm = 100;

    protected:
    };
}  //  namespace Kinematics
