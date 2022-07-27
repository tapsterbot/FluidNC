// Copyright (c) 2021 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*

	This implements Parallel Delta Kinematics

*/

#include "Kinematics.h"
#include "Cartesian.h"

// M_PI is not defined in standard C/C++ but some compilers
// support it anyway.  The following suppresses Intellisense
// problem reports.
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

#define ARM_INTERNAL_ANGLE 0.05  // radians 2.866°  // due to mounting angle
#define DXL_CENTER 2015          // (DXL_COUNTS / 2) - (ARM_INTERNAL_ANGLE * DXL_COUNT_PER_RADIAN)

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
        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        void motors_to_cartesian(float* cartesian, float* motors, int n_axis) override;

        // Configuration handlers:
        void         validate() const override {}
        virtual void group(Configuration::HandlerBase& handler) override;
        void         afterParse() override {}

        // Name of the configurable. Must match the name registered in the cpp file.
        virtual const char* name() const override { return "parallel_delta"; }

        ~ParallelDelta() {}

    private:
        //  Config items Using geometry names from the published kinematics rather than typical Fluid Style
        // To make the math easier to compare with the code
        float rf;  // The length of the crank arm on the motor
        float f;
        float re;
        float e;

        float _max_negative_angle = (M_PI / 3.0);  // default is 60 Deg up
        float _max_positive_angle = (M_PI / 2.0);  // default is 90 Deg down

    protected:
    };
}  //  namespace Kinematics
