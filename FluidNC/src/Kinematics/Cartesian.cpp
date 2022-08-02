#include "Cartesian.h"

#include "../Machine/MachineConfig.h"

namespace Kinematics {
    void Cartesian::init() {
        log_info("Kinematic system: " << name());
    }

    bool Cartesian::kinematics_homing(AxisMask cycle_mask) {
        // Do nothing.
        return false;
    }

    void Cartesian::kinematics_post_homing() {
        // Do nothing.
    }

    bool Cartesian::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        // Motor space is cartesian space, so we do no transform.
        return mc_move_motors(target, pl_data);
    }

    void Cartesian::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        // Motor space is cartesian space, so we do no transform.
        copyAxes(cartesian, motors);
    }

    bool Cartesian::transform_cartesian_to_motors(float* motors, float* cartesian) {
        // Motor space is cartesian space, so we do no transform.
        copyAxes(motors, cartesian);
        return true;
    }

    bool Cartesian::soft_limit_error_exists(float* cartesian) {
        bool limit_error = false;

        for (int axis = 0; axis < config->_axes->_numberAxis; axis++) {
            if (config->_axes->_axis[axis]->_softLimits &&
                (cartesian[axis] < limitsMinPosition(axis) || cartesian[axis] > limitsMaxPosition(axis))) {
                String axis_letter = String(Machine::Axes::_names[axis]);
                log_info("Soft limit on " << axis_letter << " target:" << cartesian[axis]);
                limit_error = true;
            }
        }
        return !limit_error;
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<Cartesian> registration("Cartesian");
    }
}
