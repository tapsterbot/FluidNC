#include "ParallelDelta.h"

#include "../Machine/MachineConfig.h"
#include "../Limits.h"  // ambiguousLimit()
#include "../Machine/Homing.h"

#include "../Protocol.h"  // protocol_execute_realtime

#include <cmath>

/*
Default configuration

kinematics:
  ParallelDelta:

  TODO 
   - Constrain the geometry values to realistic values.

*/

namespace Kinematics {
    void ParallelDelta::group(Configuration::HandlerBase& handler) {
        handler.item("crank_mm", _crank_length);
        handler.item("base_triangle_mm", _base_triangle);
        handler.item("linkage_mm", _linkage_length);
        handler.item("end_effector_triangle_mm", _end_effector_triangle_mm);
    }

    void ParallelDelta::init() {
        log_info("Kinematic system: " << name());
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<ParallelDelta> registration("parallel_delta");
    }
}
