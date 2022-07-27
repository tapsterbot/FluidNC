#include "ParallelDelta.h"

#include "../Machine/MachineConfig.h"
#include "../Limits.h"  // ambiguousLimit()
#include "../Machine/Homing.h"

#include "../Protocol.h"  // protocol_execute_realtime

#include <cmath>

/*
  ==================== How it Works ====================================
  On a delta machine, Grbl axis units are in radians
  The kinematics converts the cartesian moves in gcode into
  the radians to move the arms. The Grbl motion planner never sees
  the actual cartesian values.

  To make the moves straight and smooth on a delta, the cartesian moves
  are broken into small segments where the non linearity will not be noticed.
  This is similar to how Grgl draws arcs.

  If you request MPos status it will tell you the position in
  arm angles. The MPos will report in cartesian values using forward kinematics. 
  The arm 0 values (angle) are the arms at horizontal.

  Positive angles are below horizontal.

  The machine's Z zero point in the kinematics is parallel to the arm axes.
  The offset of the Z distance from the arm axes to the end effector joints 
  at arm angle zero will be printed at startup on the serial port.

  Feedrate in gcode is in the cartesian units. This must be converted to the
  angles. This is done by calculating the segment move distance and the angle 
  move distance and applying that ration to the feedrate. 

  FYI: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
  Better: http://hypertriangle.com/~alex/delta-robot-tutorial/


Default configuration

kinematics:
  ParallelDelta:

  TODO 
   - Constrain the geometry values to realistic values.
   - Implement $MI for dynamixel motors.

*/

namespace Kinematics {

    // trigonometric constants to speed up calculations
    const float sqrt3  = 1.732050807;
    const float dtr    = M_PI / (float)180.0;  // degrees to radians
    const float sin120 = sqrt3 / 2.0;
    const float cos120 = -0.5;
    const float tan60  = sqrt3;
    const float sin30  = 0.5;
    const float tan30  = 1.0 / sqrt3;

    // the geometry of the delta
    float rf;  // radius of the fixed side (length of motor cranks)
    float re;  // radius of end effector side (length of linkages)
    float f;   // sized of fixed side triangel
    float e;   // size of end effector side triangle

    void ParallelDelta::group(Configuration::HandlerBase& handler) {
        handler.item("crank_mm", rf, 50.0, 500.0);
        handler.item("base_triangle_mm", f, 20.0, 500.0);
        handler.item("linkage_mm", re, 20.0, 500.0);
        handler.item("end_effector_triangle_mm", e, 20.0, 500.0);

        handler.item("max_negative_angle_rad", _max_negative_angle, -(M_PI / 2.0), 0.0);  // max angle up the arm can safely go
        handler.item("max_positive_angle_rad", _max_positive_angle, 0.0, (M_PI / 2.0));   // max_angle down the arm can safely go
    }

    void ParallelDelta::init() {
        float angles[MAX_N_AXIS]    = { 0.0, 0.0, 0.0 };
        float cartesian[MAX_N_AXIS] = { 0.0, 0.0, 0.0 };

        // Calculate the Z offset at the arm zero angles ...
        // Z offset is the z distance from the motor axes to the end effector axes at zero angle
        motors_to_cartesian(cartesian, angles, 3);  // Sets the cartesian values

        // print a startup message to show the kinematics are enabled. Print the offset for reference
        log_info("Kinematic system: " << name());
        log_info("  Z Offset:" << cartesian[Z_AXIS] << " Max neg angle:" << _max_negative_angle << " Max pos angle:" << _max_positive_angle);

        //     grbl_msg_sendf(CLIENT_SERIAL,
        //                    MsgLevel::Info,
        //                    "DXL_COUNT_MIN %4.0f CENTER %d MAX %4.0f PER_RAD %d",
        //                    DXL_COUNT_MIN,
        //                    DXL_CENTER,
        //                    DXL_COUNT_MAX,
        //                    DXL_COUNT_PER_RADIAN);
    }

    bool ParallelDelta::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        // Motor space is cartesian space, so we do no transform.
        return mc_move_motors(target, pl_data);
    }

    void ParallelDelta::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        //read_settings();
        //log_info("motors_to_cartesian motors: (" << motors[0] << "," << motors[1] << "," << motors[2] << ")");

        log_info("motors_to_cartesian rf:" << rf << " re:" << re << " f:" << f << " e:" << e);

        float t = (f - e) * tan30 / 2;
        //log_info("  t:" << t);

        float y1 = -(t + rf * cos(motors[0]));
        float z1 = -rf * sin(motors[0]);
        //log_info("  y1:" << y1 << " z1:" << z1);

        float y2 = (t + rf * cos(motors[1])) * sin30;
        float x2 = y2 * tan60;
        float z2 = -rf * sin(motors[1]);
        //log_info("  y2:" << y2 << " x2:" << x2 << " z2:" << z2);

        float y3 = (t + rf * cos(motors[2])) * sin30;
        float x3 = -y3 * tan60;
        float z3 = -rf * sin(motors[2]);
        //log_info("  y3:" << y3 << " x3:" << x3 << " z3:" << z3);

        float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
        //log_info("  dnm:" << dnm);

        float w1 = y1 * y1 + z1 * z1;
        float w2 = x2 * x2 + y2 * y2 + z2 * z2;
        float w3 = x3 * x3 + y3 * y3 + z3 * z3;
        //log_info("  w1:" << w1 << " w2:" << w2 << " w3:" << w3);

        // x = (a1*z + b1)/dnm
        float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
        float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;
        //log_info("  a1:" << a1 << " b1:" << b1);

        // y = (a2*z + b2)/dnm;
        float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
        float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;
        //log_info("  a2:" << a2 << " b2:" << b2);

        // a*z^2 + b*z + c = 0
        float a = a1 * a1 + a2 * a2 + dnm * dnm;
        float b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
        float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);
        //log_info("  c:" << c << " b:" << b << " c:" << c);

        // discriminant
        float d = b * b - (float)4.0 * a * c;
        //log_info("  d:" << d);
        if (d < 0) {
            log_warn("Forward Kinematics Error");
            return;
        }

        //log_info("  a:" << a << " b:" << b << " d:" << d);
        cartesian[Z_AXIS] = -(float)0.5 * (b + sqrt(d)) / a;
        cartesian[X_AXIS] = (a1 * cartesian[Z_AXIS] + b1) / dnm;
        cartesian[Y_AXIS] = (a2 * cartesian[Z_AXIS] + b2) / dnm;
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<ParallelDelta> registration("parallel_delta");
    }
}
