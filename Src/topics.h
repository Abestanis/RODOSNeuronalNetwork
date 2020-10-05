#pragma once

#ifdef BUILD_FOR_GAZEBO
#  include <GazeboTopic.h>
#  include <gazebo/msgs/vector3d.pb.h>
#  include <gazebo/msgs/vector2d.pb.h>
#endif /* BUILD_FOR_GAZEBO */

#include "topic.h"
#include "matlib.h"

#ifdef BUILD_FOR_GAZEBO
#  define VECTOR3D_GET(vector3d, axis) (vector3d.axis())
typedef gazebo::msgs::Vector3d Vector3DMsg;

#  define DOUBLE_GET_VALUE(doubleMsg) (doubleMsg.x())
#  define DOUBLE_MSG_INIT(doubleMsg, value) do {doubleMsg.set_x(value); doubleMsg.set_y(0);} while (0)
typedef gazebo::msgs::Vector2d DoubleMessage;

#  define BOOL_GET_VALUE(boolMsg) (boolMsg.unused())
#  define BOOL_MSG_INIT(boolMsg, value) boolMsg.set_unused(value)
typedef gazebo::msgs::Empty BoolMessage;
#else
#  define GazeboTopic RODOS::Topic
#  define VECTOR3D_GET(vector3d, axis) (vector3d.axis)
typedef RODOS::Vector3D Vector3DMsg;

#  define DOUBLE_GET_VALUE(doubleMsg) (doubleMsg)
#  define DOUBLE_MSG_INIT(doubleMsg, value) (doubleMsg = value)
typedef double DoubleMessage;

#  define BOOL_GET_VALUE(boolMsg) (boolMsg)
#  define BOOL_MSG_INIT(boolMsg, value) (boolMsg = value)
typedef bool BoolMessage;
#endif /* BUILD_FOR_GAZEBO */

/**
 * Possible configuration of which autopilot is active.
 */
enum Autopilot : uint8_t {
    /**
     * No autopilot is active.
     */
    NONE = 0U,
    /**
     * The PID controller is active.
     */
    PID = 1U,
    /**
     * The neuronal network controller is active.
     */
    NN = 2U,
    
    /**
     * The maximum value in this enum.
     */
    MAX_VALUE = NN,
};

/**
 * Proportional, integral and derivative values.
 */
struct PIDValues {
    /**
     * The proportional factor.
     */
    double p;
    /**
     * The integral factor.
     */
    double i;
    /**
     * The derivative factor.
     */
    double d;
};

/**
 * Configuration of the PID controller.
 */
struct PIDConfig {
    /**
     * PID factors for the roll axis.
     */
    PIDValues roll;
    /**
     * PID factors for the pitch axis.
     */
    PIDValues pitch;
    /**
     * PID factors for the yaw axis.
     */
    PIDValues yaw;
};

/**
 * The target orientation of the plane.
 */
struct TargetOrientation {
    /**
     * The target roll angle in radiant.
     */
    double roll;
    /**
     * The target pitch angle in radiant.
     */
    double pitch;
    /**
     * The target yaw angle in radiant.
     */
    double yaw;
};

/// Sensor Topics
/**
 * Topic that provides sensor data from the gyroscope in degrees per second.
 */
extern GazeboTopic<Vector3DMsg> gyroSensorTopic;

/**
 * Topic that provides sensor data from the accelerometer in g.
 */
extern GazeboTopic<Vector3DMsg> accelerometerSensorTopic;

/// Servo Control Topics
/**
 * Topic that controls the target angle of the elevator servo in degrees.
 */
extern GazeboTopic<DoubleMessage> elevatorControlTopic;

/**
 * Topic that controls the target angle of the aileron servos in degrees.
 */
extern GazeboTopic<DoubleMessage> aileronsControlTopic;

/**
 * Topic that controls the target angle of the rudder servo in degrees.
 */
extern GazeboTopic<DoubleMessage> rudderControlTopic;

/**
 * Topic that propagates the planes current orientation
 * as calculated by the location processing system.
 */
extern RODOS::Topic<RODOS::Quaternion> quaternionTopic;

/**
 * Topic to set the configuration of the PID controller.
 */
extern RODOS::Topic<PIDConfig> pidConfigTopic; // TODO: Re-add this

/**
 * Topic that governs which autopilot is active.
 */
extern RODOS::Topic<Autopilot> autopilotTopic;

/**
 * Topic that publishes the target orientation of the airplane.
 */
extern RODOS::Topic<TargetOrientation> targetOrientationTopic;

/**
 * Topic that indicates if the sensors are ready to send data.
 * Can also be use to force a re-calibration of the sensors.
 */
extern GazeboTopic<BoolMessage> sensorsReadyTopic;

#ifdef PROFILE_CONTROLLERS
/**
 * Topic that propagates runtime measurements of the current active autopilot in nanoseconds.
 */
extern RODOS::Topic<int64_t> controllerProfilerTopic;
#endif /* PROFILE_CONTROLLERS */

#ifdef BUILD_FOR_GAZEBO
/**
 * Topic to controll the simulated motors in the wind tunnel during the simulation in Gazebo.
 */
extern GazeboTopic<gazebo::msgs::Vector2d> windTunnelMotorTopic;
#endif /* BUILD_FOR_GAZEBO */
