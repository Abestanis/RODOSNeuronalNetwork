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

enum Autopilot : uint8_t {
    NONE = 0U,
    PID = 1U,
    NN = 2U,
    
    MAX_VALUE = NN,
};

struct PIDValues {
    double p, i, d;
};

struct PIDConfig {
    PIDValues roll, pitch, yaw;
};

struct TargetOrientation {
    double roll, pitch, yaw;
};

// Sensor Topics
extern GazeboTopic<Vector3DMsg> gyroSensorTopic;
extern GazeboTopic<Vector3DMsg> accelerometerSensorTopic;

// Servo Control Topics
extern GazeboTopic<DoubleMessage> elevatorControlTopic;
extern GazeboTopic<DoubleMessage> aileronsControlTopic;
extern GazeboTopic<DoubleMessage> rudderControlTopic;

extern RODOS::Topic<RODOS::Quaternion> quaternionTopic;
extern RODOS::Topic<PIDConfig> pidConfigTopic;

extern RODOS::Topic<Autopilot> autopilotTopic;
extern RODOS::Topic<TargetOrientation> targetOrientationTopic;
extern GazeboTopic<BoolMessage> sensorsReadyTopic;

#ifdef PROFILE_CONTROLLERS
extern RODOS::Topic<int64_t> controllerProfiler;
#endif /* PROFILE_CONTROLLERS */

#ifdef BUILD_FOR_GAZEBO
extern GazeboTopic<gazebo::msgs::Vector2d> windTunnelMotorTopic;
#endif /* BUILD_FOR_GAZEBO */
