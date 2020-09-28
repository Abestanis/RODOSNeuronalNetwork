#include "topics.h"
#include "rodos.h"

// Sensor Topics
GazeboTopic<Vector3DMsg> gyroSensorTopic(-1, "Gyroscope");
GazeboTopic<Vector3DMsg> accelerometerSensorTopic(-1, "Accelerometer");

// Servo Control Topics
GazeboTopic<DoubleMessage> elevatorControlTopic(-1, "Elevator Servo");
GazeboTopic<DoubleMessage> aileronsControlTopic(-1, "Ailerons Servo");
GazeboTopic<DoubleMessage> rudderControlTopic(-1, "Rudder Servo");

Topic<Quaternion> quaternionTopic(-1, "Orientation");
Topic<PIDConfig> pidConfigTopic(-1, "PID-Controller Config");
Topic<Autopilot> autopilotTopic(-1, "ControlState AutoPilot");
Topic<TargetOrientation> targetOrientationTopic(-1, "Target Orientation");

GazeboTopic<BoolMessage> sensorsReadyTopic(-1, "Sensors Ready Flag");

#ifdef PROFILE_CONTROLLERS
Topic<int64_t> controllerProfiler(-1, "Controller Profiler");
#endif /* PROFILE_CONTROLLERS */

#ifdef BUILD_FOR_GAZEBO
GazeboTopic<gazebo::msgs::Vector2d> windTunnelMotorTopic(-1, "Wind Tunnel Motor Speed");
#endif /* BUILD_FOR_GAZEBO */


void MAIN() {
}
