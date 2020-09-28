#include "topics.h"
#include "rodos.h"

#define ROLL_MAX 28.0
#define ROLL_MIN -17.0
#define PITCH_MAX 28.0
#define PITCH_MIN -17.0
#define YAW_MAX 28.0
#define YAW_MIN -17.0
#define INITIAL_YAW_P 30.0
#define INITIAL_YAW_I 0.0
#define INITIAL_YAW_D 0.0
#define INITIAL_PITCH_P 30.0
#define INITIAL_PITCH_I 0.1
#define INITIAL_PITCH_D 1.0
#define INITIAL_ROLL_P 32.0
#define INITIAL_ROLL_I 0.1
#define INITIAL_ROLL_D 0.0

static CommBuffer<Quaternion> orientationBuffer;
static Subscriber orientationSubscriber(
        quaternionTopic, orientationBuffer, "PID Orientation Subscriber");

static CommBuffer<PIDConfig> pidConfigBuffer;
static Subscriber pidConfigSubscriber(pidConfigTopic, pidConfigBuffer,
        "PID Config Subscriber");

static CommBuffer<Autopilot> selectedAutoPilotBuffer;
static Subscriber selectedAutoPilotSubscriber(autopilotTopic, selectedAutoPilotBuffer,
        "PID Selected Autopilot Subscriber");

static CommBuffer<TargetOrientation> targetOrientationBuffer;
static Subscriber targetOrientationSubscriber(targetOrientationTopic, targetOrientationBuffer,
        "PID Target Orientation Subscriber");


class PIDControl : public StaticThread<> {
public:
    
    explicit PIDControl(int64_t period);
    
    [[noreturn]] void run() override;

private:
    void rollControl(double roll, double targetRoll);
    
    void yawControl(double yaw, double targetYaw);
    
    void pitchControl(double pitch, double targetPitch);
    
    static inline double clamp(double value, double min, double max);
    
    double rollIntegration, pitchIntegration, yawIntegration;
    double prevRoll, prevYaw, prevPitch;
    
    int64_t period;
    PIDConfig config = {};
};

PIDControl::PIDControl(int64_t period) : StaticThread("PID-Controller") {
    this->period = period;
    
    rollIntegration = 0.0;
    pitchIntegration = 0.0;
    yawIntegration = 0.0;
    prevRoll = 0.0;
    prevPitch = 0.0;
    prevYaw = 0.0;
    
    config.roll.p = INITIAL_ROLL_P;
    config.yaw.p = INITIAL_YAW_P;
    config.pitch.p = INITIAL_PITCH_P;
    
    config.roll.i = INITIAL_ROLL_I;
    config.yaw.i = INITIAL_YAW_I;
    config.pitch.i = INITIAL_PITCH_I;
    
    config.roll.d = INITIAL_ROLL_D;
    config.yaw.d = INITIAL_YAW_D;
    config.pitch.d = INITIAL_PITCH_D;
}

[[noreturn]] void PIDControl::run() {
    Autopilot selectedAutopilot = Autopilot::NONE;
    Quaternion orientationQuaternion;
    YPR orientation;
    TargetOrientation targetOrientation = {.roll = 0, .pitch = 0, .yaw = 0};
    int64_t nextTime = NOW();
    
    while (true) {
        nextTime += period;
        selectedAutoPilotBuffer.getOnlyIfNewData(selectedAutopilot);
        if (selectedAutopilot == Autopilot::PID) {
            pidConfigBuffer.getOnlyIfNewData(config);
            orientationBuffer.get(orientationQuaternion);
            targetOrientationBuffer.getOnlyIfNewData(targetOrientation);
            orientation = orientationQuaternion.toYPR();
#ifdef PROFILE_CONTROLLERS
            int64_t startTime = NOW();
#endif /* PROFILE_CONTROLLERS */
            rollControl(orientation.roll, targetOrientation.roll);
            yawControl(orientation.yaw, targetOrientation.yaw);
            pitchControl(orientation.pitch, targetOrientation.pitch);
#ifdef PROFILE_CONTROLLERS
            controllerProfiler.publish(NOW() - startTime);
#endif /* PROFILE_CONTROLLERS */
        }
        suspendCallerUntil(nextTime);
    }
}

void PIDControl::rollControl(double roll, double targetRoll) {
    double error = targetRoll - roll;
    
    // Calculate the I-Term
    rollIntegration += (error * config.roll.i);
    rollIntegration = clamp(rollIntegration, PITCH_MIN, PITCH_MAX);
    
    // Calculate the D-Term
    double rollChange = roll - prevRoll;
    
    // Combine the PID terms
    double output = (config.roll.p * error) + rollIntegration - config.roll.d * rollChange;
    output = clamp(output, PITCH_MIN, PITCH_MAX);
    
    DoubleMessage outputMessage;
    DOUBLE_MSG_INIT(outputMessage, output);
    aileronsControlTopic.publish(outputMessage);
    prevRoll = roll;
}

void PIDControl::yawControl(double yaw, double targetYaw) {
    double error = targetYaw - yaw;
    
    // Calculate the I-Term
    yawIntegration = yawIntegration + (error * config.yaw.i);
    yawIntegration = clamp(yawIntegration, YAW_MIN, YAW_MAX);
    
    // Calculate the D-Term
    double yawChange = yaw - prevYaw;
    
    // Combine the PID terms
    double output = (config.yaw.p * error) + yawIntegration - config.yaw.d * yawChange;
    output = clamp(output, YAW_MIN, YAW_MAX);
    
    DoubleMessage outputMessage;
    DOUBLE_MSG_INIT(outputMessage, output);
    rudderControlTopic.publish(outputMessage);
    prevYaw = yaw;
}

void PIDControl::pitchControl(double pitch, double targetPitch) {
    double error = targetPitch - pitch;
    
    // Calculate the I-Term
    pitchIntegration = pitchIntegration + (error * config.pitch.i);
    pitchIntegration = clamp(pitchIntegration, PITCH_MIN, PITCH_MAX);
    
    // Calculate the D-Term
    double pitchChange = pitch - prevPitch;
    
    // Combine the PID terms
    double output = (config.pitch.p * error) + pitchIntegration - config.pitch.d * pitchChange;
    output = clamp(output, PITCH_MIN, PITCH_MAX);
    
    DoubleMessage outputMessage;
    DOUBLE_MSG_INIT(outputMessage, output);
    elevatorControlTopic.publish(outputMessage);
    prevPitch = pitch;
}

double PIDControl::clamp(double value, double min, double max) {
    return std::fmin(std::fmax(value, min), max);
}

PIDControl pidControl(50 * MILLISECONDS);
