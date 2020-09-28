#include <rodos.h>
#include "topics.h"
#include "app_x-cube-ai.h"

extern ai_buffer ai_input[AI_NETWORK_IN_NUM];

static CommBuffer<Quaternion> orientationBuffer;
static Subscriber orientationSubscriber(
        quaternionTopic, orientationBuffer, "PID Orientation Subscriber");

static CommBuffer<Autopilot> selectedAutoPilotBuffer;
static Subscriber selectedAutoPilotSubscriber(autopilotTopic, selectedAutoPilotBuffer,
        "PID Selected Autopilot Subscriber");

static CommBuffer<TargetOrientation> targetOrientationBuffer;
static Subscriber targetOrientationSubscriber(targetOrientationTopic, targetOrientationBuffer,
        "PID Target Orientation Subscriber");

static Vector3DMsg gyroscopeData;
static Vector3DMsg accelerometerData;


static class GyroscopeSubscriber : public Subscriber {
public:
    GyroscopeSubscriber() :
            Subscriber(gyroSensorTopic, "NN-Controller Gyroscope-Subscriber") {
    }

protected:
    uint32_t put(const uint32_t topicId, const size_t len, void* data,
                 const NetMsgInfo &netMsgInfo) override {
        gyroscopeData = *(Vector3DMsg*) data;
        return 1;
    }
} nnGyroscopeSubscriber;

static class AccelerometerSubscriber : public Subscriber {
public:
    AccelerometerSubscriber() :
            Subscriber(accelerometerSensorTopic, "NN-Controller Accelerometer-Subscriber") {
    }

protected:
    uint32_t put(const uint32_t topicId, const size_t len, void* data,
                 const NetMsgInfo &netMsgInfo) override {
        accelerometerData = *(Vector3DMsg*) data;
        return 1;
    }
} nnAccelerometerSubscriber;

class NNController : public StaticThread<> {
public:
    NNController(int64_t period) : StaticThread("NNController") {
        this->period = period;
    }
    
    void init() override {
        AI_ALIGNED(4)
        static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
        aiInit(activations);
    }
    
    [[noreturn]] void run() override {
        AI_ALIGNED(4)
        static ai_i8 sensorData[AI_NETWORK_IN_1_SIZE_BYTES];
        
        AI_ALIGNED(4)
        static ai_i8 nnOutput[AI_NETWORK_OUT_1_SIZE_BYTES];
        
        const ai_buffer_format bufferFormat = AI_BUFFER_FORMAT(&ai_input[0]);
        
        Autopilot selectedAutopilot = Autopilot::NONE;
        Quaternion orientationQuaternion;
        YPR orientation;
        TargetOrientation targetOrientation = {.roll = 0, .pitch = 0, .yaw = 0};
        int64_t nextTime = NOW();
        
        while (true) {
            nextTime += period;
            selectedAutoPilotBuffer.getOnlyIfNewData(selectedAutopilot);
            if (selectedAutopilot == Autopilot::NN) {
                orientationBuffer.get(orientationQuaternion);
                targetOrientationBuffer.getOnlyIfNewData(targetOrientation);
                orientation = orientationQuaternion.toYPR();
                setDouble(sensorData, 0, gyroscopeData.x, bufferFormat);
                setDouble(sensorData, 1, gyroscopeData.y, bufferFormat);
                setDouble(sensorData, 2, gyroscopeData.z, bufferFormat);
                setDouble(sensorData, 3, accelerometerData.x, bufferFormat);
                setDouble(sensorData, 4, accelerometerData.y, bufferFormat);
                setDouble(sensorData, 5, accelerometerData.z, bufferFormat);
                setDouble(sensorData, 6, orientation.roll, bufferFormat);
                setDouble(sensorData, 7, orientation.yaw, bufferFormat);
                setDouble(sensorData, 8, orientation.pitch, bufferFormat);
                setDouble(sensorData, 9, targetOrientation.roll, bufferFormat);
                setDouble(sensorData, 10, targetOrientation.yaw, bufferFormat);
                setDouble(sensorData, 11, targetOrientation.pitch, bufferFormat);

#ifdef PROFILE_CONTROLLERS
                int64_t startTime = NOW();
#endif /* PROFILE_CONTROLLERS */
                if (aiRun(sensorData, nnOutput)) {
                    PRINTF("NN Result : Error\n");
                    continue;
                }
#ifdef PROFILE_CONTROLLERS
                controllerProfiler.publish(NOW() - startTime);
#endif /* PROFILE_CONTROLLERS */
                
                elevatorControlTopic.publish(getDouble(nnOutput, 0, bufferFormat));
                aileronsControlTopic.publish(getDouble(nnOutput, 1, bufferFormat));
                rudderControlTopic.publish(getDouble(nnOutput, 2, bufferFormat));
            }
            suspendCallerUntil(nextTime);
        }
    }

private:
    
    static void setDouble(ai_i8* buffer, size_t index, double value,
                          ai_buffer_format bufferFormat) {
        if (AI_BUFFER_FMT_GET_TYPE(bufferFormat) == AI_BUFFER_FMT_TYPE_FLOAT) {
            ((ai_float*) buffer)[index] = (ai_float) value;
        } else { /* AI_BUFFER_FMT_TYPE_Q */
            /* Prepare parameters for double to Qmn conversion */
            static const ai_i16 N = AI_BUFFER_FMT_GET_FBITS(bufferFormat);
            static const ai_float scale = (0x1U << N);
            static const ai_i16 M = AI_BUFFER_FMT_GET_BITS(bufferFormat)
                                    - AI_BUFFER_FMT_GET_SIGN(bufferFormat) - N;
            static const ai_float maxValue = (ai_float) (0x1U << M);
            
            value *= maxValue;
            /* Convert double to Qmn format */
            const ai_i32 tmp = AI_ROUND(value * scale, ai_i32);
            buffer[index] = AI_CLAMP(tmp, -128, 127, ai_i8);
        }
    }
    
    static double getDouble(ai_i8* buffer, size_t index, ai_buffer_format bufferFormat) {
        if (AI_BUFFER_FMT_GET_TYPE(bufferFormat) == AI_BUFFER_FMT_TYPE_FLOAT) {
            return ((ai_float*) buffer)[index];
        } else { /* AI_BUFFER_FMT_TYPE_Q */
            /* Prepare parameters for Qmn to souble conversion */
            static const ai_i16 N = AI_BUFFER_FMT_GET_FBITS(bufferFormat);
            static const ai_float scale = (0x1U << N);
            static const ai_i16 M = AI_BUFFER_FMT_GET_BITS(bufferFormat)
                                    - AI_BUFFER_FMT_GET_SIGN(bufferFormat) - N;
            static const ai_float maxValue = (ai_float) (0x1U << M);
            return ((ai_float) buffer[index] / scale) / maxValue;
        }
    }
    
    int64_t period;
};

NNController nnController(50 * MILLISECONDS);
