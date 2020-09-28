#include "topics.h"
#include "rodos.h"
#include "matlib.h"

#define RAD_TO_DEGREE_FACTOR (180.0 / M_PI)
#define ANGLE_TO_RAD_FACTOR (M_PI / 180.0)
#define COMP_FILTER_ROT 0.985

static Vector3DMsg gyroscopeData;
static Vector3DMsg accelerometerData;


static CommBuffer<BoolMessage> sensorsReadyBuffer;
static Subscriber senorsReadySubscriber(sensorsReadyTopic, sensorsReadyBuffer,
        "LocationProcessing SensorsReady-Status-Subscriber");

static class GyroscopeSubscriber : public Subscriber {
public:
    GyroscopeSubscriber() :
            Subscriber(gyroSensorTopic, "LocationProcessing Gyroscope-Subscriber") {
    }

protected:
    uint32_t put(const uint32_t topicId, const size_t len, void* data,
                 const NetMsgInfo &netMsgInfo) override {
        gyroscopeData = *(Vector3DMsg*) data;
        return 1;
    }
} gyroscopeSubscriber;

static class AccelerometerSubscriber : public Subscriber {
public:
    AccelerometerSubscriber() :
            Subscriber(accelerometerSensorTopic, "LocationProcessing Accelerometer-Subscriber") {
    }

protected:
    uint32_t put(const uint32_t topicId, const size_t len, void* data,
                 const NetMsgInfo &netMsgInfo) override {
        accelerometerData = *(Vector3DMsg*) data;
        return 1;
    }
} accelerometerSubscriber;

class LocationProcessingThread : public StaticThread<> {
public:
    explicit LocationProcessingThread(uint64_t period) : StaticThread("LocationProcessingThread") {
        this->period = period;
        updateRateFactor = ((double) SECONDS) / period;
    }
    
    [[noreturn]] void run() override {
        int64_t nextTime = NOW();
        BoolMessage sensorsReady;
        BOOL_MSG_INIT(sensorsReady, false);
        while (true) {
            nextTime += this->period;
            sensorsReadyBuffer.getOnlyIfNewData(sensorsReady);
            if (!BOOL_GET_VALUE(sensorsReady)) {
                quaternion = Quaternion();
            } else {
                calculateQuaternion();
                //                auto angles = quaternion.toRPY();
                //                PRINTF("Yaw %f, Pitch %f, Roll %f\n", angles.getYaw() * RAD_TO_DEGREE_FACTOR,
                //                        angles.getPitch() * RAD_TO_DEGREE_FACTOR,
                //                        angles.getRoll() * RAD_TO_DEGREE_FACTOR);
            }
            quaternionTopic.publish(quaternion);
            suspendCallerUntil(nextTime);
        }
    }

private:
    void calculateQuaternion() {
        YPR orientation = quaternion.toYPR();
        orientation.yaw +=
                ANGLE_TO_RAD_FACTOR * (VECTOR3D_GET(gyroscopeData, z) / updateRateFactor);
        orientation.pitch +=
                ANGLE_TO_RAD_FACTOR * (VECTOR3D_GET(gyroscopeData, y) / updateRateFactor);
        orientation.roll +=
                ANGLE_TO_RAD_FACTOR * ((-1) * VECTOR3D_GET(gyroscopeData, x) / updateRateFactor);
        YPR accelerometerYPR = fromVectorToYPR(accelerometerData);
        orientation.pitch = (accelerometerYPR.pitch * (1.0 - COMP_FILTER_ROT)) +
                            (orientation.pitch * COMP_FILTER_ROT);
        orientation.roll = (accelerometerYPR.roll * (1.0 - COMP_FILTER_ROT)) +
                           (orientation.roll * COMP_FILTER_ROT);
        quaternion = orientation.toQuaternion();
    }
    
    static YPR fromVectorToYPR(const Vector3DMsg &vector) {
        YPR tempYPR;
        if (VECTOR3D_GET(vector, z) != 0.0) {
            tempYPR.pitch = atan(VECTOR3D_GET(vector, x) / (
                    sqrt(pow(VECTOR3D_GET(vector, y), 2) + pow(VECTOR3D_GET(vector, z), 2))));
            tempYPR.roll = atan(VECTOR3D_GET(vector, y) / (
                    sqrt(pow(VECTOR3D_GET(vector, x), 2) + pow(VECTOR3D_GET(vector, z), 2))));
            tempYPR.yaw = atan(VECTOR3D_GET(vector, x) / (
                    sqrt(pow(VECTOR3D_GET(vector, z), 2) + pow(VECTOR3D_GET(vector, y), 2))));
        }
        return tempYPR;
    }
    
    Quaternion quaternion;
    uint64_t period;
    double updateRateFactor;
};

LocationProcessingThread locationProcessingThread(50 * MILLISECONDS);
