#include "rodos.h"
#include "topics.h"
#include "LSM9DS1.h"

#define GYROSCOPE_SCALE_FACTOR 0.0138
#define GYROSCOPE_SENSITIVITY 0.03
#define GYROSCOPE_MAX_ROTATION 0.3
#define ACCELEROMETER_OFFSET_X 20
#define ACCELEROMETER_OFFSET_Y 15
#define ACCELEROMETER_OFFSET_Z 15
#define ACCELEROMETER_SENSITIVITY 3.9
#define CALIBRATION_LOOPS 1000

static CommBuffer<bool> sensorsReadyBuffer;
static Subscriber senorsReadySubscriber(sensorsReadyTopic, sensorsReadyBuffer,
        "Sensors Ready Status");

class SensorsThread : public StaticThread<> {
public:
    explicit SensorsThread(uint64_t period) :
            StaticThread("SensorsThread"), gyroscopeCalibrationVector(0, 0, 0) {
        this->period = period;
        sensorsReadyTopic.publish(false);
    }
    
    void run() override {
        if (!init9DOF()) {
            return;
        }
        LSM9DS1::Int16Vec3d data;
        Vector3DMsg position;
        int64_t nextLoopTime = NOW();
        while (true) {
            nextLoopTime += this->period;
            sensorsReadyBuffer.getOnlyIfNewData(this->calibrated);
            if (!this->calibrated) {
                nextLoopTime += -this->period + 10 * MILLISECONDS;
                if (!calibrate()) {
                    PRINTF("Calibration failed. Restart device\n");
                    return;
                }
            } else {
                if (sensor.readAccelerometer(&data) > 0) {
                    position.x = (data.x * ACCELEROMETER_SENSITIVITY) + ACCELEROMETER_OFFSET_X;
                    position.y = (data.y * ACCELEROMETER_SENSITIVITY) + ACCELEROMETER_OFFSET_Y;
                    position.z = (data.z * ACCELEROMETER_SENSITIVITY) + ACCELEROMETER_OFFSET_Z;
                    accelerometerSensorTopic.publish(position);
                } else {
                    PRINTF("Failed to read accelerometer\n");
                }
                if (sensor.readGyroscope(&data) > 0) {
                    position.x = (data.x * GYROSCOPE_SCALE_FACTOR
                                  - gyroscopeCalibrationVector.x) * GYROSCOPE_SENSITIVITY;
                    position.y = (data.y * GYROSCOPE_SCALE_FACTOR
                                  - gyroscopeCalibrationVector.y) * GYROSCOPE_SENSITIVITY;
                    position.z = (data.z * GYROSCOPE_SCALE_FACTOR
                                  - gyroscopeCalibrationVector.z) * GYROSCOPE_SENSITIVITY;
                    position.x = std::fmax(std::fmin(
                            position.x, GYROSCOPE_MAX_ROTATION), -GYROSCOPE_MAX_ROTATION);
                    position.y = std::fmax(std::fmin(
                            position.y, GYROSCOPE_MAX_ROTATION), -GYROSCOPE_MAX_ROTATION);
                    position.z = std::fmax(std::fmin(
                            position.z, GYROSCOPE_MAX_ROTATION), -GYROSCOPE_MAX_ROTATION);
                    gyroSensorTopic.publish(position);
                } else {
                    PRINTF("Failed to read gyroscope\n");
                }
            }
            suspendCallerUntil(nextLoopTime);
        }
    }
    
    bool init9DOF() {
        int32_t initResult = sensor.init();
        if (initResult != 1) {
            PRINTF("Failed to initialize LSM9DS1 sensor: %d\n", (int) initResult);
            return false;
        }
        initResult = sensor.enableAccelerometer();
        if (initResult < 0) {
            PRINTF("Failed to initialize accelerometer sensor: %d\n", (int) initResult);
            return false;
        }
        initResult = sensor.enableGyroscope();
        if (initResult < 0) {
            PRINTF("Failed to initialize gyroscope sensor: %d\n", (int) initResult);
            return false;
        }
        return true;
    }
    
    bool calibrate() {
        if (calibrationCounter++ == 0) {
            PRINTF("Calibrating Sensors...\n");
            gyroscopeCalibrationVector.x = 0;
            gyroscopeCalibrationVector.y = 0;
            gyroscopeCalibrationVector.z = 0;
        }
        LSM9DS1::Int16Vec3d sensorData;
        if (sensor.readGyroscope(&sensorData) <= 0) {
            PRINTF("Calibration failed: Failed to read gyroscope\n");
            return false;
        }
        gyroscopeCalibrationVector.x += sensorData.x * GYROSCOPE_SCALE_FACTOR;
        gyroscopeCalibrationVector.y += sensorData.y * GYROSCOPE_SCALE_FACTOR;
        gyroscopeCalibrationVector.z += sensorData.z * GYROSCOPE_SCALE_FACTOR;
        if (calibrationCounter == CALIBRATION_LOOPS) {
            gyroscopeCalibrationVector.x /= (double) CALIBRATION_LOOPS;
            gyroscopeCalibrationVector.y /= (double) CALIBRATION_LOOPS;
            gyroscopeCalibrationVector.z /= (double) CALIBRATION_LOOPS;
            calibrationCounter = 0;
            calibrated = true;
            sensorsReadyTopic.publish(true);
            PRINTF("Sensors calibrated\n");
        }
        return true;
    }

private:
    LSM9DS1 sensor;
    uint64_t period;
    bool calibrated = false;
    int calibrationCounter = 0;
    Vector3DMsg gyroscopeCalibrationVector;
};

SensorsThread sensorsThread(50 * MILLISECONDS);
