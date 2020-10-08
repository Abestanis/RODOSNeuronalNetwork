#include "rodos.h"
#include "topics.h"
#include "LSM9DS1.h"

/**
 * Scale factor for the raw gyroscope sensor data.
 */
#define GYROSCOPE_SCALE_FACTOR 0.0138
/**
 * Gyroscope sensitivity factor.
 */
#define GYROSCOPE_SENSITIVITY 0.03
/**
 * Maximum allowed gyroscope sensor measurement in degrees per second.
 */
#define GYROSCOPE_MAX_ROTATION 0.3
/**
 * Static accelerometer calibration x axis offset.
 */
#define ACCELEROMETER_OFFSET_X 20
/**
 * Static accelerometer calibration y axis offset.
 */
#define ACCELEROMETER_OFFSET_Y 15
/**
 * Static accelerometer calibration z axis offset.
 */
#define ACCELEROMETER_OFFSET_Z 15
/**
 * Accelerometer sensitivity factor.
 */
#define ACCELEROMETER_SENSITIVITY 3.9
/**
 * Period of the sensor calibration loop.
 */
#define CALIBRATION_LOOP_PERIOD (10 * MILLISECONDS)
/**
 * How many measurements to take for the calibration.
 */
#define CALIBRATION_LOOPS 1000

/**
 * Buffer that holds the last value of the sensor ready topic.
 */
static CommBuffer<bool> sensorsReadyBuffer;
/**
 * Subscriber to the sensor ready topic, which writes the latest value in the buffer.
 */
static Subscriber senorsReadySubscriber(sensorsReadyTopic, sensorsReadyBuffer,
        "Sensors Ready Status");

/**
 * A thread that reads data from the sensors and publishes the data on topics.
 */
class SensorsThread : public StaticThread<> {
public:
    /**
     * Create a new sensor thread.
     * @param period The update period of the sensor read loop.
     */
    explicit SensorsThread(uint64_t period) :
            StaticThread("SensorsThread"), gyroscopeCalibrationVector(0, 0, 0) {
        this->period = period;
        sensorsReadyTopic.publish(false);
    }
    
    /**
     * Initialize the sensor and run the main and calibration loop.
     */
    void run() override {
        if (!init9DOF()) {
            return;
        }
        LSM9DS1::Int16Vec3d data;
        Vector3DMsg position;
        int64_t nextLoopTime = NOW();
        while (true) {
            sensorsReadyBuffer.getOnlyIfNewData(this->calibrated);
            if (!this->calibrated) {
                nextLoopTime += CALIBRATION_LOOP_PERIOD;
                if (!calibrate()) {
                    PRINTF("Calibration failed. Restart device\n");
                    return;
                }
            } else {
                nextLoopTime += this->period;
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
    
    /**
     * Initialize the LSM9DS1 sensor.
     * @return Whether the initialization was successful.
     */
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
    
    /**
     * Execute a calibration step.
     * @return Whether or not the calibration step was successful.
     */
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
        if (calibrationCounter >= CALIBRATION_LOOPS) {
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
    /**
     * The server that provides accelerometer and gyroscope data.
     */
    LSM9DS1 sensor;
    /**
     * The update period of the sensor read loop.
     */
    uint64_t period;
    /**
     * Whether or not the sensors are calibrated.
     */
    bool calibrated = false;
    /**
     * A counter that counts the number of executed calibration steps.
     */
    int calibrationCounter = 0;
    /**
     * The gyroscope sensor calibration offsets.
     */
    Vector3DMsg gyroscopeCalibrationVector;
};

/**
 * The thread that provides sensor data.
 */
SensorsThread sensorsThread(50 * MILLISECONDS);
