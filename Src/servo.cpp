#include "rodos.h"
#include "topics.h"
#include "stm32f4xx_conf.h"

/**
 * The frequency that the pulse width modulation can be changed in hertz.
 */
#define PWM_FREQUENCY 50
/**
 * The number of individual steps the PWM can be configured in.
 * (If the PWM is configured to this value, it would be always on.)
 */
#define PWM_STEPS 1000
/**
 * Default servo value in PWM active steps.
 */
#define PWM_START_VALUE 72
/**
 * Maximum allowed servo value in PWM active steps.
 */
#define PWM_UPPER_LIMIT 100
/**
 * Minimum allowed servo value in PWM active steps.
 */
#define PWM_UNDER_LIMIT 55

/**
 * Buffers that save the last value from the servo controller topics.
 */
static CommBuffer<double> elevatorServoBuffer, aileronsServoBuffer, rudderServoBuffer;
/**
 * Subscriber to the elevator topic, which writes the latest value in the elevator buffer.
 */
static Subscriber elevatorServoSubscriber(elevatorControlTopic, elevatorServoBuffer,
        "Elevator Servo Status");
/**
 * Subscriber to the ailerons topic, which writes the latest value in the ailerons buffer.
 */
static Subscriber aileronsServoSubscriber(aileronsControlTopic, aileronsServoBuffer,
        "Ailerons Servo Status");
/**
 * Subscriber to the rudder topic, which writes the latest value in the rudder buffer.
 */
static Subscriber rudderServoSubscriber(rudderControlTopic, rudderServoBuffer,
        "Rudder Servo Status");

/*
 * Represents an servo motor actuator that can be controlled with a PWM signal.
 */
class Servo : public StaticThread<> {
public:
    
    /**
     * Create a new servo controller thread.
     *
     * @param name The name of the thread.
     * @param period The period in nanoseconds in which the thread checks for updated PWM signals.
     * @param pwm The hardware PWM id to use.
     * @param pwmStartValue The initial pwm value of the PWM in active steps.
     * @param commandBuffer The command buffer to read the latest PWM target in degrees.
     */
    Servo(const char* name, int64_t period, PWM_IDX pwm, int pwmStartValue,
          CommBuffer<double>* commandBuffer) : StaticThread(name), pwm(pwm) {
        this->period = period;
        this->pwm = pwm;
        this->pwmStartValue = pwmStartValue;
        this->commandBuffer = commandBuffer;
    }
    
    /**
     * Initialize the servo.
     */
    void init() override {
        pwm.init(PWM_FREQUENCY, PWM_STEPS);
        pwm.write(pwmStartValue);
    }
    
    /**
     * Run the servo thread loop: Pull the PWM target from the buffer and set the PWM to it
     */
    [[noreturn]] void run() override {
        int dutyCycle;
        double targetAngle = 0;
        int64_t nextTime = NOW();
        while (true) {
            nextTime += period;
            commandBuffer->getOnlyIfNewData(targetAngle);
            dutyCycle = (int) std::round(targetAngle) + pwmStartValue;
            
            // Make sure the requested value is within the secure limits
            if (dutyCycle > PWM_UPPER_LIMIT) {
                dutyCycle = PWM_UPPER_LIMIT;
            } else if (dutyCycle < PWM_UNDER_LIMIT) {
                dutyCycle = PWM_UNDER_LIMIT;
            }
            
            // Set the duty cycle of the servo, but make sure to only change one
            // at a time to prevent a spike in electricity usage by the servos.
            onlyOneServo.enter();
            pwm.write(dutyCycle);
            onlyOneServo.leave();
            
            suspendCallerUntil(nextTime);
        }
    }

private:
    /**
     * The period in nanoseconds for the PWM controller thread loop.
     */
    int64_t period;
    /**
     * Pulse width modulation timer.
     */
    HAL_PWM pwm;
    /**
     * Initial value of the pulse width modulation signal.
     */
    int pwmStartValue;
    /**
     * A buffer from which to read target angle.
     */
    CommBuffer<double>* commandBuffer;
    /**
     * A semaphore that is shared between all servos.
     * It is used to prevent changing to many servos at once.
     */
    static Semaphore onlyOneServo;
};

Semaphore Servo::onlyOneServo;

/**
 * The servo that controls the elevator control area.
 */
static Servo elevatorServo("Servo Elevator", 50 * MILLISECONDS,
        PWM_IDX00, PWM_START_VALUE, &elevatorServoBuffer);
/**
 * The servo that controls both ailerons control areas.
 */
static Servo aileronsServo("Servo Ailerons", 50 * MILLISECONDS,
        PWM_IDX01, PWM_START_VALUE, &aileronsServoBuffer);
/**
 * The servo that controls the rudder control area.
 */
static Servo rudderServo("Servo Rudder", 50 * MILLISECONDS,
        PWM_IDX03, PWM_START_VALUE, &rudderServoBuffer);
