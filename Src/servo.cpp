#include "rodos.h"
#include "topics.h"
#include "stm32f4xx_conf.h"

/**
 * Default servo value
 */
#define PWM_START_VALUE 72
/**
 * Max servo value
 */
#define PWM_UPPER_LIMIT 100
/**
 * Min servo value
 */
#define PWM_UNDER_LIMIT 55


static CommBuffer<double> elevatorServoBuffer, aileronsServoBuffer, rudderServoBuffer;
static Subscriber elevatorServoSubscriber(elevatorControlTopic, elevatorServoBuffer,
        "Elevator Servo Status");
static Subscriber aileronsServoSubscriber(aileronsControlTopic, aileronsServoBuffer,
        "Ailerons Servo Status");
static Subscriber rudderServoSubscriber(rudderControlTopic, rudderServoBuffer,
        "Rudder Servo Status");

/*
 * Represents an actuator that can be controlled with a PWM signal.
 */
class Servo : public StaticThread<> {
public:
    
    Servo(const char* name, int64_t period, HAL_PWM* pwm, int pwmStartValue,
          CommBuffer<double>* commandBuffer) : StaticThread(name) {
        this->period = period;
        this->pwm = pwm;
        this->pwmStartValue = pwmStartValue;
        this->commandBuffer = commandBuffer;
    }
    
    /**
     * Initialize the servo.
     */
    void init() override {
        pwm->init(50, 1000);
        pwm->write(pwmStartValue);
    }
    
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
            
            // Set the duty cycle of the servo, but make sure to only change one at a time.
            onlyOneServo.enter();
            pwm->write(dutyCycle);
            onlyOneServo.leave();
            
            suspendCallerUntil(nextTime);
        }
    }

private:
    int64_t period;
    /**
     * Pulse width modulation timer.
     */
    HAL_PWM* pwm;
    /**
     * Initial value of the Pulse width modulation signal.
     */
    int pwmStartValue;
    /**
     * A buffer from which to read target angles.
     */
    CommBuffer<double>* commandBuffer;
    /**
     * A semaphore that is shared between all servos.
     * It is used to prevent changing to many servos at once.
     */
    static Semaphore onlyOneServo;
};

Semaphore Servo::onlyOneServo;

static HAL_PWM pwm0(PWM_IDX00);
static HAL_PWM pwm1(PWM_IDX01);
static HAL_PWM pwm3(PWM_IDX03);

static Servo elevatorServo("Servo Elevator", 1 * SECONDS, &pwm0, PWM_START_VALUE,
        &elevatorServoBuffer);
static Servo aileronsServo("Servo Ailerons", 1 * SECONDS, &pwm1, PWM_START_VALUE,
        &aileronsServoBuffer);
static Servo rudderServo("Servo Rudder", 1 * SECONDS, &pwm3, PWM_START_VALUE, &rudderServoBuffer);
