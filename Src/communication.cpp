#ifdef BUILD_FOR_GAZEBO
#  include <unistd.h>
#  include <arpa/inet.h>
#  include <fcntl.h>
#endif /* BUILD_FOR_GAZEBO */

#include <unistd.h>
#include "topics.h"
#include "rodos.h"

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

enum Message : char {
    PRINT = 'p',
    DATA = 'd',
    CALIBRATE = 'c',
    AUTOPILOT = 'a',
    TARGET = 't',
    SERVOS = 's',
#ifdef PROFILE_CONTROLLERS
    PROFILER = 'r',
#endif /* PROFILE_CONTROLLERS */
#ifdef BUILD_FOR_GAZEBO
    WIND_TUNNEL = 'w',
#endif /* BUILD_FOR_GAZEBO */
};

typedef struct {
    char messageType;  // Message::PRINT
    uint32_t size;
    char message[128];
} __attribute__((packed)) printMessage;

typedef struct {
    char messageType;  // Message::DATA
    uint32_t index;
    int64_t time;
    uint8_t selectedAutopilot;
    bool sensorsReady;
    double accelerometerX, accelerometerY, accelerometerZ;
    double gyroscopeX, gyroscopeY, gyroscopeZ;
    double elevatorServo, aileronsServo, rudderServo;
    double yaw, pitch, roll;
    double targetYaw, targetPitch, targetRoll;
    char end;  // Always 'E'
} __attribute__((packed)) dataMessage;

typedef struct {
    char messageType;  // Message::TARGET
    double yaw, pitch, roll;  // Target orientation angles
} __attribute__((packed)) targetMessage;

typedef struct {
    char messageType;  // Message::SERVOS
    double elevatorServo, aileronsServo, rudderServo;  // Servo angles
} __attribute__((packed)) servosMessage;

#ifdef PROFILE_CONTROLLERS
typedef struct {
    char messageType;  // Message::PROFILER
    int64_t executionTime;
} __attribute__((packed)) profileMessage;


static CommBuffer<int64_t> controllerProfilerStatus;
static Subscriber controllerProfilerSubscriber(
        controllerProfiler, controllerProfilerStatus, "Status Controller Profiler");
#endif /* PROFILE_CONTROLLERS */

#ifdef BUILD_FOR_GAZEBO
typedef struct {
    char messageType;  // Message::WIND_TUNNEL
    double leftMotorSpeed, rightMotorSpeed;
} __attribute__((packed)) windTunnelMessage;

extern GazeboTopic<gazebo::msgs::Vector2d> windTunnelMotorTopic;
#endif /* BUILD_FOR_GAZEBO */


static Application communication("Communication", 2000);

static CommBuffer<DoubleMessage> elevatorServoBuffer, aileronsServoBuffer, rudderServoBuffer;
static Subscriber elevatorServoSubscriber(
        elevatorControlTopic, elevatorServoBuffer, "Status Elevator Servo");
static Subscriber aileronsServoSubscriber(
        aileronsControlTopic, aileronsServoBuffer, "Status Ailerons Servo");
static Subscriber rudderServoSubscriber(
        rudderControlTopic, rudderServoBuffer, "Status Rudder Servo");

static CommBuffer<Quaternion> positionStatus;
static Subscriber positionSubscriber(quaternionTopic, positionStatus, "Status Position");

static CommBuffer<TargetOrientation> targetOrientationStatus;
static Subscriber targetOrientationSubscriber(targetOrientationTopic, targetOrientationStatus,
        "Status Target Orientation");

static CommBuffer<Vector3DMsg> accelerometerStatus;
static Subscriber accelerometerStatusSubscriber(
        accelerometerSensorTopic, accelerometerStatus, "Status Accelerometer");

static CommBuffer<Vector3DMsg> gyroscopeStatus;
static Subscriber gyroscopeStatusSubscriber(gyroSensorTopic, gyroscopeStatus, "Status Gyroscope");

static CommBuffer<BoolMessage> sensorsReadyBuffer;
static Subscriber senorsReadySubscriber(sensorsReadyTopic, sensorsReadyBuffer,
        "Status Sensors Ready");

static CommBuffer<Autopilot> autopilotSelectionBuffer;
static Subscriber autopilotSelectionSubscriber(
        autopilotTopic, autopilotSelectionBuffer, "Status Autopilot Selection");

#ifdef BUILD_FOR_GAZEBO
class SocketPort {
public:
    SocketPort() {
        controlSocket = socket(AF_INET, SOCK_STREAM, 0);
    }
    
    ~SocketPort() {
        close(controlSocket);
    }
    
    void init(int) {
        struct sockaddr_in serverAddress = {
                .sin_family = AF_INET
        };
        struct in_addr serverIPv4Address = {};
        inet_pton(AF_INET, "10.0.2.2", &serverIPv4Address);
        struct hostent* hp = gethostbyaddr(&serverIPv4Address, sizeof(serverIPv4Address), AF_INET);
        if (hp == nullptr) {
            PRINTF("Unable to find host\n");
            return;
        }
        RODOS::bcopy(hp->h_addr, &(serverAddress.sin_addr.s_addr), hp->h_length);
        serverAddress.sin_port = htons(9999);
        if (connect(controlSocket, (const sockaddr*) &serverAddress, sizeof(serverAddress)) == 0) {
            PRINTF("Successfully connected\n");
            haveConnection = true;
            fcntl(controlSocket, F_SETFL, fcntl(controlSocket, F_GETFL) | O_NONBLOCK);
        } else {
            PRINTF("Connection failed\n");
        }
    }
    
    void suspendUntilWriteFinished() {}

    void suspendUntilDataReady() {}
    
    size_t write(const void* data, size_t length) const {
        if (!haveConnection) {
            return 0;
        }
        ssize_t bytesWritten = send(controlSocket, data, length, 0);
        if (bytesWritten < 0) {
            return 0;
        }
        return bytesWritten;
    }
    
    size_t read(void *buffer, size_t bufferLength) const {
        if (!haveConnection) {
            return 0;
        }
        ssize_t bytesRead = recv(controlSocket, buffer, bufferLength, 0);
        if (bytesRead < 0) {
            return 0;
        }
        return bytesRead;
    }

private:
    bool haveConnection = false;
    int controlSocket;
};
#endif /* BUILD_FOR_GAZEBO */

/*
 * Der Thread Communication empfaengt und verabreitet Befehle,
 * bestaetigt deren Empfang und sendet den aktuellen Stats
 */
class Communication : public StaticThread<> {
public:
    explicit Communication(int64_t period);
    
    void init() override;
    
    [[noreturn]] void run() override;

private:
    void sendStatus();
    
    void handleCommands();
    
    void sendMessage(const char* message);
    
    void handleAutopilotCommand();
    
    void handleTargetCommand();
    
    void handleServosCommand();

#ifdef BUILD_FOR_GAZEBO
    void handleWindTunnelCommand();
#endif /* BUILD_FOR_GAZEBO */
    
    /**
     * Read bufferSize bytes from the com port. Block until all bytes have been received.
     * @param buffer The buffer to put the bytes into.
     * @param bufferSize The size of the buffer in bytes.
     */
    void readBytes(uint8_t* buffer, size_t bufferSize);

#ifndef BUILD_FOR_GAZEBO
    HAL_UART comPort;
#else
    SocketPort comPort;
#endif /* BUILD_FOR_GAZEBO */
    int64_t period;
};

Communication::Communication(int64_t periode) :
        StaticThread("Communication")
#ifndef BUILD_FOR_GAZEBO
        , comPort(UART_IDX3, GPIO_056, GPIO_057)
#endif /* BUILD_FOR_GAZEBO */
{
    this->period = periode;
    
}

void Communication::init() {
    comPort.init(115200);
}

[[noreturn]] void Communication::run() {
    int64_t nextLoopTime = NOW();
    while (true) {
        nextLoopTime += period;
        handleCommands();
        sendStatus();
        suspendCallerUntil(nextLoopTime);
    }
}

void Communication::sendStatus() {
    static dataMessage data = {
            .messageType = Message::DATA,
            .index = 0,
            .end = 'E'
    };
    
    Vector3DMsg accelerometerData;
    Vector3DMsg gyroscopeData;
    Quaternion position;
    TargetOrientation targetOrientation = {};
    Autopilot selectedAutoPilot;
    BoolMessage sensorReady;
    DoubleMessage elevatorServoStatus, aileronsServoStatus, rudderServoStatus;
    autopilotSelectionBuffer.get(selectedAutoPilot);
    sensorsReadyBuffer.get(sensorReady);
    accelerometerStatus.get(accelerometerData);
    gyroscopeStatus.get(gyroscopeData);
    elevatorServoBuffer.get(elevatorServoStatus);
    aileronsServoBuffer.get(aileronsServoStatus);
    rudderServoBuffer.get(rudderServoStatus);
    positionStatus.get(position);
    targetOrientationStatus.get(targetOrientation);
    YPR positionAngles = position.toYPR();
    
    data.time = NOW();
    data.selectedAutopilot = selectedAutoPilot;
    data.sensorsReady = BOOL_GET_VALUE(sensorReady);
    data.accelerometerX = VECTOR3D_GET(accelerometerData, x);
    data.accelerometerY = VECTOR3D_GET(accelerometerData, y);
    data.accelerometerZ = VECTOR3D_GET(accelerometerData, z);
    data.gyroscopeX = VECTOR3D_GET(gyroscopeData, x);
    data.gyroscopeY = VECTOR3D_GET(gyroscopeData, y);
    data.gyroscopeZ = VECTOR3D_GET(gyroscopeData, z);
    data.elevatorServo = DOUBLE_GET_VALUE(elevatorServoStatus);
    data.aileronsServo = DOUBLE_GET_VALUE(aileronsServoStatus);
    data.rudderServo = DOUBLE_GET_VALUE(rudderServoStatus);
    data.yaw = positionAngles.yaw;
    data.pitch = positionAngles.pitch;
    data.roll = positionAngles.roll;
    data.targetYaw = targetOrientation.yaw;
    data.targetPitch = targetOrientation.pitch;
    data.targetRoll = targetOrientation.roll;
    comPort.suspendUntilWriteFinished();
    comPort.write(&data, sizeof(dataMessage));
    data.index++;

#ifdef PROFILE_CONTROLLERS
    static profileMessage profilerData = {
            .messageType = Message::PROFILER
    };
    controllerProfilerStatus.get(profilerData.executionTime);
    comPort.suspendUntilWriteFinished();
    comPort.write(&profilerData, sizeof(profileMessage));
#endif /* PROFILE_CONTROLLERS */
}

void Communication::handleCommands() {
    char command;
    while (comPort.read(&command, 1) > 0) {
        switch (command) {
        case PRINT: // Ping
            sendMessage("pong");
            break;
        case CALIBRATE: {
            BoolMessage sensorReadyData;
            BOOL_MSG_INIT(sensorReadyData, false);
            sensorsReadyTopic.publish(sensorReadyData);
            break;
        }
        case AUTOPILOT:
            handleAutopilotCommand();
            break;
        case TARGET:
            handleTargetCommand();
            break;
        case SERVOS:
            handleServosCommand();
            break;
#ifdef BUILD_FOR_GAZEBO
            case WIND_TUNNEL:
                handleWindTunnelCommand();
                break;
#endif /* BUILD_FOR_GAZEBO */
        default:
            PRINTF("Unknown command: '%c'\n", command);
        }
    }
}

void Communication::sendMessage(const char* message) {
    static printMessage messageData = {
            .messageType = Message::PRINT
    };
    messageData.size = max(RODOS::strlen(message), ARRAY_SIZE(messageData.message));
    RODOS::memcpy(messageData.message, message, messageData.size);
    comPort.write(&messageData,
            sizeof(printMessage) - ARRAY_SIZE(messageData.message) + messageData.size);
}

void Communication::handleAutopilotCommand() {
    comPort.suspendUntilDataReady();
    uint8_t autopilotSelection = Autopilot::NONE;
    while (comPort.read(&autopilotSelection, 1) < 1) {
        Thread::yield();
    }
    if (autopilotSelection < Autopilot::NONE || autopilotSelection > Autopilot::MAX_VALUE) {
        PRINTF("Invalid autopilot selection: %d\n", (int) autopilotSelection);
    } else {
        autopilotTopic.publish(Autopilot(autopilotSelection));
    }
}

void Communication::handleTargetCommand() {
    targetMessage targetData;
    readBytes(reinterpret_cast<uint8_t*>(&targetData) + 1, sizeof(targetMessage) - sizeof(char));
    TargetOrientation target = {
            .roll = targetData.roll,
            .pitch = targetData.pitch,
            .yaw = targetData.yaw,
    };
    targetOrientationTopic.publish(target);
}

void Communication::handleServosCommand() {
    autopilotTopic.publish(Autopilot::NONE);
    servosMessage servosData;
    readBytes(reinterpret_cast<uint8_t*>(&servosData) + 1, sizeof(servosMessage) - sizeof(char));
    DoubleMessage elevatorServo, aileronsServo, rudderServo;
    DOUBLE_MSG_INIT(elevatorServo, servosData.elevatorServo);
    DOUBLE_MSG_INIT(aileronsServo, servosData.aileronsServo);
    DOUBLE_MSG_INIT(rudderServo, servosData.rudderServo);
    elevatorControlTopic.publish(elevatorServo);
    aileronsControlTopic.publish(aileronsServo);
    rudderControlTopic.publish(rudderServo);
}

#ifdef BUILD_FOR_GAZEBO
void Communication::handleWindTunnelCommand() {
    windTunnelMessage windTunnelData;
    readBytes(reinterpret_cast<uint8_t*>(&windTunnelData) + 1, sizeof(windTunnelMessage) - sizeof(char));
    gazebo::msgs::Vector2d motorSpeeds;
    motorSpeeds.set_x(windTunnelData.leftMotorSpeed);
    motorSpeeds.set_y(windTunnelData.rightMotorSpeed);
    windTunnelMotorTopic.publish(motorSpeeds);
}
#endif /* BUILD_FOR_GAZEBO */

void Communication::readBytes(uint8_t* buffer, size_t bufferSize) {
    size_t bytesLeft = bufferSize;
    do {
        comPort.suspendUntilDataReady();
        bytesLeft -= comPort.read(buffer + (bufferSize - bytesLeft), bytesLeft);
    } while (bytesLeft > 0);
}

Communication communicationThread(50 * MILLISECONDS);
