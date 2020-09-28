#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/**
 * Max servo angle
 */
#define SERVO_MAX_ANGLE 28
/**
 * Min servo angle
 */
#define SERVO_MIN_ANGLE -17
/**
 * Scale to calculate the servo target from the target servo angle.
 */
#define SERVO_ANGLE_SCALE (-0.53 / SERVO_MAX_ANGLE)
/**
 * Max amount of velocity percentage a rotor can change in a tick.
 */
#define MOTOR_MAX_SPEED_CHANGE 0.1

namespace gazebo {
    /**
     * A plugin to control the Cessna via gazebo topics.
     */
    class CessnaPlugin : public ModelPlugin {
    public:
        CessnaPlugin() {
        }
        
        /***
         * The load function is called by Gazebo when the plugin is inserted into simulation.
         *
         * @param model A pointer to the model that this plugin is attached to.
         * @param sdf A pointer to the plugin's SDF element.
         */
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
            // Store the model pointer for convenience.
            this->model = model;
            
            std::array<std::string, 6> JOINT_NAMES = {
                    "elevators_joint",
                    "rudder_joint",
                    "left_flap_joint",
                    "left_aileron_joint",
                    "right_flap_joint",
                    "right_aileron_joint",
            };
            for (const auto &name : JOINT_NAMES) {
                physics::JointPtr joint = model->GetJoint("cessna_c172::" + name);
                if (joint == nullptr) {
                    gzerr << "Missing " << name << " joint\n";
                    return;
                }
                joints[name] = joint;
                this->model->GetJointController()->SetPositionPID(
                        joint->GetScopedName(), common::PID(500, 10, 100, 0, 0, 20000, -20000));
                this->model->GetJointController()->SetPositionTarget(
                        joint->GetScopedName(), 0);
            }
            
            // Create the node
            this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
            this->node->Init(this->model->GetWorld()->GetName());
#else
            this->node->Init(this->model->GetWorld()->Name());
#endif
            
            // Subscribe to the topic and register a callback
            this->elevatorSubscriber = this->node->Subscribe(
                    "Elevator Servo", &CessnaPlugin::OnMsgElevator, this);
            this->aileronsSubscriber = this->node->Subscribe(
                    "Ailerons Servo", &CessnaPlugin::OnMsgAilerons, this);
            this->rudderSubscriber = this->node->Subscribe(
                    "Rudder Servo", &CessnaPlugin::OnMsgRudder, this);
            this->imuSubscriber = this->node->Subscribe(
                    "~/cessna_c172/cessna_c172/body/IMU_Fuselage/imu", &CessnaPlugin::OnImuMsg,
                    this);
            this->worldControlSubscriber = this->node->Subscribe(
                    "~/world_control", &CessnaPlugin::OnWorldControlMsg, this);
            this->windTunnelSubscriber = this->node->Subscribe(
                    "Wind Tunnel Motor Speed", &CessnaPlugin::OnWindTunnelMsg, this);
            
            this->gyroscopeSensorTopic = this->node
                                             ->Advertise<gazebo::msgs::Vector3d>("Gyroscope", 50,
                                                     20);
            this->accelerometerSensorTopic = this->node->Advertise<gazebo::msgs::Vector3d>(
                    "Accelerometer", 50, 20);
            this->sensorsReadyTopic = this->node
                                          ->Advertise<gazebo::msgs::Empty>("Sensors Ready Flag", 50,
                                                  20);
            this->windTopic = this->node->Advertise<gazebo::msgs::Wind>("~/wind", 50, 20);
            
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&CessnaPlugin::OnUpdate, this));
        }
    
    private:
        /**
         * Set the target value of the elevator PID controller.
         * 
         * @param msg A vector containing the target value as the x component.
         */
        void OnMsgElevator(ConstVector2dPtr &msg) {
            this->model->GetJointController()->SetPositionTarget(
                    joints["elevators_joint"]->GetScopedName(), getTargetServoAngle(msg));
        }
        
        /**
         * Set the target value of the ailerons PID controller.
         *
         * @param msg A vector containing the target value as the x component.
         */
        void OnMsgAilerons(ConstVector2dPtr &msg) {
            double angle = getTargetServoAngle(msg);
            this->model->GetJointController()
                ->SetPositionTarget(joints["left_aileron_joint"]->GetScopedName(), angle);
            this->model->GetJointController()
                ->SetPositionTarget(joints["right_aileron_joint"]->GetScopedName(), -angle);
        }
        
        /**
         * Set the target value of the rudder PID controller.
         * 
         * @param msg A vector containing the target value as the x component.
         */
        void OnMsgRudder(ConstVector2dPtr &msg) {
            this->model->GetJointController()->SetPositionTarget(
                    joints["rudder_joint"]->GetScopedName(), getTargetServoAngle(msg));
        }
        
        /**
         * Forward the sensor data from the IMU to the RODOS topics.
         *
         * @param msg The IMU senor data.
         */
        void OnImuMsg(ConstIMUPtr &msg) {
            gazebo::msgs::Vector3d gyroSensorData = msg->angular_velocity();
            gyroSensorData.set_x(gyroSensorData.x() * (180.0 / M_PI) * 2);
            gyroSensorData.set_y(gyroSensorData.y() * (180.0 / M_PI) * 2);
            gyroSensorData.set_z(gyroSensorData.z() * (180.0 / M_PI) * 2);
            gyroscopeSensorTopic->Publish(gyroSensorData);
            
            gazebo::msgs::Vector3d accelerometerSensorData = msg->linear_acceleration();
            accelerometerSensorTopic->Publish(accelerometerSensorData);
        }
        
        /**
         * React to changes in the simulated world.
         *
         * @param msg The world control data.
         */
        void OnWorldControlMsg(ConstWorldControlPtr &msg) {
            gazebo::msgs::Empty sensorReadyData;
            sensorReadyData.set_unused(!msg->pause());
            sensorsReadyTopic->Publish(sensorReadyData);
        }
        
        /**
         * React to change commands to the wind tunnel motor speeds.
         *
         * @param msg The wind tunnel motor speed data.
         */
        void OnWindTunnelMsg(ConstVector2dPtr &msg) {
            leftWindTunnelMotorSpeedTarget = msg->x();
            rightWindTunnelMotorSpeedTarget = msg->y();
        }
        
        /**
         * Called every time the wold updates.
         */
        void OnUpdate() {
            leftWindTunnelMotorSpeed += std::max(
                    std::min(leftWindTunnelMotorSpeedTarget - leftWindTunnelMotorSpeed,
                            MOTOR_MAX_SPEED_CHANGE), -MOTOR_MAX_SPEED_CHANGE);
            rightWindTunnelMotorSpeed += std::max(
                    std::min(rightWindTunnelMotorSpeedTarget - rightWindTunnelMotorSpeed,
                            MOTOR_MAX_SPEED_CHANGE), -MOTOR_MAX_SPEED_CHANGE);
            double motorSpeedDiff = leftWindTunnelMotorSpeed - rightWindTunnelMotorSpeed;
            double windSpeed = std::max(leftWindTunnelMotorSpeed, rightWindTunnelMotorSpeed);
            gazebo::msgs::Wind windConfig;
            windConfig.set_enable_wind(true);
            gazebo::msgs::Vector3d* linearVelocity = windConfig.mutable_linear_velocity();
            linearVelocity->set_x(-windSpeed * 0.9);
            linearVelocity->set_y(motorSpeedDiff * 0.1);
            linearVelocity->set_z(0);
            this->windTopic->Publish(windConfig);
        }
        
        /**
         * Get the servo target value from the requested servo angle.
         * @param msg The message containing the requested servo angle.
         * @return The servo target value.
         */
        static double getTargetServoAngle(ConstVector2dPtr &msg) {
            return std::fmax(std::fmin(msg->x(), SERVO_MAX_ANGLE), SERVO_MIN_ANGLE) *
                   SERVO_ANGLE_SCALE;
        }
        
        /**
         * Pointer to the model.
         */
        physics::ModelPtr model;
        
        /**
         * A node used for transport
         */
        transport::NodePtr node;
        
        /**
         * A subscriber to the Gazebo elevator topic.
         */
        transport::SubscriberPtr elevatorSubscriber;
        /**
         * A subscriber to the Gazebo ailerons topic.
         */
        transport::SubscriberPtr aileronsSubscriber;
        /**
         * A subscriber to the Gazebo rudder topic.
         */
        transport::SubscriberPtr rudderSubscriber;
        /**
         * A subscriber to the IMU sensor topic.
         */
        transport::SubscriberPtr imuSubscriber;
        /**
         * A subscriber to the wind tunnel motor speed topic.
         */
        transport::SubscriberPtr windTunnelSubscriber;
        /**
         * A subscriber to the world_control topic.
         */
        transport::SubscriberPtr worldControlSubscriber;
        /**
         * Gyroscope sensor topic.
         */
        transport::PublisherPtr gyroscopeSensorTopic;
        /**
         * Accelerometer sensor topic.
         */
        transport::PublisherPtr accelerometerSensorTopic;
        /**
         * Sensors ready topic.
         */
        transport::PublisherPtr sensorsReadyTopic;
        
        /**
         * The joints of the cessna by name.
         */
        std::map <std::string, physics::JointPtr> joints;
        
        /**
         * Pointer to the update event connection.
         */
        event::ConnectionPtr updateConnection;
        transport::PublisherPtr windTopic;
        double leftWindTunnelMotorSpeed = 0;
        double rightWindTunnelMotorSpeed = 0;
        double leftWindTunnelMotorSpeedTarget = 0;
        double rightWindTunnelMotorSpeedTarget = 0;
    };
    
    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(CessnaPlugin)
}
