#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // Added for tf2::Quaternion
#include "ATRVmini_driver.h"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <angles/angles.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <cmath>


using namespace std;

/**
 *  ATRVmini node for ROS 2 - Modified for ROS 2 Humble
 *  Originally: ATRVmini node for ROS - Jaka Cikac 2013
 *  Modified from: B21_Node By David Lu!! 2/2010
 */
class ATRVminiNode : public rclcpp::Node {
    private:
        ATRVmini driver;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_accel_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_sonar_power_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_brake_power_sub;
        
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr base_sonar_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr body_sonar_pub;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_power_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sonar_power_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr bump_pub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
        
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        
        rclcpp::TimerBase::SharedPtr timer_;

        bool isSonarOn, isBrakeOn, last_brake;
        float acceleration;
        float last_distance, last_bearing, last_voltage, last_tvel, last_rvel;
        float x_odo, y_odo, a_odo;
        float cmdTranslation, cmdRotation;
        bool brake_dirty, sonar_dirty;
        bool initialized;
        float first_bearing;
        int updateTimer;
        int prev_bumps;
        bool sonar_just_on;

        void publishBrake();
        void publishBattery();
        void publishOdometry();
        void publishSonar();
        void publishBumps();

    public:
        ATRVminiNode();
        ~ATRVminiNode();
        int initialize(const char* port);
        void spinOnce();

        // Message Listeners
        void NewCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
        void SetAcceleration(const std_msgs::msg::Float32::SharedPtr msg);
        void ToggleSonarPower(const std_msgs::msg::Bool::SharedPtr msg);
        void ToggleBrakePower(const std_msgs::msg::Bool::SharedPtr msg);
        void timerCallback();
};

ATRVminiNode::ATRVminiNode() : Node("atrv_node") {
    isSonarOn = isBrakeOn = false;
    brake_dirty = sonar_dirty = false;
    sonar_just_on = false;
    last_brake = false;
    cmdTranslation = cmdRotation = 0.0;
    updateTimer = 99;
    initialized = false;
    prev_bumps = 0;
    
    // QoS settings
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    
    // Subscribers
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&ATRVminiNode::NewCommand, this, std::placeholders::_1));
    
    cmd_accel_sub = this->create_subscription<std_msgs::msg::Float32>(
        "cmd_accel", 1, std::bind(&ATRVminiNode::SetAcceleration, this, std::placeholders::_1));
    
    cmd_sonar_power_sub = this->create_subscription<std_msgs::msg::Bool>(
        "cmd_sonar_power", 1, std::bind(&ATRVminiNode::ToggleSonarPower, this, std::placeholders::_1));
    
    cmd_brake_power_sub = this->create_subscription<std_msgs::msg::Bool>(
        "cmd_brake_power", 1, std::bind(&ATRVminiNode::ToggleBrakePower, this, std::placeholders::_1));
    
    acceleration = 0.7;

    // Publishers
    base_sonar_pub = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "sonar_cloud_base", sensor_qos);
    
    body_sonar_pub = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "sonar_cloud_body", sensor_qos);
    
    sonar_power_pub = this->create_publisher<std_msgs::msg::Bool>(
        "sonar_power", latched_qos);
    
    brake_power_pub = this->create_publisher<std_msgs::msg::Bool>(
        "brake_power", latched_qos);
    
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "odom", sensor_qos);
    
    twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "twist", sensor_qos);
    
    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        "state", reliable_qos);
    
    bump_pub = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "bump", sensor_qos);
    
    battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "battery_state", latched_qos);
    
    // Initialize transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize timer
    int hz = 15;
    this->declare_parameter("rate", hz);
    this->get_parameter("rate", hz);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / hz),
        std::bind(&ATRVminiNode::timerCallback, this));
}

int ATRVminiNode::initialize(const char* port) {
    int ret = driver.initialize(port);
    if (ret < 0)
        return ret;
        
    //Values changed by matejD. Originals:
    //driver.setOdometryPeriod (100000);
    //driver.setDigitalIoPeriod(100000);

    driver.setOdometryPeriod(10000);
    driver.setDigitalIoPeriod(10000);
    driver.motionSetDefaults();
    return 0;
}

ATRVminiNode::~ATRVminiNode() {
    driver.motionSetDefaults();
    driver.setOdometryPeriod(0);
    driver.setDigitalIoPeriod(0);
    driver.setSonarPower(false);
    driver.setIrPower(false);
}

void ATRVminiNode::timerCallback() {
    spinOnce();
}

/// cmd_vel callback
void ATRVminiNode::NewCommand(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmdTranslation = msg->linear.x;
    cmdRotation = msg->angular.z;
    driver.setMovement(cmdTranslation, cmdRotation, acceleration);
}

/// cmd_acceleration callback
void ATRVminiNode::SetAcceleration(const std_msgs::msg::Float32::SharedPtr msg) {
    acceleration = msg->data;
}

/// cmd_sonar_power callback
void ATRVminiNode::ToggleSonarPower(const std_msgs::msg::Bool::SharedPtr msg) {
    isSonarOn = msg->data;
    sonar_dirty = true;
}

/// cmd_brake_power callback
void ATRVminiNode::ToggleBrakePower(const std_msgs::msg::Bool::SharedPtr msg) {
    isBrakeOn = msg->data;
    brake_dirty = true;
}

void ATRVminiNode::spinOnce() {
    // Sending the status command too often overwhelms the driver
    if (updateTimer >= 30) {
        driver.sendSystemStatusCommand();
        updateTimer = 0;
    }
    updateTimer++;

    if (sonar_dirty) {
        driver.setSonarPower(isSonarOn);
        sonar_dirty = false;
        driver.sendSystemStatusCommand();
    }
    if (brake_dirty) {
        driver.setBrakePower(isBrakeOn);
        brake_dirty = false;
    }

    auto bmsg = std_msgs::msg::Bool();
    bmsg.data = isSonarOn;
    sonar_power_pub->publish(bmsg);

    publishBrake();
    publishBattery();
    publishOdometry();
    publishSonar();
    publishBumps();
}

void ATRVminiNode::publishBrake() {
    bool brake = driver.getBrakePower();

    if(last_brake != brake){
        last_brake = brake;

        auto msg = std_msgs::msg::Bool();
        msg.data = brake;
        brake_power_pub->publish(msg);
    }
}

void ATRVminiNode::publishBattery() {
    float voltage = driver.getVoltage();

    if(voltage != last_voltage){
        last_voltage = voltage;

        auto msg = sensor_msgs::msg::BatteryState();
        msg.capacity = 18.0;
        msg.design_capacity = 18.0;
        msg.current = std::nanf("");
        msg.temperature = std::nanf("");
        msg.voltage = voltage;
        msg.percentage = driver.getPercentage();
        msg.charge = 18.0 * msg.percentage;
        msg.present = true;

        if(driver.isPluggedIn()){
            msg.power_supply_status = msg.POWER_SUPPLY_STATUS_CHARGING;
        }else{
            msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING;
        }

        msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_technology = msg.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

        msg.cell_voltage.resize(12, voltage/12.0); 

        msg.location = "front&back";
        msg.serial_number = "Unknown";
        
        battery_pub->publish(msg);
    }
}

void ATRVminiNode::publishOdometry() {
    if (!driver.isOdomReady()) {
        return;
    }
    
    float distance = driver.getDistance();
    float true_bearing = angles::normalize_angle(driver.getBearing());
    
    if (!initialized) {
        initialized = true;
        first_bearing = true_bearing;
        x_odo = 0;
        y_odo = 0;
        a_odo = 0 * true_bearing;
    } else {
        float bearing = true_bearing - first_bearing;
        float d_dist = distance - last_distance;
        float d_bearing = bearing - last_bearing;

        // Reject implausible odometry jumps (2.4 meter margin)
        if (d_dist > 1.2 || d_dist < -1.2) {
            return;
        }
        
        a_odo += d_bearing;
        a_odo = angles::normalize_angle(a_odo);

        // Integrate latest motion into odometry
        x_odo += d_dist * cos(a_odo);
        y_odo += d_dist * sin(a_odo);
    }
    
    last_distance = distance;
    last_bearing = true_bearing - first_bearing;

    // Create quaternion from yaw (bearing)
    tf2::Quaternion q;
    q.setRPY(0, 0, last_bearing);
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.x = q.x();
    odom_quat.y = q.y();
    odom_quat.z = q.z();
    odom_quat.w = q.w();

    // First, publish the transform over tf
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = this->now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_odo;
    odom_trans.transform.translation.y = y_odo;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // Send the transform
    tf_broadcaster->sendTransform(odom_trans);

    // Next, publish the odometry message over ROS
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";

    // Set the position
    odom.pose.pose.position.x = x_odo;
    odom.pose.pose.position.y = y_odo;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Set the velocity
    odom.child_frame_id = "base_link";
    float tvel = driver.getTranslationalVelocity();
    float d_tvel = tvel - last_tvel; 
    
    // Reject implausible velocity jumps
    if (d_tvel > 1.0 || d_tvel < -1.0) {
        return;
    }
    
    last_tvel = tvel;
    odom.twist.twist.linear.x = tvel * cos(a_odo);
    odom.twist.twist.linear.y = tvel * sin(a_odo);
    
    float rvel = driver.getRotationalVelocity();
    odom.twist.twist.angular.z = rvel;
    
    float d_rvel = rvel - last_rvel; 
    // Reject implausible rotational velocity jumps
    if (d_rvel > 1.0 || d_rvel < -1.0) {
        return;
    }

    last_rvel = rvel;
    
    // Publish the pure twist stamped message
    auto twist_stamped = geometry_msgs::msg::TwistStamped();
    twist_stamped.header.stamp = this->now();
    twist_stamped.twist.linear.x = tvel;
    twist_stamped.twist.angular.z = rvel;

    // Publish the messages
    odom_pub->publish(odom);
    twist_pub->publish(twist_stamped);

    // Finally, publish the joint state
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = this->now();
    joint_state.name.resize(1);
    joint_state.position.resize(1);
    joint_state.name[0] = "joint_twist";
    joint_state.position[0] = true_bearing;

    joint_pub->publish(joint_state);
}

void ATRVminiNode::publishSonar() {
    auto cloud = sensor_msgs::msg::PointCloud();
    cloud.header.stamp = this->now();
    cloud.header.frame_id = "base_link";

    if (isSonarOn) {
        driver.getBaseSonarPoints(&cloud);
        base_sonar_pub->publish(cloud);

        driver.getBodySonarPoints(&cloud);
        cloud.header.frame_id = "body";
        body_sonar_pub->publish(cloud);

    } else if (sonar_just_on) {
        base_sonar_pub->publish(cloud);
        cloud.header.frame_id = "body";
        body_sonar_pub->publish(cloud);
    }
}

void ATRVminiNode::publishBumps() {
    auto cloud1 = sensor_msgs::msg::PointCloud();
    auto cloud2 = sensor_msgs::msg::PointCloud();
    cloud1.header.stamp = this->now();
    cloud2.header.stamp = this->now();
    cloud1.header.frame_id = "base_link";
    cloud2.header.frame_id = "body";
    int bumps = driver.getBaseBumps(&cloud1) +
                driver.getBodyBumps(&cloud2);

    if (bumps>0 || prev_bumps>0) {
        bump_pub->publish(cloud1);
        bump_pub->publish(cloud2);
    }
    prev_bumps = bumps;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ATRVminiNode>();
    
    std::string port = "/dev/ttyUSB0";
    node->declare_parameter("port", port);
    node->get_parameter("port", port);
    
    RCLCPP_INFO(node->get_logger(), "Attempting to connect to %s", port.c_str());
    if (node->initialize(port.c_str()) < 0) {
        RCLCPP_ERROR(node->get_logger(), "Could not initialize RFLEX driver!");
        return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Connected!");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}