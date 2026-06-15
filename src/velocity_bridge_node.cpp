#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <cmath>

class VelocityBridgeNode : public rclcpp::Node {
public:
    VelocityBridgeNode() : Node("velocity_bridge_node") {
        // Nav2 standard cmd_vel subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelocityBridgeNode::cmd_vel_callback, this, std::placeholders::_1));

        // PX4 sensor_data QoS profile for flight setpoints
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos);

        RCLCPP_INFO(this->get_logger(), "\033[92m🎛️ Velocity Bridge Node Active! Translating /cmd_vel to PX4...\033[0m");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        px4_msgs::msg::TrajectorySetpoint setpoint_msg;
        setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // 🎯 Setting position to NAN tell PX4 to ignore position and use pure velocity control
        setpoint_msg.position = {NAN, NAN, NAN};
        
        // Mapping Nav2 Twist velocities to PX4 Trajectory velocities
        setpoint_msg.velocity = {
            static_cast<float>(msg->linear.x),
            static_cast<float>(msg->linear.y),
            static_cast<float>(msg->linear.z)
        };
        
        setpoint_msg.acceleration = {0.0f, 0.0f, 0.0f};
        setpoint_msg.jerk = {0.0f, 0.0f, 0.0f};
        setpoint_msg.yaw = NAN;
        setpoint_msg.yawspeed = static_cast<float>(msg->angular.z);

        // Publish to Pixhawk/MAVROS hardware stream
        trajectory_setpoint_pub_->publish(setpoint_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
