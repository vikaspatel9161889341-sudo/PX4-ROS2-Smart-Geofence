#include <string>
#include <cmath>
#include <memory>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class BatteryEmergencyCondition : public BT::ConditionNode {
public:
    BatteryEmergencyCondition(const std::string & name, const BT::NodeConfiguration & config)
    : BT::ConditionNode(name, config) {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", 10,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                last_battery_remaining_ = msg->percentage;
            });

        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_x_ = msg->pose.position.x;
                current_y_ = msg->pose.position.y;
                current_z_ = msg->pose.position.z;
            });
            
        RCLCPP_INFO(node_->get_logger(), "🛡️ Nav2 BT Battery Condition Node Registered at 10Hz!");
    }

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override {
        float distance_to_home = std::sqrt(current_x_*current_x_ + current_y_*current_y_ + current_z_*current_z_);
        float power_consumption_constant = 0.009f; 
        float safety_margin = 0.15f;               
        
        float energy_required_to_return = (distance_to_home * power_consumption_constant) + safety_margin;

        if (last_battery_remaining_ <= energy_required_to_return) {
            RCLCPP_ERROR(node_->get_logger(), "🚨 [BT INTERCEPT] Battery Low! Triggering Forced RTL Branch.");
            return BT::NodeStatus::SUCCESS; 
        }

        return BT::NodeStatus::FAILURE; 
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    float last_battery_remaining_ = 0.5f; 
    float current_x_ = 0.0f; float current_y_ = 0.0f; float current_z_ = 0.0f;
};

// 🎯 FIXED: Humble Standard clean registration macro
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<BatteryEmergencyCondition>("BatteryEmergency");
}
