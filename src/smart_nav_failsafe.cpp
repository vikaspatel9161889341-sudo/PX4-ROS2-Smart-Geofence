#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <vector>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>

using namespace std::chrono_literals;

class SmartNavFailsafeNode : public rclcpp::Node {
public:
    SmartNavFailsafeNode() : Node("smart_nav_failsafe_node") {
        
        // Quality of Service Profile for PX4
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Publishers & Subscribers
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);
        
        position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos, std::bind(&SmartNavFailsafeNode::position_callback, this, std::placeholders::_1));

        // 10Hz Master Control Loop
        timer_ = this->create_wall_timer(100ms, std::bind(&SmartNavFailsafeNode::master_supervisor_loop, this));

        RCLCPP_INFO(this->get_logger(), "\033[92m🛡️ C++ Smart Navigation & Bingo Fuel Node Initialized!\033[0m");
    }

private:
    // Mock Octomap search function (Task 2 Placeholder)
    bool is_voxel_occupied(float x, float y, float z) {
        (void)x; (void)y; (void)z; 
        return false; 
    }

    // Task 3: Mock IDA* Pathfinding Algorithm
    std::vector<std::vector<float>> plan_ida_star_path(std::vector<float> start, std::vector<float> goal) {
        RCLCPP_INFO(this->get_logger(), "\033[94m📐 Running IDA* Pathfinding Thread...\033[0m");
        
        std::vector<std::vector<float>> planned_path;
        if (!is_voxel_occupied(goal[0], goal[1], goal[2])) {
            planned_path.push_back(start);
            planned_path.push_back(goal); 
            RCLCPP_INFO(this->get_logger(), "\033[94m✅ IDA* Optimal Path Found successfully!\033[0m");
        }
        return planned_path;
    }

    void position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
        current_altitude_ = -msg->z;
    }

    // Task 4: 10Hz Bingo Fuel Supervisor Thread Loop
    void master_supervisor_loop() {
        if (failsafe_triggered_) return;

        // Simulate battery decay for testing in SITL
        if (counter_ > 50 && simulated_battery_ > 0.0) {
            simulated_battery_ -= 0.003; 
        }

        // Calculate 3D Euclidean Distance to Home (0,0,0)
        float distance_to_home = std::sqrt(current_x_*current_x_ + current_y_*current_y_ + current_z_*current_z_);
        
        // Exact Protocol Equation: C <= (D * Pm) + S
        float power_consumption_constant = 0.009; 
        float safety_margin = 0.15;               
        float battery_required_to_return = (distance_to_home * power_consumption_constant) + safety_margin;

        // Print Telemetry logs every 1 second
        if (counter_ % 10 == 0 && current_altitude_ > 0.5) {
            RCLCPP_INFO(this->get_logger(), "📈 Alt: %.2fm | Dist to Home: %.1fm | \033[96m🔋 Battery: %.1f%%\033[0m | \033[95m⚠️ Need: %.1f%%\033[0m",
                        current_altitude_, distance_to_home, simulated_battery_ * 100.0, battery_required_to_return * 100.0);
        }

        // BINGO FUEL CHECK
        if (simulated_battery_ <= battery_required_to_return && counter_ > 60) {
            failsafe_triggered_ = true;
            RCLCPP_ERROR(this->get_logger(), "\033[91m🚨🚨 BINGO FUEL EMERGENCY DETECTED! Capacity: %.1f%% <= Required: %.1f%% 🚨🚨\033[0m", 
                         simulated_battery_*100.0, battery_required_to_return*100.0);
            RCLCPP_ERROR(this->get_logger(), "\033[91m🔄 Execution: Sending Forced RTL Command to PX4...\033[0m");
            trigger_rtl();
            return;
        }

        // Simple Offboard State Machine
        if (counter_ < 400) {
            publish_offboard_control_mode();
        }

        if (counter_ >= 10 && counter_ < 100 && counter_ % 10 == 0) {
            engage_offboard_mode();
            arm();
        }

        // Trigger IDA* planning once drone stabilizes at height
        if (counter_ == 60) {
            std::vector<float> start = {current_x_, current_y_, current_z_};
            std::vector<float> goal = {10.0, 10.0, -4.0}; 
            active_path_ = plan_ida_star_path(start, goal);
        }

        // Fly the mission path
        if (counter_ >= 0 && counter_ < 60) {
            publish_trajectory_setpoint(0.0, 0.0, -4.0); 
        } else if (counter_ >= 60 && !active_path_.empty()) {
            publish_trajectory_setpoint(10.0, 10.0, -4.0);
        }

        counter_++;
    }

    void arm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }

    void engage_offboard_mode() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    }

    void trigger_rtl() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z) {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.position = {x, y, z};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    // FIXED: Added default parameters to handle single-argument calls smoothly
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    // Node Resources
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription_;

    // Execution Variables
    uint64_t counter_ = 0;
    float current_x_ = 0.0;
    float current_y_ = 0.0;
    float current_z_ = 0.0;
    float current_altitude_ = 0.0;
    float simulated_battery_ = 1.0;
    bool failsafe_triggered_ = false;
    std::vector<std::vector<float>> active_path_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartNavFailsafeNode>());
    rclcpp::shutdown();
    return 0;
}
