#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <vector>
#include <thread> 
#include <mutex>  
#include <algorithm>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>

using namespace std::chrono_literals;

class SmartNavFailsafeNode : public rclcpp::Node {
public:
    SmartNavFailsafeNode() : Node("smart_nav_failsafe_node") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // 🎯 ROS 2 Parameters Declaration (No more hardcoding!)
        this->declare_parameter<float>("power_consumption_constant", 0.009f);
        this->declare_parameter<float>("safety_margin", 0.15f);
        this->declare_parameter<float>("target_x", 10.0f);
        this->declare_parameter<float>("target_y", 10.0f);
        this->declare_parameter<float>("target_z", -4.0f);

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);
        
        position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos, std::bind(&SmartNavFailsafeNode::position_callback, this, std::placeholders::_1));
        battery_subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
            "/fmu/out/battery_status_v1", qos, std::bind(&SmartNavFailsafeNode::battery_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&SmartNavFailsafeNode::master_supervisor_loop, this));

        RCLCPP_INFO(this->get_logger(), "\033[92m🛡️ Dynamic Parameter Node Initialized!\033[0m");
    }

    ~SmartNavFailsafeNode() {
        if (pathfinding_thread_.joinable()) {
            pathfinding_thread_.join();
        }
    }

private:
    float distance_3d(std::vector<float> p1, std::vector<float> p2) {
        return std::sqrt(std::pow(p1[0]-p2[0], 2) + std::pow(p1[1]-p2[1], 2) + std::pow(p1[2]-p2[2], 2));
    }

    float search(std::vector<float> current, float g_cost, float threshold, std::vector<float> goal, std::vector<std::vector<float>>& path, int depth) {
        if (depth > 50) return INFINITY; 

        float h_cost = distance_3d(current, goal); 
        float f_cost = g_cost + h_cost;            

        if (f_cost > threshold) return f_cost; 
        if (distance_3d(current, goal) < 1.0f) {
            path.push_back(current);
            return -1.0f; 
        }

        float min_over_threshold = INFINITY;
        std::vector<std::vector<float>> neighbors;

        for (float dx : {-1.0f, 0.0f, 1.0f}) {
            for (float dy : {-1.0f, 0.0f, 1.0f}) {
                if (dx == 0.0f && dy == 0.0f) continue; 
                
                float next_x = current[0] + dx * 2.0f; 
                float next_y = current[1] + dy * 2.0f;
                float next_z = current[2]; 
                neighbors.push_back({next_x, next_y, next_z});
            }
        }

        std::sort(neighbors.begin(), neighbors.end(), [this, goal](std::vector<float> a, std::vector<float> b) {
            return distance_3d(a, goal) < distance_3d(b, goal);
        });

        for (const auto& neighbor : neighbors) {
            float step_cost = distance_3d(current, neighbor);
            float result = search(neighbor, g_cost + step_cost, threshold, goal, path, depth + 1);
            
            if (result == -1.0f) { 
                path.push_back(current);
                return -1.0f;
            }
            if (result < min_over_threshold) {
                min_over_threshold = result;
            }
        }
        return min_over_threshold;
    }

    void async_ida_star_worker(std::vector<float> start, std::vector<float> goal) {
        RCLCPP_INFO(this->get_logger(), "\033[94m🧵 [Thread 2] IDA* Mathematical Loop Initiated...\033[0m");
        
        float threshold = distance_3d(start, goal); 
        std::vector<std::vector<float>> finalized_path;

        while (true) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (failsafe_triggered_) {
                    RCLCPP_WARN(this->get_logger(), "\033[93m🛑 [Thread 2] Failsafe Active! Killing pathfinding thread immediately.\033[0m");
                    return; 
                }
            }

            RCLCPP_INFO(this->get_logger(), "\033[95m📐 [Thread 2] Testing Path Cost Threshold Limit: %.2f\033[0m", threshold);
            std::vector<std::vector<float>> current_iteration_path;
            
            float result = search(start, 0.0f, threshold, goal, current_iteration_path, 0);
            
            if (result == -1.0f) { 
                std::reverse(current_iteration_path.begin(), current_iteration_path.end());
                finalized_path = current_iteration_path;
                break;
            }
            if (result == INFINITY || threshold > 100.0f) {
                RCLCPP_ERROR(this->get_logger(), "❌ Search bounded or exhausted.");
                break;
            }
            threshold = result + 0.5f; 
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!failsafe_triggered_) {
            active_path_ = finalized_path;
            path_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "\033[94m✅ [Thread 2] IDA* Mathematical Solver complete! Optimal Path mapped.\033[0m");
        }
    }

    void position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_x_ = msg->x; current_y_ = msg->y; current_z_ = msg->z; current_altitude_ = -msg->z;
    }

    void battery_callback(const px4_msgs::msg::BatteryStatus::UniquePtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg->remaining > 0.0) real_battery_telemetry_ = msg->remaining;
    }

    void master_supervisor_loop() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (failsafe_triggered_) return;

        // 🎯 Fetch Live Parameters dynamically on every loop tick
        float power_consumption_constant = this->get_parameter("power_consumption_constant").as_double();
        float safety_margin = this->get_parameter("safety_margin").as_double();
        float target_x = this->get_parameter("target_x").as_double();
        float target_y = this->get_parameter("target_y").as_double();
        float target_z = this->get_parameter("target_z").as_double();

        if (counter_ > 50 && simulated_battery_ > 0.0) {
            simulated_battery_ -= 0.003; 
        }
        float effective_battery = simulated_battery_; 

        float distance_to_home = std::sqrt(current_x_*current_x_ + current_y_*current_y_ + current_z_*current_z_);
        float battery_required_to_return = (distance_to_home * power_consumption_constant) + safety_margin;

        if (counter_ % 10 == 0 && current_altitude_ > 0.5) {
            RCLCPP_INFO(this->get_logger(), "📈 [Thread 1] Alt: %.2fm | Dist: %.1fm | 🔋 Battery: %.1f%% | ⚠️ Need: %.1f%%",
                        current_altitude_, distance_to_home, effective_battery * 100.0, battery_required_to_return * 100.0);
        }

        if (effective_battery <= battery_required_to_return && counter_ > 60) {
            failsafe_triggered_ = true;
            RCLCPP_ERROR(this->get_logger(), "\033[91m🚨🚨 BINGO FUEL EMERGENCY! Capacity: %.1f%% <= Required: %.1f%% 🚨🚨\033[0m", 
                         effective_battery*100.0, battery_required_to_return*100.0);
            trigger_rtl();
            return;
        }

        if (counter_ < 400) publish_offboard_control_mode();
        if (counter_ >= 10 && counter_ < 100 && counter_ % 10 == 0) {
            engage_offboard_mode();
            arm();
        }

        if (counter_ == 60) {
            std::vector<float> start = {current_x_, current_y_, current_z_};
            std::vector<float> goal = {target_x, target_y, target_z}; 
            pathfinding_thread_ = std::thread(&SmartNavFailsafeNode::async_ida_star_worker, this, start, goal);
            pathfinding_thread_.detach(); 
        }

        if (counter_ >= 0 && counter_ < 60) {
            publish_trajectory_setpoint(0.0, 0.0, target_z); 
        } else if (counter_ >= 60 && path_ready_) {
            publish_trajectory_setpoint(target_x, target_y, target_z);
        }

        counter_++;
    }

    void arm() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }
    void engage_offboard_mode() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); }
    void trigger_rtl() { publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH); }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg; msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z) {
        px4_msgs::msg::TrajectorySetpoint msg; msg.position = {x, y, z};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = command; msg.param1 = param1; msg.param2 = param2;
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1; msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    // Node Resources
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_subscription_;

    std::thread pathfinding_thread_;
    std::mutex data_mutex_;

    uint64_t counter_ = 0;
    float current_x_ = 0.0; float current_y_ = 0.0; float current_z_ = 0.0; float current_altitude_ = 0.0;
    float simulated_battery_ = 0.5; float real_battery_telemetry_ = 1.0;
    bool failsafe_triggered_ = false; bool path_ready_ = false;
    std::vector<std::vector<float>> active_path_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmartNavFailsafeNode>());
    rclcpp::shutdown();
    return 0;
}
