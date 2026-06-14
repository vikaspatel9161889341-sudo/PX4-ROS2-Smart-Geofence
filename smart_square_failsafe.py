import rclpy
from rclpy.node import Node
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, BatteryStatus

class SmartOffboardNode(Node):
    def __init__(self):
        super().__init__('smart_offboard_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers & Subscribers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        self.position_subscription = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.position_callback, qos_profile)
        self.battery_subscription = self.create_subscription(BatteryStatus, '/fmu/out/battery_status_v1', self.battery_callback, qos_profile)

        # Variables for 3D Tracking
        self.counter = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_altitude = 0.0
        
        # 🔋 Smart Battery Variables
        self.simulated_battery = 1.0  # 100% initial battery
        self.failsafe_triggered = False
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("\033[92m🛡️ Dynamic Smart-RTL System Activated!\033[0m")

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_altitude = -msg.z

    def battery_callback(self, msg):
        # In real drone, we take msg.remaining. For simulation, we use our decay logic below.
        pass

    def timer_callback(self):
        if self.failsafe_triggered:
            return

        # 🔋 Simulate Battery consumption over time (0.2% drop every 0.1 second after takeoff)
        if self.counter > 150 and self.simulated_battery > 0.0:
            self.simulated_battery -= 0.002 

        # 📐 MATH: Calculate 3D Distance to Home Point (0,0,0)
        distance_to_home = math.sqrt(self.current_x**2 + self.current_y**2 + self.current_z**2)
        
        # 📊 ESTIMATION LOGIC:
        # Assume drone takes 0.8% battery per meter to fly back + 15% safe reserve for landing descent.
        battery_required_to_return = (distance_to_home * 0.008) + 0.15
        
        current_bat_pct = self.simulated_battery * 100
        req_bat_pct = battery_required_to_return * 100

        # Live Telemetry Monitoring
        if self.counter % 10 == 0 and self.current_altitude > 0.05:
            self.get_logger().info(
                f"📈 Height: {self.current_altitude:.2f}m | Distance: {distance_to_home:.1f}m | "
                f"\033[96m🔋 Battery: {current_bat_pct:.1f}%\033[0m | \033[95m⚠️ Need: {req_bat_pct:.1f}%\033[0m"
            )
        
        # 🚨 DYNAMIC SMART-RTL TRIGGER
        # If available battery drops to or below the exact amount needed to return, TURN BACK IMMEDATELY!
        if self.simulated_battery <= battery_required_to_return and self.counter > 150:
            self.failsafe_triggered = True
            self.get_logger().error(f"\033[91m🚨🚨 SMART RTL TRIGGERED! Current Battery ({current_bat_pct:.1f}%) matches Return Cost ({req_bat_pct:.1f}%)! 🚨🚨\033[0m")
            self.get_logger().error("\033[91m🔄 No energy left for mission. Turning back to launch pad right now...\033[0m")
            self.rtl()
            return

        # Geofence backup
        if self.current_altitude > 6.0:
            self.failsafe_triggered = True
            self.get_logger().error("\033[91m🚨🚨 GEOFENCE BREACH DETECTED! 🚨🚨\033[0m")
            self.rtl()
            return

        if self.counter < 800:
            self.publish_offboard_control_mode()

        # Arm & Offboard Retry
        if self.counter >= 10 and self.counter < 120 and self.counter % 10 == 0:
            self.engage_offboard_mode()
            self.arm()

        # ⭕ Circle Mission Path (Radius 4m)
        if self.counter >= 0 and self.counter < 150:
            self.publish_trajectory_setpoint(0.0, 0.0, -4.0)
        elif self.counter >= 150 and self.counter < 750:
            theta = ((self.counter - 150) / 400.0) * 2.0 * math.pi
            radius = 4.0
            x_target = radius * math.sin(theta)
            y_target = radius * (1.0 - math.cos(theta))
            self.publish_trajectory_setpoint(x_target, y_target, -4.0)
        elif self.counter == 750:
            self.land()

        self.counter += 1

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def rtl(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.timestamp = True, int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command, msg.param1, msg.param2 = command, param1, param2
        msg.target_system, msg.target_component, msg.source_system, msg.source_component, msg.from_external = 1, 1, 1, 1, True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmartOffboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
