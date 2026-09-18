#include "test_node/px4_ros2_communication.hpp"

#include <cmath>
#include <memory>
#include <chrono>
#include <cstdint>

using namespace std::chrono_literals;

CustomMode::CustomMode() : Node("custom_mode"), setpoint_counter_(0) {
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&CustomMode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "CustomMode node started");
}

void CustomMode::timer_callback() {

    //tell PX4 we are providing offboard control
    publish_custom_mode();

    //tell PX4 where we want the vehicle to go
    publish_trajectory_setpoint();

    if(setpoint_counter_ == 10) {
	RCLCPP_INFO(this->get_logger(), "Requesting offboard control from PX4");
	set_offboard_mode();

	RCLCPP_INFO(this->get_logger(), "Requesting arm");
	arm();
    }

    if(setpoint_counter_ < 11) {
	++setpoint_counter_;
    }
}

void CustomMode::publish_custom_mode() {
    px4_msgs::msg::OffboardControlMode msg{};

    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;

    msg.timestamp = now_us();

    offboard_control_mode_pub_->publish(msg);
}

void CustomMode::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    /*
     * PX4 uses the NED coordinate system:
     *
     *   X = North
     *   Y = East
     *   Z = Down
     *
     * Therefore:
     *
     *   Z = -2
     *
     * means 2 meters above the origin.
     */
    msg.position[0] = 0.0;
    msg.position[1] = 0.0;
    msg.position[2] = -2.0;

    msg.yaw = NAN;
    msg.timestamp = now_us();

    trajectory_setpoint_pub_->publish(msg);
}

void CustomMode::set_offboard_mode() {
    px4_msgs::msg::VehicleCommand msg{};

    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

    msg.param1 = 1.0;
    msg.param2 = 6.0;

    msg.target_system = 1;
    msg.target_component = 1;

    msg.source_system = 1;
    msg.source_component = 1;

    msg.from_external = true;

    msg.timestamp = now_us();

    vehicle_command_pub_->publish(msg);
}

void CustomMode::arm() {
    px4_msgs::msg::VehicleCommand msg{};

    msg.command =
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;

    // 1 = ARM
    msg.param1 = 1.0;

    msg.target_system = 1;
    msg.target_component = 1;

    msg.source_system = 1;
    msg.source_component = 1;

    msg.from_external = true;

    msg.timestamp = now_us();

    vehicle_command_pub_->publish(msg);
}

uint64_t CustomMode::now_us() {
    return this->get_clock()->now().nanoseconds() / 1000;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CustomMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
