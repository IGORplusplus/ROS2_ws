#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

class CustomMode : public rclcpp::Node {
    public:
	CustomMode();

    private:
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

	rclcpp::TimerBase::SharedPtr timer_;

	int setpoint_counter_;

	void timer_callback();
	void publish_custom_mode();
	void publish_trajectory_setpoint();
	void set_offboard_mode();
	void arm();
	uint64_t now_us();
};
