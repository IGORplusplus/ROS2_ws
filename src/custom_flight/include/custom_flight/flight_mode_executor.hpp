#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "custom_flight_interfaces/action/takeoff.hpp"
#include "custom_flight_interfaces/action/move.hpp"
#include "custom_flight_interfaces/action/landing.hpp"
#include "custom_flight_interfaces/srv/get_state.hpp"
#include "custom_flight_interfaces/msg/drone_state.hpp"

#include <vector>
#include <cmath>
#include <cstdint>

class FlightModeExecutor : public rclcpp::Node
{
public:
    FlightModeExecutor();

private:
    enum class State
    {
        INIT,
        ARMING,
        TAKEOFF,
        WAYPOINT_1,
        WAYPOINT_2,
        WAYPOINT_3,
        WAYPOINT_4,
        LANDING,
        COMPLETE
    };

    void timer_callback();

    void publish_offboard_control_mode();
    void publish_position_setpoint(float x, float y, float z, float yaw);

    void arm();
    void disarm();
    void land();
    void set_offboard_mode();

    bool reached_position(
        float target_x,
        float target_y,
        float target_z);

    // ROS 2 publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_control_mode_pub_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_pub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_pub_;

    // ROS 2 subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        local_position_sub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
        vehicle_status_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Vehicle state
    px4_msgs::msg::VehicleLocalPosition local_position_;
    px4_msgs::msg::VehicleStatus vehicle_status_;

    bool position_received_{false};
    bool status_received_{false};

    // State machine
    State state_{State::INIT};

    // Waypoint parameters
    float takeoff_altitude_{-5.0f};

    float takeoff_x_{0.0f};
    float takeoff_y_{0.0f};

    float waypoint_1_x_{5.0f};
    float waypoint_1_y_{0.0f};

    float waypoint_2_x_{5.0f};
    float waypoint_2_y_{5.0f};

    float waypoint_3_x_{0.0f};
    float waypoint_3_y_{5.0f};

    float waypoint_4_x_{0.0f};
    float waypoint_4_y_{0.0f};

    float position_tolerance_{0.5f};

    // PX4 requires timestamps in microseconds
    uint64_t timestamp() const;
};
