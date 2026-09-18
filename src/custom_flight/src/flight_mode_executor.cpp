#include "custom_flight/flight_mode_executor.hpp"

#include <chrono>

using namespace std::chrono_literals;

FlightModeExecutor::FlightModeExecutor()
    : Node("flight_mode_executor")
{
    // ---------------------------------------------------------
    // Publishers
    // ---------------------------------------------------------

    offboard_control_mode_pub_ =
        create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode",
            10);

    trajectory_setpoint_pub_ =
        create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint",
            10);

    vehicle_command_pub_ =
        create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command",
            10);

    // ---------------------------------------------------------
    // Subscribers
    // ---------------------------------------------------------

    local_position_sub_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
            {
                local_position_ = *msg;
                position_received_ = true;
            });

    vehicle_status_sub_ =
        create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v4",
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
            {
                vehicle_status_ = *msg;
                status_received_ = true;
            });

    // ---------------------------------------------------------
    // Main control loop
    // ---------------------------------------------------------

    timer_ = create_wall_timer(
        100ms,
        std::bind(&FlightModeExecutor::timer_callback, this));

    RCLCPP_INFO(
        get_logger(),
        "Custom Flight Mode Executor started");
}


// =============================================================
// Main state machine
// =============================================================

void FlightModeExecutor::timer_callback()
{
    if (!position_received_ || !status_received_)
    {
        return;
    }

    // PX4 needs continuous offboard heartbeat.
    publish_offboard_control_mode();

    switch (state_)
    {
        case State::INIT:
        {
            RCLCPP_INFO(
                get_logger(),
                "Starting flight sequence");

            // PX4 requires a stream of offboard messages
            // before entering offboard mode.
            publish_position_setpoint(
                local_position_.x,
                local_position_.y,
                takeoff_altitude_,
                0.0f);

            static int counter = 0;

            counter++;

            if (counter > 20)
            {
                set_offboard_mode();
                arm();

                state_ = State::TAKEOFF;

                RCLCPP_INFO(
                    get_logger(),
                    "Transition -> TAKEOFF");
            }

            break;
        }

	case State::TAKEOFF:
	{
	    // Hold the takeoff X/Y position and climb to -2 m.
	    publish_position_setpoint(
		    takeoff_x_,
		    takeoff_y_,
		    takeoff_altitude_,
		    local_position_.heading);

	    RCLCPP_INFO_THROTTLE(
		    get_logger(),
		    *get_clock(),
		    1000,
		    "TAKEOFF: x=%.2f y=%.2f z=%.2f | target z=%.2f | armed=%d | nav_state=%d",
		    local_position_.x,
		    local_position_.y,
		    local_position_.z,
		    takeoff_altitude_,
		    vehicle_status_.arming_state,
		    vehicle_status_.nav_state);

	    if (std::abs(
			local_position_.z - takeoff_altitude_)
		    < position_tolerance_)
	    {
		RCLCPP_INFO(
			get_logger(),
			"Takeoff complete");

		state_ = State::WAYPOINT_1;
	    }

	    break;
	}

        case State::WAYPOINT_1:
        {
            publish_position_setpoint(
                waypoint_1_x_,
                waypoint_1_y_,
                takeoff_altitude_,
                0.0f);

            if (reached_position(
                    waypoint_1_x_,
                    waypoint_1_y_,
                    takeoff_altitude_))
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Reached waypoint 1");

                state_ = State::WAYPOINT_2;
            }

            break;
        }

        case State::WAYPOINT_2:
        {
            publish_position_setpoint(
                waypoint_2_x_,
                waypoint_2_y_,
                takeoff_altitude_,
                0.0f);

            if (reached_position(
                    waypoint_2_x_,
                    waypoint_2_y_,
                    takeoff_altitude_))
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Reached waypoint 2");

                state_ = State::WAYPOINT_3;
            }

            break;
        }

        case State::WAYPOINT_3:
        {
            publish_position_setpoint(
                waypoint_3_x_,
                waypoint_3_y_,
                takeoff_altitude_,
                0.0f);

            if (reached_position(
                    waypoint_3_x_,
                    waypoint_3_y_,
                    takeoff_altitude_))
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Reached waypoint 3");

                state_ = State::WAYPOINT_4;
            }

            break;
        }

        case State::WAYPOINT_4:
        {
            publish_position_setpoint(
                waypoint_4_x_,
                waypoint_4_y_,
                takeoff_altitude_,
                0.0f);

            if (reached_position(
                    waypoint_4_x_,
                    waypoint_4_y_,
                    takeoff_altitude_))
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Reached waypoint 4");

                state_ = State::LANDING;

                land();
            }

            break;
        }

        case State::LANDING:
        {
            // Once PX4 receives LAND, the flight controller
            // takes care of the actual landing.

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Landing...");

            if (vehicle_status_.arming_state ==
                px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
            {
                state_ = State::COMPLETE;
            }

            break;
        }

        case State::COMPLETE:
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Flight complete");

            break;
        }

        case State::ARMING:
        {
            // Reserved for future explicit arming state.
            break;
        }
    }
}


// =============================================================
// PX4 Offboard heartbeat
// =============================================================

void FlightModeExecutor::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};

    msg.timestamp = timestamp();

    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub_->publish(msg);
}


// =============================================================
// Position setpoint
// =============================================================

void FlightModeExecutor::publish_position_setpoint(
    float x,
    float y,
    float z,
    float yaw)
{
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = timestamp();

    msg.position = {
        x,
        y,
        z
    };

    msg.yaw = yaw;

    trajectory_setpoint_pub_->publish(msg);
}


// =============================================================
// Arm
// =============================================================

void FlightModeExecutor::arm()
{
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp = timestamp();

    msg.command =
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;

    msg.param1 = 1.0f;

    msg.target_system = 1;
    msg.target_component = 1;

    msg.source_system = 1;
    msg.source_component = 1;

    msg.from_external = true;

    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
        get_logger(),
        "Arm command sent");
}


// =============================================================
// Disarm
// =============================================================

void FlightModeExecutor::disarm()
{
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp = timestamp();

    msg.command =
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;

    msg.param1 = 0.0f;

    msg.target_system = 1;
    msg.target_component = 1;

    msg.source_system = 1;
    msg.source_component = 1;

    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
}


// =============================================================
// Switch to Offboard
// =============================================================

void FlightModeExecutor::set_offboard_mode()
{
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp = timestamp();

    msg.command =
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

    msg.param1 = 1.0f;
    msg.param2 = 6.0f;

    msg.target_system = 1;
    msg.target_component = 1;

    msg.source_system = 1;
    msg.source_component = 1;

    msg.from_external = true;

    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
        get_logger(),
        "Offboard mode command sent");
}


// =============================================================
// Land
// =============================================================

void FlightModeExecutor::land()
{
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp = timestamp();

    msg.command =
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;

    msg.target_system = 1;
    msg.target_component = 1;

    msg.source_system = 1;
    msg.source_component = 1;

    msg.from_external = true;

    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
        get_logger(),
        "Land command sent");
}


// =============================================================
// Position reached?
// =============================================================

bool FlightModeExecutor::reached_position(
    float target_x,
    float target_y,
    float target_z)
{
    const float dx =
        local_position_.x - target_x;

    const float dy =
        local_position_.y - target_y;

    const float dz =
        local_position_.z - target_z;

    const float distance =
        std::sqrt(
            dx * dx +
            dy * dy +
            dz * dz);

    return distance < position_tolerance_;
}


// =============================================================
// Timestamp
// =============================================================

uint64_t FlightModeExecutor::timestamp() const
{
    return static_cast<uint64_t>(
        this->get_clock()->now().nanoseconds() / 1000);
}


// =============================================================
// Main
// =============================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<FlightModeExecutor>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
