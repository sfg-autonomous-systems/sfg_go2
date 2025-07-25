#include "sfg_go2_locomotion/locomotion_controller.hpp"

namespace sfg_go2_locomotion
{
    LocomotionController::LocomotionController(const rclcpp::NodeOptions &options) : Node("locomotion_controller", options)
    {
        // Declare and retrieve ROS parameters.
        std::string parameter = "network_interface";
        declare_parameter<std::string>(
            parameter,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The name of the network interface to use for communication with the robot."));
        get_parameter(parameter, m_network_interface);

        parameter = "client_timeout";
        declare_parameter(
            parameter,
            10.0f,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Timeout for the SportClient in seconds."));
        get_parameter(parameter, m_client_timeout);

        parameter = "msg_timeout";
        declare_parameter(
            parameter,
            0.25f,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description(
                    "Messages that have been received with a timestamp older "
                    "than this value will be ignored. Unit is seconds."));
        get_parameter(parameter, m_command_timeout);

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);

        m_sport_mode_state_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
        m_sport_mode_state_subscriber->InitChannel(std::bind(&LocomotionController::sport_mode_state_callback, this, std::placeholders::_1));

        m_sport_client = std::make_shared<unitree::robot::go2::SportClient>();
        m_sport_client->SetTimeout(m_client_timeout);
        m_sport_client->Init();
        m_sport_client->AutoRecoverSet(false);

        // Set up interfaces.
        m_velocity_subscriber = create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&LocomotionController::velocity_callback, this, std::placeholders::_1));

        m_stop_move_timer = create_wall_timer(
            std::chrono::duration<float>(m_command_timeout),
            std::bind(&LocomotionController::stop_move_timer_callback, this));
        m_stop_move_timer->cancel();

        RCLCPP_INFO(get_logger(), "Started locomotion controller.");
    }

    void LocomotionController::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (m_sport_client == nullptr)
        {
            return;
        }

        if (get_clock()->now() - msg->header.stamp > rclcpp::Duration::from_seconds(m_command_timeout))
        {
            RCLCPP_WARN(get_logger(), "Received velocity command with outdated timestamp. Ignoring command.");
            return;
        }

        m_sport_client->Move(msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);
        m_stop_move_timer->reset();
    }

    void LocomotionController::stop_move_timer_callback()
    {
        if (m_sport_client == nullptr)
        {
            return;
        }

        m_sport_client->StopMove();
    }

    void LocomotionController::sport_mode_state_callback(const void *msg)
    {
        m_sport_mode_state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(msg);
    }
}