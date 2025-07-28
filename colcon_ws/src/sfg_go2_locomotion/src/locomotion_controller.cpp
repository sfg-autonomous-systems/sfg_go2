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

        parameter = "command_timeout";
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
            get_name() + std::string("/cmd_vel"),
            rclcpp::SensorDataQoS(),
            std::bind(&LocomotionController::velocity_callback, this, std::placeholders::_1));

        m_stop_move_timer = create_wall_timer(
            std::chrono::duration<float>(m_command_timeout),
            std::bind(&LocomotionController::stop_move_timer_callback, this));
        m_stop_move_timer->cancel();
        m_change_mode_service = create_service<sfg_agent_msgs::srv::TriggerAction>(
            get_name() + std::string("/set_locomotion_mode"),
            std::bind(&LocomotionController::change_mode_callback, this, std::placeholders::_1, std::placeholders::_2));

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

    void LocomotionController::change_mode_callback(
        const std::shared_ptr<sfg_agent_msgs::srv::TriggerAction::Request> request,
        std::shared_ptr<sfg_agent_msgs::srv::TriggerAction::Response> response)
    {
        static std::map<std::string, std::function<int32_t(unitree::robot::go2::SportClient *)>> action_map = {
            {"stand up", &unitree::robot::go2::SportClient::RecoveryStand},
            {"lay down", &unitree::robot::go2::SportClient::StandDown},
            {"damp", &unitree::robot::go2::SportClient::Damp}};

        if (m_sport_client == nullptr)
        {
            response->message = "Failed to change locomotion mode: The underlying driver is not initialized.";
            response->success = false;
            return;
        }

        if (!m_sport_mode_state.has_value())
        {
            response->message = "Failed to change locomotion mode: The current locomotion state is unknown.";
            response->success = false;
            return;
        }

        auto iterator = action_map.find(request->action);

        if (iterator == action_map.end())
        {
            response->message = "Failed to change locomotion mode: The mode '" + request->action + "' is unknown.";
            response->message += " Available modes are: ";

            for (const auto &pair : action_map)
            {
                response->message += pair.first + ", ";
            }
            response->message = response->message.substr(0, response->message.size() - 2);
            response->success = false;
            return;
        }

        auto error = iterator->second(m_sport_client.get());

        if (error)
        {
            response->message = "Failed to change locomotion mode: The underlying driver returned an error code of " + std::to_string(error) + ".";
            response->success = false;
            return;
        }

        response->success = true;
    }

    void LocomotionController::sport_mode_state_callback(const void *msg)
    {
        m_sport_mode_state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(msg);
    }
}