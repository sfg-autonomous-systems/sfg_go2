#include "sfg_go2_hardware_interface/locomotion_controller.hpp"

#include "sfg_utils/fqn/ros_fqn_builder.hpp"

namespace sfg_go2_hardware_interface
{
    const std::map<std::string, std::function<int32_t(unitree::robot::go2::SportClient *)>> LocomotionController::s_state_map = {
        {"stand up", &unitree::robot::go2::SportClient::RecoveryStand},
        {"lay down", &unitree::robot::go2::SportClient::StandDown},
        {"damp", &unitree::robot::go2::SportClient::Damp}};

    LocomotionController::LocomotionController(const rclcpp::NodeOptions &options) : Node("locomotion_controller", options)
    {
        // Declare and retrieve ROS parameters.
        m_network_interface = declare_parameter<std::string>(
            "network_interface",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("The name of the network interface to use for communication with the robot."));

        m_client_timeout = declare_parameter(
            "client_timeout",
            10.0f,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Timeout for the SportClient in seconds."));

        m_command_timeout = declare_parameter(
            "command_timeout",
            0.25f,
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description(
                    "Messages that have been received with a timestamp older "
                    "than this value will be ignored. Unit is seconds."));

        m_command_speed_limits = Eigen::Matrix<double, 3, 2>(
                                     declare_parameter(
                                         "speed_limits",
                                         std::vector<double>{0.5, 0.5, 0.5, 0.5, 0.5, 1.5},
                                         rcl_interfaces::msg::ParameterDescriptor()
                                             .set__description(
                                                 "The speed limits along the robot's x, y, and yaw axes. "
                                                 "Each consecutive pair of values defines the minimum and maximum absolute speed for the respective axis. "
                                                 "The first two values are for x, the next two for y, and the last two for yaw."))
                                         .data())
                                     .cast<float>();

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);
        m_sport_mode_state_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
        m_sport_mode_state_subscriber->InitChannel(std::bind(&LocomotionController::sport_mode_state_callback, this, std::placeholders::_1));

        m_sport_client = std::make_shared<unitree::robot::go2::SportClient>();
        m_sport_client->SetTimeout(m_client_timeout);
        m_sport_client->Init();
        m_sport_client->AutoRecoverSet(false);

        // Set up interfaces.
        m_cmd_vel_subscriber = create_subscription<geometry_msgs::msg::TwistStamped>(
            sfg_utils::fqn::RosFqnBuilder().resource(sfg_utils::fqn::Resource::CmdVel).build(sfg_utils::fqn::RosFqnSegment::Resource),
            10,
            std::bind(&LocomotionController::cmd_vel_callback, this, std::placeholders::_1));
        m_apply_move_timer = create_wall_timer(
            std::chrono::duration<float>(0.05f),
            std::bind(&LocomotionController::apply_move_callback, this));
        m_set_state_service = create_service<sfg_agent_msgs::srv::TriggerAction>(
            sfg_utils::fqn::RosFqnBuilder().resource(sfg_utils::fqn::Resource::Custom, "set_state").build(sfg_utils::fqn::RosFqnSegment::Resource),
            std::bind(&LocomotionController::set_state_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Started locomotion controller.");
    }

    void LocomotionController::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg)
    {
        m_last_cmd_vel = *msg;
    }

    void LocomotionController::apply_move_callback()
    {
        if (m_sport_client == nullptr)
        {
            return;
        }

        Eigen::Vector3f target_speed = Eigen::Vector3f::Zero();

        if (now() - m_last_cmd_vel.header.stamp <= rclcpp::Duration::from_seconds(m_command_timeout))
        {
            target_speed = {
                static_cast<float>(m_last_cmd_vel.twist.linear.x),
                static_cast<float>(m_last_cmd_vel.twist.linear.y),
                static_cast<float>(m_last_cmd_vel.twist.angular.z),
            };
            Eigen::Vector3f sign = target_speed.cwiseSign();
            target_speed = target_speed.cwiseAbs().cwiseMin(m_command_speed_limits.col(1)).cwiseMax(m_command_speed_limits.col(0)).cwiseProduct(sign);
        }

        if (auto error = m_sport_client->Move(target_speed[0], target_speed[1], target_speed[2]))
        {
            RCLCPP_WARN(get_logger(), "Failed to send move command [vx=%.2f, vy=%.2f, vyaw=%.2f]: The underlying driver returned an error code of %d.", target_speed[0], target_speed[1], target_speed[2], error);
        }
    }

    void LocomotionController::set_state_callback(
        const std::shared_ptr<sfg_agent_msgs::srv::TriggerAction::Request> request,
        std::shared_ptr<sfg_agent_msgs::srv::TriggerAction::Response> response)
    {
        if (m_sport_client == nullptr)
        {
            response->message = "Failed to change locomotion state: The underlying driver is not initialized.";
            response->success = false;
            return;
        }

        {
            std::lock_guard lock(m_last_sport_mode_state_mutex);

            if (!m_last_sport_mode_state.has_value())
            {
                response->message = "Failed to change locomotion state: The current locomotion state is unknown.";
                response->success = false;
                return;
            }
        }

        auto iterator = s_state_map.find(request->action);

        if (iterator == s_state_map.end())
        {
            response->message = "Failed to change locomotion state: The state '" + request->action + "' is unknown.\n";
            response->message += "Available states are: ";

            for (const auto &pair : s_state_map)
            {
                response->message += pair.first + ", ";
            }
            response->message = response->message.substr(0, response->message.size() - 2);
            response->success = false;
            return;
        }

        if (auto error = iterator->second(m_sport_client.get()))
        {
            response->message = "Failed to change locomotion state: The underlying driver returned an error code of " + std::to_string(error) + ".";
            response->success = false;
            return;
        }

        response->success = true;
    }

    void LocomotionController::sport_mode_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_sport_mode_state_mutex);
        m_last_sport_mode_state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(msg);
    }
}