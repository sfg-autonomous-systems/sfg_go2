#include "sfg_go2_hardware_interface/locomotion_controller.hpp"

namespace sfg_go2_hardware_interface
{
    LocomotionController::LocomotionController(const rclcpp::NodeOptions &options)
        : LocomotionControllerBase("locomotion_controller", options)
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

        m_initial_control_mode = declare_parameter<std::string>(
            "initial_control_mode",
            "SPORT",
            rcl_interfaces::msg::ParameterDescriptor()
                .set__description("Initial control mode. One of: SPORT, POLICY_LOW_LEVEL, IDLE."));

        m_control_mode = control_mode_from_string(m_initial_control_mode);

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);

        m_sport_mode_state_subscriber =
            std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
        m_sport_mode_state_subscriber->InitChannel(
            std::bind(&LocomotionController::sport_mode_state_callback, this, std::placeholders::_1));

        m_sport_client = std::make_unique<unitree::robot::go2::SportClient>();
        m_sport_client->SetTimeout(m_client_timeout);
        m_sport_client->Init();
        m_sport_client->AutoRecoverSet(false);

        // Stage 1 placeholder: subscribe to a 12-element float array.
        m_policy_lowcmd_subscriber = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/go2/policy_lowcmd",
            10,
            std::bind(&LocomotionController::policy_lowcmd_callback, this, std::placeholders::_1));

        m_control_mode_subscriber = create_subscription<std_msgs::msg::String>(
            "/go2/control_mode",
            10,
            std::bind(&LocomotionController::control_mode_callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Started locomotion controller. Initial mode: %s",
            control_mode_to_string(m_control_mode).c_str());
    }

    void LocomotionController::arm()
    {
        m_sport_client->RecoveryStand();
    }

    void LocomotionController::disarm()
    {
        m_sport_client->StandDown();
    }

    void LocomotionController::emergency_stop()
    {
        m_sport_client->Damp();
    }

    void LocomotionController::apply_cmd(const geometry_msgs::msg::TwistStamped &cmd)
    {
        std::lock_guard lock(m_control_mode_mutex);

        if (m_control_mode != ControlMode::SPORT)
        {
            return;
        }

        if (auto error = m_sport_client->Move(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z))
        {
            throw std::runtime_error("The underlying driver returned an error code of '" + std::to_string(error) + "'.");
        }
    }

    void LocomotionController::policy_lowcmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::lock_guard lock(m_control_mode_mutex);

        if (m_control_mode != ControlMode::POLICY_LOW_LEVEL)
        {
            return;
        }

        if (msg->data.size() != 12)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Received /go2/policy_lowcmd with %zu elements. Expected 12.",
                msg->data.size());
            return;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Received /go2/policy_lowcmd in POLICY_LOW_LEVEL mode. First action = %.4f",
            msg->data[0]);
    }

    void LocomotionController::control_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        const auto new_mode = control_mode_from_string(msg->data);

        {
            std::lock_guard lock(m_control_mode_mutex);
            if (new_mode == m_control_mode)
            {
                return;
            }

            m_control_mode = new_mode;
        }

        RCLCPP_INFO(
            get_logger(),
            "Switched control mode to %s",
            control_mode_to_string(new_mode).c_str());
    }

    void LocomotionController::sport_mode_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_sport_mode_state_mutex);
        m_last_sport_mode_state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(msg);
    }

    std::string LocomotionController::control_mode_to_string(ControlMode mode)
    {
        switch (mode)
        {
            case ControlMode::SPORT:
                return "SPORT";
            case ControlMode::POLICY_LOW_LEVEL:
                return "POLICY_LOW_LEVEL";
            case ControlMode::IDLE:
                return "IDLE";
            default:
                return "UNKNOWN";
        }
    }

    LocomotionController::ControlMode LocomotionController::control_mode_from_string(const std::string &mode)
    {
        if (mode == "SPORT")
        {
            return ControlMode::SPORT;
        }
        if (mode == "POLICY_LOW_LEVEL")
        {
            return ControlMode::POLICY_LOW_LEVEL;
        }
        if (mode == "IDLE")
        {
            return ControlMode::IDLE;
        }

        throw std::runtime_error(
            "Invalid control mode '" + mode + "'. Expected SPORT, POLICY_LOW_LEVEL, or IDLE.");
    }
}