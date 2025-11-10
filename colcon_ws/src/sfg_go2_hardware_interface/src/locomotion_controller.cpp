#include "sfg_go2_hardware_interface/locomotion_controller.hpp"

namespace sfg_go2_hardware_interface
{
    LocomotionController::LocomotionController(const rclcpp::NodeOptions &options) : LocomotionControllerBase("locomotion_controller", options)
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

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);
        m_sport_mode_state_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
        m_sport_mode_state_subscriber->InitChannel(std::bind(&LocomotionController::sport_mode_state_callback, this, std::placeholders::_1));

        m_sport_client = std::make_unique<unitree::robot::go2::SportClient>();
        m_sport_client->SetTimeout(m_client_timeout);
        m_sport_client->Init();
        m_sport_client->AutoRecoverSet(false);

        RCLCPP_INFO(get_logger(), "Started locomotion controller.");
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
        if (auto error = m_sport_client->Move(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z))
        {
            throw std::runtime_error("The underlying driver returned an error code of '" + std::to_string(error) + "'.");
        }
    }

    void LocomotionController::sport_mode_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_sport_mode_state_mutex);
        m_last_sport_mode_state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(msg);
    }
}