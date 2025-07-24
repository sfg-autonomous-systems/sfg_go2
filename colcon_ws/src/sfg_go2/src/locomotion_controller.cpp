#include "sfg_go2/locomotion_controller.hpp"

namespace sfg_go2
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

        unitree::robot::ChannelFactory::Instance()->Init(0, m_network_interface);
        m_sport_client = std::make_shared<unitree::robot::go2::SportClient>();
        m_sport_client->SetTimeout(10.0f);
        m_sport_client->Init();
        m_sport_client->StandUp();

        RCLCPP_INFO(get_logger(), "Started locomotion controller.");
    }
}