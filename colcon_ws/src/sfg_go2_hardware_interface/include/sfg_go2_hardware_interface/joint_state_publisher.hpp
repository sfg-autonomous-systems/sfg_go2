#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace sfg_go2_hardware_interface
{
    class JointStatePublisher : public rclcpp::Node
    {
    public:
        JointStatePublisher(const rclcpp::NodeOptions &options);

    private:
        void lowstate_callback(const void *msg);
        void publish_joint_states();

        // ROS parameters
        std::string m_network_interface;
        float m_publish_rate;
        std::vector<std::string> m_joint_names;

        unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> m_lowstate_subscriber;
        unitree_go::msg::dds_::LowState_ m_last_low_state;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
        rclcpp::TimerBase::SharedPtr m_publish_joint_states_timer;
    };
}