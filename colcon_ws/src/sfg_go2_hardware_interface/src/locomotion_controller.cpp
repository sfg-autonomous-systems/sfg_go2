#include "sfg_go2_hardware_interface/locomotion_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

using namespace std::chrono_literals;

namespace
{
    constexpr const char *TOPIC_LOWCMD = "rt/lowcmd";
    constexpr const char *TOPIC_LOWSTATE = "rt/lowstate";

    constexpr float POS_STOP_F = 2.146E9f;
    constexpr float VEL_STOP_F = 16000.0f;

    // Unitree low-level examples publish LowCmd every 2 ms = 500 Hz.
    // Your old controller only wrote when policy messages arrived, around 50 Hz.
    constexpr auto LOWCMD_WRITE_PERIOD = 2ms;

    // First direct motor diagnostic gains.
    // These are stronger than your first blended test but still conservative.
    constexpr float LOW_LEVEL_TRACK_STEP_RAD = 0.005f;
    constexpr float LOW_LEVEL_MAX_POSITION_ERROR_RAD = 0.25f;
    constexpr float LOW_LEVEL_KP = 45.0f;
    constexpr float LOW_LEVEL_KD = 3.0f;

    // Debug command limits.
    constexpr float DEBUG_MAX_SINGLE_JOINT_OFFSET_RAD = 0.20f;
}

namespace sfg_go2_hardware_interface
{
    LocomotionController::LocomotionController(const rclcpp::NodeOptions &options)
        : LocomotionControllerBase("locomotion_controller", options)
    {
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

        m_low_state_subscriber =
            std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
        m_low_state_subscriber->InitChannel(
            std::bind(&LocomotionController::low_state_callback, this, std::placeholders::_1),
            1);

        m_low_cmd_publisher =
            std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
        m_low_cmd_publisher->InitChannel();

        init_low_cmd();

        m_sport_client = std::make_unique<unitree::robot::go2::SportClient>();
        m_sport_client->SetTimeout(m_client_timeout);
        m_sport_client->Init();
        m_sport_client->AutoRecoverSet(false);

        m_motion_switcher = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
        m_motion_switcher->SetTimeout(10.0f);
        m_motion_switcher->Init();

        m_policy_lowcmd_subscriber = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/go2/policy_lowcmd",
            10,
            std::bind(&LocomotionController::policy_lowcmd_callback, this, std::placeholders::_1));

        // Direct debug topic.
        //
        // Usage:
        //   data = [motor_index, offset_rad]
        //
        // Example:
        //   ros2 topic pub --once /go2/debug_single_joint_cmd std_msgs/msg/Float32MultiArray "{data: [0.0, 0.08]}"
        //
        // Disable debug mode:
        //   ros2 topic pub --once /go2/debug_single_joint_cmd std_msgs/msg/Float32MultiArray "{data: [-1.0, 0.0]}"
        m_debug_single_joint_subscriber = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/go2/debug_single_joint_cmd",
            10,
            std::bind(&LocomotionController::debug_single_joint_callback, this, std::placeholders::_1));

        m_control_mode_subscriber = create_subscription<std_msgs::msg::String>(
            "/go2/control_mode",
            10,
            std::bind(&LocomotionController::control_mode_callback, this, std::placeholders::_1));

        // This continuously writes LowCmd at 500 Hz when POLICY_LOW_LEVEL is active.
        // This matches Unitree's low-level example structure much more closely than writing at 50 Hz.
        m_lowcmd_timer = create_wall_timer(
            LOWCMD_WRITE_PERIOD,
            std::bind(&LocomotionController::lowcmd_timer_callback, this));

        RCLCPP_INFO(
            get_logger(),
            "Started locomotion controller. Initial mode: %s",
            control_mode_to_string(m_control_mode).c_str());

        RCLCPP_INFO(
            get_logger(),
            "LowCmd writer configured for 500 Hz. Debug topic: /go2/debug_single_joint_cmd");
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

    void LocomotionController::init_low_cmd()
    {
        m_low_cmd.head()[0] = 0xFE;
        m_low_cmd.head()[1] = 0xEF;
        m_low_cmd.level_flag() = 0xFF;
        m_low_cmd.gpio() = 0;

        for (std::size_t i = 0; i < NUM_LOWCMD_MOTORS; i++)
        {
            m_low_cmd.motor_cmd()[i].mode() = 0x01;
            m_low_cmd.motor_cmd()[i].q() = POS_STOP_F;
            m_low_cmd.motor_cmd()[i].dq() = VEL_STOP_F;
            m_low_cmd.motor_cmd()[i].kp() = 0.0f;
            m_low_cmd.motor_cmd()[i].kd() = 0.0f;
            m_low_cmd.motor_cmd()[i].tau() = 0.0f;
        }
    }

    bool LocomotionController::release_motion_mode_for_low_level()
    {
        if (m_low_level_released)
        {
            return true;
        }

        if (!m_motion_switcher)
        {
            RCLCPP_ERROR(get_logger(), "MotionSwitcherClient is not initialized.");
            return false;
        }

        RCLCPP_WARN(
            get_logger(),
            "Calling MotionSwitcherClient::ReleaseMode() to enable low-level motor control.");

        const int32_t ret = m_motion_switcher->ReleaseMode();

        if (ret == 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "ReleaseMode() succeeded. Low-level motor commands should now be accepted.");

            m_low_level_released = true;
            return true;
        }

        RCLCPP_ERROR(
            get_logger(),
            "ReleaseMode() failed. ret=%d",
            ret);

        return false;
    }

    uint32_t LocomotionController::crc32_core(uint32_t *ptr, uint32_t len)
    {
        unsigned int xbit = 0;
        unsigned int data = 0;
        unsigned int CRC32 = 0xFFFFFFFF;
        const unsigned int dwPolynomial = 0x04c11db7;

        for (unsigned int i = 0; i < len; i++)
        {
            xbit = 1 << 31;
            data = ptr[i];

            for (unsigned int bits = 0; bits < 32; bits++)
            {
                if (CRC32 & 0x80000000)
                {
                    CRC32 <<= 1;
                    CRC32 ^= dwPolynomial;
                }
                else
                {
                    CRC32 <<= 1;
                }

                if (data & xbit)
                {
                    CRC32 ^= dwPolynomial;
                }

                xbit >>= 1;
            }
        }

        return CRC32;
    }

    bool LocomotionController::write_low_cmd_with_crc()
    {
        m_low_cmd.crc() = crc32_core(
            reinterpret_cast<uint32_t *>(&m_low_cmd),
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);

        return m_low_cmd_publisher->Write(m_low_cmd);
    }

    void LocomotionController::sport_mode_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_sport_mode_state_mutex);
        m_last_sport_mode_state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(msg);
    }

    void LocomotionController::low_state_callback(const void *msg)
    {
        std::lock_guard lock(m_last_low_state_mutex);
        m_last_low_state = *static_cast<const unitree_go::msg::dds_::LowState_ *>(msg);
    }

    bool LocomotionController::read_current_motor_positions(std::array<float, NUM_GO2_MOTORS> &current_q)
    {
        std::optional<unitree_go::msg::dds_::LowState_> low_state_copy;

        {
            std::lock_guard lock(m_last_low_state_mutex);
            low_state_copy = m_last_low_state;
        }

        if (!low_state_copy.has_value())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "No LowState received yet.");
            return false;
        }

        const auto &motor_state = low_state_copy.value().motor_state();

        for (std::size_t i = 0; i < NUM_GO2_MOTORS; i++)
        {
            const float q = motor_state[i].q();

            if (!std::isfinite(q))
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Non-finite LowState q at motor %zu.",
                    i);
                return false;
            }

            current_q[i] = q;
        }

        return true;
    }

    void LocomotionController::reset_low_level_tracking_from_current()
    {
        std::array<float, NUM_GO2_MOTORS> current_q{};

        if (!read_current_motor_positions(current_q))
        {
            RCLCPP_WARN(
                get_logger(),
                "Could not reset low-level tracking because current motor positions are unavailable.");
            return;
        }

        {
            std::lock_guard lock(m_low_level_target_mutex);

            m_desired_q = current_q;
            m_have_desired_q = true;

            if (!m_have_policy_target)
            {
                m_last_policy_target_q = current_q;
                m_have_policy_target = true;
            }
        }

        RCLCPP_WARN(
            get_logger(),
            "Low-level desired_q initialized from current motor positions.");
    }

    void LocomotionController::policy_lowcmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != NUM_GO2_MOTORS)
        {
            RCLCPP_WARN(
                get_logger(),
                "Received /go2/policy_lowcmd with %zu elements. Expected %zu.",
                msg->data.size(),
                NUM_GO2_MOTORS);
            return;
        }

        std::array<float, NUM_GO2_MOTORS> target_q{};

        for (std::size_t i = 0; i < NUM_GO2_MOTORS; i++)
        {
            const float q = msg->data[i];

            if (!std::isfinite(q))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Received non-finite policy target at motor %zu. Ignoring whole policy target.",
                    i);
                return;
            }

            target_q[i] = q;
        }

        {
            std::lock_guard lock(m_low_level_target_mutex);
            m_last_policy_target_q = target_q;
            m_have_policy_target = true;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Stored policy target. First policy target = %.4f",
            target_q[0]);
    }

    void LocomotionController::debug_single_joint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2)
        {
            RCLCPP_WARN(
                get_logger(),
                "Received /go2/debug_single_joint_cmd with %zu elements. Expected [motor_index, offset_rad].",
                msg->data.size());
            return;
        }

        const int motor_index = static_cast<int>(std::lround(msg->data[0]));
        const float requested_offset = msg->data[1];

        if (motor_index < 0)
        {
            std::lock_guard lock(m_low_level_target_mutex);
            m_debug_target_active = false;

            RCLCPP_WARN(
                get_logger(),
                "Debug single-joint mode disabled. Controller will use policy target again.");
            return;
        }

        if (motor_index >= static_cast<int>(NUM_GO2_MOTORS))
        {
            RCLCPP_WARN(
                get_logger(),
                "Invalid debug motor index %d. Expected 0-11, or -1 to disable debug.",
                motor_index);
            return;
        }

        if (!std::isfinite(requested_offset))
        {
            RCLCPP_WARN(
                get_logger(),
                "Non-finite debug offset. Ignoring command.");
            return;
        }

        std::array<float, NUM_GO2_MOTORS> current_q{};

        if (!read_current_motor_positions(current_q))
        {
            RCLCPP_WARN(
                get_logger(),
                "Cannot start debug single-joint mode: no current motor positions.");
            return;
        }

        const float offset = std::clamp(
            requested_offset,
            -DEBUG_MAX_SINGLE_JOINT_OFFSET_RAD,
            DEBUG_MAX_SINGLE_JOINT_OFFSET_RAD);

        std::array<float, NUM_GO2_MOTORS> debug_target = current_q;
        debug_target[static_cast<std::size_t>(motor_index)] += offset;

        {
            std::lock_guard lock(m_low_level_target_mutex);

            m_debug_target_q = debug_target;
            m_debug_target_active = true;

            // Start from current position so we ramp smoothly into the debug target.
            m_desired_q = current_q;
            m_have_desired_q = true;
        }

        RCLCPP_WARN(
            get_logger(),
            "Debug single-joint target enabled. motor=%d current=%.4f target=%.4f offset=%.4f",
            motor_index,
            current_q[static_cast<std::size_t>(motor_index)],
            debug_target[static_cast<std::size_t>(motor_index)],
            offset);
    }

    void LocomotionController::lowcmd_timer_callback()
    {
        ControlMode mode;

        {
            std::lock_guard lock(m_control_mode_mutex);
            mode = m_control_mode;
        }

        if (mode != ControlMode::POLICY_LOW_LEVEL)
        {
            return;
        }

        if (!m_low_level_released)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "POLICY_LOW_LEVEL active, but low-level mode has not been released yet.");
            return;
        }

        std::array<float, NUM_GO2_MOTORS> current_q{};

        if (!read_current_motor_positions(current_q))
        {
            return;
        }

        std::array<float, NUM_GO2_MOTORS> target_q{};
        std::array<float, NUM_GO2_MOTORS> desired_q{};
        bool debug_active = false;

        {
            std::lock_guard lock(m_low_level_target_mutex);

            if (!m_have_desired_q)
            {
                m_desired_q = current_q;
                m_have_desired_q = true;
            }

            if (m_debug_target_active)
            {
                target_q = m_debug_target_q;
                debug_active = true;
            }
            else if (m_have_policy_target)
            {
                target_q = m_last_policy_target_q;
            }
            else
            {
                // No policy target yet. Hold current.
                target_q = current_q;
            }

            for (std::size_t i = 0; i < NUM_GO2_MOTORS; i++)
            {
                if (!std::isfinite(target_q[i]))
                {
                    target_q[i] = current_q[i];
                }

                const float target_delta = target_q[i] - m_desired_q[i];

                const float desired_step = std::clamp(
                    target_delta,
                    -LOW_LEVEL_TRACK_STEP_RAD,
                    LOW_LEVEL_TRACK_STEP_RAD);

                m_desired_q[i] += desired_step;

                // Prevent desired_q from getting too far away from the measured joint.
                m_desired_q[i] = std::clamp(
                    m_desired_q[i],
                    current_q[i] - LOW_LEVEL_MAX_POSITION_ERROR_RAD,
                    current_q[i] + LOW_LEVEL_MAX_POSITION_ERROR_RAD);
            }

            desired_q = m_desired_q;
        }

        float max_position_error = 0.0f;

        for (std::size_t i = 0; i < NUM_GO2_MOTORS; i++)
        {
            const float position_error = desired_q[i] - current_q[i];
            max_position_error = std::max(max_position_error, std::abs(position_error));

            m_low_cmd.motor_cmd()[i].mode() = 0x01;
            m_low_cmd.motor_cmd()[i].q() = desired_q[i];
            m_low_cmd.motor_cmd()[i].dq() = 0.0f;
            m_low_cmd.motor_cmd()[i].kp() = LOW_LEVEL_KP;
            m_low_cmd.motor_cmd()[i].kd() = LOW_LEVEL_KD;
            m_low_cmd.motor_cmd()[i].tau() = 0.0f;
        }

        for (std::size_t i = NUM_GO2_MOTORS; i < NUM_LOWCMD_MOTORS; i++)
        {
            m_low_cmd.motor_cmd()[i].mode() = 0x01;
            m_low_cmd.motor_cmd()[i].q() = POS_STOP_F;
            m_low_cmd.motor_cmd()[i].dq() = VEL_STOP_F;
            m_low_cmd.motor_cmd()[i].kp() = 0.0f;
            m_low_cmd.motor_cmd()[i].kd() = 0.0f;
            m_low_cmd.motor_cmd()[i].tau() = 0.0f;
        }

        const bool ok = write_low_cmd_with_crc();

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "500Hz LowCmd source=%s motor0 current=%.4f target=%.4f desired=%.4f err=%.4f max_err=%.4f kp=%.1f kd=%.1f write_ok=%d",
            debug_active ? "DEBUG_SINGLE_JOINT" : "POLICY",
            current_q[0],
            target_q[0],
            desired_q[0],
            desired_q[0] - current_q[0],
            max_position_error,
            LOW_LEVEL_KP,
            LOW_LEVEL_KD,
            static_cast<int>(ok));
    }

    void LocomotionController::control_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        const auto new_mode = control_mode_from_string(msg->data);

        ControlMode old_mode;

        {
            std::lock_guard lock(m_control_mode_mutex);
            old_mode = m_control_mode;
        }

        if (new_mode == old_mode)
        {
            return;
        }

        if (new_mode == ControlMode::POLICY_LOW_LEVEL)
        {
            // Important: release Unitree mode BEFORE setting m_control_mode to POLICY_LOW_LEVEL.
            // Otherwise the LowCmd timer could start writing while ReleaseMode is still in progress.
            if (!release_motion_mode_for_low_level())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to release Unitree motion mode. Refusing POLICY_LOW_LEVEL.");
                return;
            }

            reset_low_level_tracking_from_current();
        }

        if (new_mode == ControlMode::SPORT || new_mode == ControlMode::IDLE)
        {
            std::lock_guard target_lock(m_low_level_target_mutex);
            m_debug_target_active = false;
            m_have_desired_q = false;
        }

        {
            std::lock_guard lock(m_control_mode_mutex);
            m_control_mode = new_mode;
        }

        RCLCPP_INFO(
            get_logger(),
            "Switched control mode from %s to %s",
            control_mode_to_string(old_mode).c_str(),
            control_mode_to_string(new_mode).c_str());
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