#ifndef H1_ARM_HW_HPP_
#define H1_ARM_HW_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "h1_arm_control_hardware/visibility_control.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Direct Unitree H1 includes
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace h1_platform
{
    class H1ArmPositionHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(H1ArmPositionHardware);

        h1_arm_control_hardware_PUBLIC
            hardware_interface::CallbackReturn
            on_init(const hardware_interface::HardwareInfo &info) override;

        h1_arm_control_hardware_PUBLIC
            std::vector<hardware_interface::StateInterface>
            export_state_interfaces() override;

        h1_arm_control_hardware_PUBLIC
            std::vector<hardware_interface::CommandInterface>
            export_command_interfaces() override;

        h1_arm_control_hardware_PUBLIC
            hardware_interface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State &prev_state) override;

        h1_arm_control_hardware_PUBLIC
            hardware_interface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State &prev_state) override;

        h1_arm_control_hardware_PUBLIC
            hardware_interface::return_type
            read(const rclcpp::Time &time,
                 const rclcpp::Duration &period) override;

        h1_arm_control_hardware_PUBLIC
            hardware_interface::return_type
            write(const rclcpp::Time &time,
                  const rclcpp::Duration &period) override;

    private:
        // Parameters for the H1
        double hw_start_sec_{0.0};
        double hw_stop_sec_{2.0};
        double hw_slowdown_{1.0};
        std::string network_interface_{"eth0"};

        // Store the command for the H1 robot - 14 DoF (7+7 arms only, no waist in ROS2)
        std::vector<double> hw_commands_;
        std::vector<double> hw_states_;

        // Permutations: our 14-joint index <-> ROS2 control index
        std::array<int, 14> perm_14_to_hw_{}; // 14-joint index -> hw index
        std::array<int, 14> perm_hw_to14_{};  // hw index -> 14-joint index

        // Direct Unitree communication
        std::shared_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> unitree_arm_sdk_publisher_;
        std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> unitree_state_subscriber_;

        // Unitree message storage
        unitree_hg::msg::dds_::LowCmd_ unitree_msg_;
        unitree_hg::msg::dds_::LowState_ unitree_state_;

        // Thread safety for state reading
        std::mutex state_mutex_;
        std::atomic<bool> state_received_{false};

        // Control parameters from reference code
        float weight_{0.0f};
        float weight_rate_{0.2f};
        float control_dt_{0.02f};
        float max_joint_velocity_{0.5f};

        // Gain arrays for 14 arm joints only (no waist)
        std::array<float, 14> kp_array_;
        std::array<float, 14> kd_array_;

        // Safe initial positions to avoid collision
        std::array<float, 14> safe_init_pos_;

        // Track previous commands for velocity limiting
        std::array<float, 14> prev_commands_;

        // Track if we're in safe mode (weight < 1.0)
        bool safe_mode_{true};

        // Flag to prevent write during activation
        std::atomic<bool> activation_in_progress_{false};
    };

} // namespace h1_platform

#endif // H1_ARM_HW_HPP_