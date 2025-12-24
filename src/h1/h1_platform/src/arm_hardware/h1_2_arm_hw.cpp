#include "h1_arm_control_hardware/h1_2_arm_hw.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Direct Unitree H1 includes (matching your reference code)
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{
  // Topic names from reference code
  static const std::string kTopicArmSDK = "rt/arm_sdk";
  static const std::string kTopicState = "rt/lowstate";

  // Joint names matching your ROS2 control setup
  static const std::array<const char *, 7> LEFT_NAMES = {
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"};

  static const std::array<const char *, 7> RIGHT_NAMES = {
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};

  // H1 Joint indices from reference code
  enum JointIndex
  {
    // Left leg
    kLeftHipYaw = 0,
    kLeftHipPitch = 1,
    kLeftHipRoll = 2,
    kLeftKnee = 3,
    kLeftAnkle = 4,
    kLeftAnkleRoll = 5,
    // Right leg
    kRightHipYaw = 6,
    kRightHipPitch = 7,
    kRightHipRoll = 8,
    kRightKnee = 9,
    kRightAnkle = 10,
    kRightAnkleRoll = 11,

    kWaistYaw = 12,

    // Left arm (indices 13-19)
    kLeftShoulderPitch = 13,
    kLeftShoulderRoll = 14,
    kLeftShoulderYaw = 15,
    kLeftElbow = 16,
    kLeftWristRoll = 17,
    kLeftWristPitch = 18,
    kLeftWristYaw = 19,
    // Right arm (indices 20-26)
    kRightShoulderPitch = 20,
    kRightShoulderRoll = 21,
    kRightShoulderYaw = 22,
    kRightElbow = 23,
    kRightWristRoll = 24,
    kRightWristPitch = 25,
    kRightWristYaw = 26,

    kNotUsedJoint = 27,
    kNotUsedJoint1 = 28,
    kNotUsedJoint2 = 29,
    kNotUsedJoint3 = 30,
    kNotUsedJoint4 = 31,
    kNotUsedJoint5 = 32,
    kNotUsedJoint6 = 33,
    kNotUsedJoint7 = 34
  };

  // 14-joint array for arms only (no waist)
  static const std::array<JointIndex, 14> arm_joints = {
      JointIndex::kLeftShoulderPitch, JointIndex::kLeftShoulderRoll,
      JointIndex::kLeftShoulderYaw, JointIndex::kLeftElbow,
      JointIndex::kLeftWristRoll, JointIndex::kLeftWristPitch,
      JointIndex::kLeftWristYaw,
      JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
      JointIndex::kRightShoulderYaw, JointIndex::kRightElbow,
      JointIndex::kRightWristRoll, JointIndex::kRightWristPitch,
      JointIndex::kRightWristYaw};

  inline void log_perm(const std::array<int, 14> &a, const char *tag)
  {
    std::stringstream ss;
    ss << tag << ": ";
    for (int i = 0; i < 14; ++i)
    {
      ss << a[i];
      if (i + 1 < 14)
        ss << ", ";
    }
    RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "%s", ss.str().c_str());
  }
} // namespace

namespace h1_platform
{

  hardware_interface::CallbackReturn
  H1ArmPositionHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    // Base class stores info_ and performs basic validation
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parameters with defaults if absent
    auto get_param = [&](const std::string &key, const std::string &def) -> std::string
    {
      auto it = info_.hardware_parameters.find(key);
      return it == info_.hardware_parameters.end() ? def : it->second;
    };

    hw_start_sec_ = std::stod(get_param("hw_start_duration_sec", "2.0")); // Increased to 2.0 for safer init
    hw_stop_sec_ = std::stod(get_param("hw_stop_duration_sec", "2.0"));
    hw_slowdown_ = std::stod(get_param("hw_slowdown", "1.0"));
    network_interface_ = get_param("network_interface", "eth0");

    // Validate we have exactly 14 joints (no waist in ROS2 control)
    if (info_.joints.size() != 14)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("H1ArmPositionHardware"),
          "Expected exactly 14 joints (7+7 arms), got %zu", info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 14 DoF (7 left + 7 right) - no waist in ROS2 control
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Validate interfaces
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("H1ArmPositionHardware"),
            "Joint '%s' has %zu command interfaces; 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("H1ArmPositionHardware"),
            "Joint '%s' command IF '%s'; '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("H1ArmPositionHardware"),
            "Joint '%s' has %zu state interfaces; 1 expected.",
            joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("H1ArmPositionHardware"),
            "Joint '%s' state IF '%s'; '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // -------- Build name-based permutations (14 joint mapping) --------
    try
    {
      std::unordered_map<std::string, int> hw_index_by_name;
      hw_index_by_name.reserve(info_.joints.size());
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        hw_index_by_name.emplace(info_.joints[i].name, static_cast<int>(i));
      }

      auto find_or_throw = [&](const std::string &n) -> int
      {
        auto it = hw_index_by_name.find(n);
        if (it == hw_index_by_name.end())
        {
          throw std::runtime_error("Joint name not found in ros2_control list: " + n);
        }
        return it->second;
      };

      // perm_14_to_hw_: our 14-joint index -> ROS2 control index
      for (int j = 0; j < 7; ++j)
        perm_14_to_hw_[j] = find_or_throw(LEFT_NAMES[j]);
      for (int j = 0; j < 7; ++j)
        perm_14_to_hw_[7 + j] = find_or_throw(RIGHT_NAMES[j]);

      // Inverse: ROS2 control index -> our 14-joint array index
      std::fill(perm_hw_to14_.begin(), perm_hw_to14_.end(), -1);
      for (int i14 = 0; i14 < 14; ++i14)
      {
        const int iHW = perm_14_to_hw_[i14];
        if (iHW < 0 || iHW >= static_cast<int>(info_.joints.size()))
        {
          throw std::runtime_error("Permutation out of HW bounds");
        }
        perm_hw_to14_[iHW] = i14;
      }

      log_perm(perm_14_to_hw_, "perm_14_to_hw");
      log_perm(perm_hw_to14_, "perm_hw_to14");
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("H1ArmPositionHardware"),
          "Failed to build joint-name permutations: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize Unitree communication directly
    try
    {
      RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"),
                  "Initializing Unitree SDK with network interface: %s", network_interface_.c_str());

      // Initialize with specified network interface
      unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_.c_str());

      // Create publisher
      unitree_arm_sdk_publisher_.reset(
          new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSDK));
      unitree_arm_sdk_publisher_->InitChannel();

      // Create subscriber
      unitree_state_subscriber_.reset(
          new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicState));
      unitree_state_subscriber_->InitChannel([this](const void *msg)
                                             {
      auto s = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
      std::lock_guard<std::mutex> lock(state_mutex_);
      memcpy(&unitree_state_, s, sizeof(unitree_state_));
      state_received_ = true; }, 1);

      RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "Unitree communication initialized");
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("H1ArmPositionHardware"),
          "Failed to initialize Unitree communication: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize control parameters from reference code
    weight_ = 0.0f;
    weight_rate_ = 0.2f;
    control_dt_ = 0.02f;
    safe_mode_ = true;
    max_joint_velocity_ = 0.5f; // From original

    // Gains for 14 arm joints only (no waist joint in ROS2 control)
    kp_array_ = {120, 120, 80, 50, 50, 50, 50,       // Left arm (7 joints)
                 120, 120, 80, 50, 50, 50, 50};      // Right arm (7 joints)
    kd_array_ = {2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0,  // Left arm (7 joints)
                 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0}; // Right arm (7 joints)

    // Safe initial positions to avoid collision (adapted from original, no waist)
    safe_init_pos_ = {0.f, 0.3f, 0.f, 0.f, 0.f, 0.f, 0.f,   // Left arm
                      0.f, -0.3f, 0.f, 0.f, 0.f, 0.f, 0.f}; // Right arm

    // Initialize previous commands to 0
    prev_commands_.fill(0.0f);

    RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "H1 Arm Hardware Interface initialized successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  H1ArmPositionHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  H1ArmPositionHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn
  H1ArmPositionHardware::on_activate(const rclcpp_lifecycle::State & /*prev_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "Activating H1 Dual Arm Controller");

    activation_in_progress_ = true;

    // Wait for first state message
    RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "Waiting for first state message...");
    int wait_count = 0;
    while (!state_received_ && wait_count < 100)
    { // Wait up to 2 seconds
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      wait_count++;
    }

    if (!state_received_)
    {
      RCLCPP_WARN(rclcpp::get_logger("H1ArmPositionHardware"), "No state message received, continuing anyway");
      return hardware_interface::CallbackReturn::ERROR; // Fail activation
    }

    // Read current joint positions
    std::array<float, 14> current_positions{};
    if (state_received_)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (int j = 0; j < 14; ++j)
      {
        JointIndex joint_idx = arm_joints[j];
        if (joint_idx < unitree_state_.motor_state().size())
        {
          current_positions[j] = unitree_state_.motor_state().at(joint_idx).q();
          int hw_idx = perm_14_to_hw_[j];
          hw_states_[hw_idx] = static_cast<double>(current_positions[j]);
          hw_commands_[hw_idx] = hw_states_[hw_idx]; // Initialize commands to current positions
        }
      }
    }
    else
    {
      // Fallback: initialize to safe positions
      for (int j = 0; j < 14; ++j)
      {
        current_positions[j] = safe_init_pos_[j];
        int hw_idx = perm_14_to_hw_[j];
        hw_states_[hw_idx] = static_cast<double>(current_positions[j]);
        hw_commands_[hw_idx] = hw_states_[hw_idx];
      }
    }

    // Gradually increase weight to 1.0 while interpolating to safe init positions (to avoid collision)
    RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "Gradually activating control weight and initializing to safe positions...");
    float activation_time = static_cast<float>(hw_start_sec_);
    int activation_steps = static_cast<int>(activation_time / control_dt_);
    float delta_weight = weight_rate_ * control_dt_;

    auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));

    for (int i = 0; i < activation_steps; ++i)
    {
      weight_ += delta_weight;
      weight_ = std::clamp(weight_, 0.0f, 1.0f);

      float phase = static_cast<float>(i) / static_cast<float>(activation_steps - 1);

      // Interpolate from current to safe init positions
      for (int j = 0; j < 14; ++j)
      {
        float interpolated_pos = current_positions[j] * (1.0f - phase) + safe_init_pos_[j] * phase;
        JointIndex joint_idx = arm_joints[j];
        if (joint_idx < unitree_msg_.motor_cmd().size())
        {
          unitree_msg_.motor_cmd().at(joint_idx).q(interpolated_pos);
          unitree_msg_.motor_cmd().at(joint_idx).dq(0.0f);
          unitree_msg_.motor_cmd().at(joint_idx).kp(kp_array_[j]);
          unitree_msg_.motor_cmd().at(joint_idx).kd(kd_array_[j]);
          unitree_msg_.motor_cmd().at(joint_idx).tau(0.0f);
        }
      }

      // Set weight - Note: kNotUsedJoint (27) is used for weight control, not actual joint control
      unitree_msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight_);

      // Send message
      if (unitree_arm_sdk_publisher_)
      {
        unitree_arm_sdk_publisher_->Write(unitree_msg_);
      }

      std::this_thread::sleep_for(sleep_time);
    }

    // Update previous commands to safe init positions
    prev_commands_ = safe_init_pos_;

    safe_mode_ = false;
    activation_in_progress_ = false;
    RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "H1 Dual Arm System activated (weight=%.3f)!", weight_);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  H1ArmPositionHardware::on_deactivate(const rclcpp_lifecycle::State & /*prev_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "Stopping H1 Dual Arm Controller");

    // Execute proper stopping sequence from reference code
    if (unitree_arm_sdk_publisher_)
    {
      RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "Executing gradual weight reduction stop...");

      float stop_time = static_cast<float>(hw_stop_sec_);
      int stop_time_steps = static_cast<int>(stop_time / control_dt_);
      float delta_weight = weight_rate_ * control_dt_;

      auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));

      for (int i = 0; i < stop_time_steps; ++i)
      {
        // Decrease weight gradually (from reference code)
        weight_ -= delta_weight;
        weight_ = std::clamp(weight_, 0.0f, 1.0f);

        // Set weight via kNotUsedJoint
        unitree_msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight_);

        // Send message
        unitree_arm_sdk_publisher_->Write(unitree_msg_);

        // Sleep
        std::this_thread::sleep_for(sleep_time);

        if (i % 25 == 0)
        { // Log every 0.5 seconds
          RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"),
                      "Stop progress: weight=%.3f, steps_left=%d", weight_, stop_time_steps - i);
        }
      }
    }

    safe_mode_ = true;
    RCLCPP_INFO(rclcpp::get_logger("H1ArmPositionHardware"), "H1 Dual Arm System stopped safely!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type
  H1ArmPositionHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!state_received_)
    {
      return hardware_interface::return_type::OK; // Haven't received state yet
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    // Read joint positions from Unitree state (14 arm joints)
    for (size_t iHW = 0; iHW < hw_states_.size(); ++iHW)
    {
      const int i14 = perm_hw_to14_[static_cast<int>(iHW)];
      if (i14 >= 0 && i14 < 14)
      {
        // Map to actual H1 joint indices
        JointIndex joint_idx = arm_joints[i14];
        if (joint_idx < unitree_state_.motor_state().size())
        {
          hw_states_[iHW] = static_cast<double>(unitree_state_.motor_state().at(joint_idx).q());
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type
  H1ArmPositionHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!unitree_arm_sdk_publisher_ || safe_mode_ || activation_in_progress_.load())
    {
      return hardware_interface::return_type::OK; // Don't send commands in safe mode or during activation
    }

    // Set weight (always 1.0 during normal operation)
    // Note: kNotUsedJoint (27) is used for weight control, not actual joint control
    unitree_msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight_);

    // Set motor commands using reference code logic with velocity limiting
    float max_joint_delta = max_joint_velocity_ * control_dt_;
    for (int j = 0; j < 14; ++j)
    {
      JointIndex joint_idx = arm_joints[j];

      if (joint_idx < unitree_msg_.motor_cmd().size())
      {
        // Get command from the corresponding hardware index
        int hw_index = perm_14_to_hw_[j];
        if (hw_index >= 0 && hw_index < static_cast<int>(hw_commands_.size()))
        {
          float target_cmd = static_cast<float>(hw_commands_[hw_index]);
          float limited_cmd = prev_commands_[j] + std::clamp(target_cmd - prev_commands_[j], -max_joint_delta, max_joint_delta);
          prev_commands_[j] = limited_cmd;

          unitree_msg_.motor_cmd().at(joint_idx).q(limited_cmd);
          unitree_msg_.motor_cmd().at(joint_idx).dq(0.0f);
          unitree_msg_.motor_cmd().at(joint_idx).kp(kp_array_[j]);
          unitree_msg_.motor_cmd().at(joint_idx).kd(kd_array_[j]);
          unitree_msg_.motor_cmd().at(joint_idx).tau(0.0f);
        }
      }
    }

    // Send command
    unitree_arm_sdk_publisher_->Write(unitree_msg_);

    return hardware_interface::return_type::OK;
  }

} // namespace h1_platform

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(h1_platform::H1ArmPositionHardware, hardware_interface::SystemInterface)