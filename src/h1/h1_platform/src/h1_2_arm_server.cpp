/*
  H1 Dual Arm Controller - Industrial Safe ROS2 Action Server
  Minimal implementation for dual 7-DoF arm control
*/
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <array>
#include <vector>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJT>;

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";
constexpr float kPi_2 = 1.57079632f;

// H1 Joint indices
enum JointIndex
{
  // Left leg (0-5)
  kLeftHipYaw = 0,
  kLeftHipPitch = 1,
  kLeftHipRoll = 2,
  kLeftKnee = 3,
  kLeftAnkle = 4,
  kLeftAnkleRoll = 5,
  // Right leg (6-11)
  kRightHipYaw = 6,
  kRightHipPitch = 7,
  kRightHipRoll = 8,
  kRightKnee = 9,
  kRightAnkle = 10,
  kRightAnkleRoll = 11,
  // Waist
  kWaistYaw = 12,
  // Left arm (13-19)
  kLeftShoulderPitch = 13,
  kLeftShoulderRoll = 14,
  kLeftShoulderYaw = 15,
  kLeftElbow = 16,
  kLeftWristRoll = 17,
  kLeftWristPitch = 18,
  kLeftWristYaw = 19,
  // Right arm (20-26)
  kRightShoulderPitch = 20,
  kRightShoulderRoll = 21,
  kRightShoulderYaw = 22,
  kRightElbow = 23,
  kRightWristRoll = 24,
  kRightWristPitch = 25,
  kRightWristYaw = 26,
  // Reserved
  kNotUsedJoint = 27
};

class H1DualArmController : public rclcpp::Node
{
public:
  explicit H1DualArmController() : rclcpp::Node("h1_dual_arm_controller")
  {
    // Parameters with colored output
    declareParameters();

    // Initialize Unitree DDS
    unitree::robot::ChannelFactory::Instance()->Init(0, eth_interface_.c_str());

    // Setup publishers/subscribers
    arm_publisher_.reset(new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSDK));
    arm_publisher_->InitChannel();

    state_subscriber_.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicState));
    state_subscriber_->InitChannel([this](const void *msg)
                                   {
      const auto* state = (const unitree_hg::msg::dds_::LowState_*)msg;
      std::lock_guard<std::mutex> lock(state_mutex_);
      memcpy(&current_state_, state, sizeof(current_state_)); }, 1);

    // Action server for dual arm trajectory following
    action_server_ = rclcpp_action::create_server<FollowJT>(
        this, "dual_arm/follow_joint_trajectory",
        std::bind(&H1DualArmController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&H1DualArmController::handleCancel, this, std::placeholders::_1),
        std::bind(&H1DualArmController::handleAccept, this, std::placeholders::_1));

    // Initialize current positions
    initializePositions();

    // Control timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000)),
        std::bind(&H1DualArmController::controlLoop, this));

    RCLCPP_INFO_STREAM(
        get_logger(),
        "\033[32mH1 Dual Arm Controller initialized\033[0m\n"
        "\033[36mControl Rate: \033[33m"
            << (1.0 / control_dt_) << " Hz\033[0m\n"
                                      "\033[36mNetwork Interface: \033[33m"
            << eth_interface_ << "\033[0m\n"
                                 "\033[36mMax Joint Velocity: \033[33m"
            << max_joint_velocity_ << " rad/s\033[0m\n"
                                      "\033[36mWeight: \033[33m"
            << weight_ << "\033[0m\n"
                          "\033[36mPosition Limits (min): \033[33m"
            << vecToStr(pos_limits_min_) << "\033[0m\n"
                                            "\033[36mPosition Limits (max): \033[33m"
            << vecToStr(pos_limits_max_) << "\033[0m");
  }

private:
  // Convert vector to string
  static std::string vecToStr(const std::vector<double> &v)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "[";
    for (size_t i = 0; i < v.size(); ++i)
    {
      if (i)
        oss << ", ";
      oss << v[i];
    }
    oss << "]";
    return oss.str();
  }

  // Joint indices for both arms (14 joints total)
  static constexpr std::array<int, 14> arm_joints_ = {
      kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
      kLeftWristRoll, kLeftWristPitch, kLeftWristYaw,
      kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
      kRightWristRoll, kRightWristPitch, kRightWristYaw};

  // Expected joint names in order
  const std::vector<std::string> expected_joint_names_ = {
      "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
      "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
      "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
      "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"};

  void declareParameters()
  {
    eth_interface_ = declare_parameter<std::string>("eth_interface", "eth0");
    control_dt_ = declare_parameter<double>("control_dt", 0.02);
    max_joint_velocity_ = declare_parameter<double>("max_joint_velocity", 0.5);

    // Per-joint gains
    auto kp_vec = declare_parameter<std::vector<double>>("kp", std::vector<double>(14, 120.0));
    auto kd_vec = declare_parameter<std::vector<double>>("kd", std::vector<double>(14, 2.0));
    std::copy_n(kp_vec.begin(), 14, kp_.begin());
    std::copy_n(kd_vec.begin(), 14, kd_.begin());

    weight_ = declare_parameter<double>("weight", 1.0);

    // Safety position limits (radians)
    pos_limits_min_ = declare_parameter<std::vector<double>>("pos_limits_min",
                                                             std::vector<double>(14, -M_PI));
    pos_limits_max_ = declare_parameter<std::vector<double>>("pos_limits_max",
                                                             std::vector<double>(14, M_PI));
  }

  void initializePositions()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < 14; ++i)
    {
      int joint_idx = arm_joints_[i];
      if (joint_idx < current_state_.motor_state().size())
      {
        current_positions_[i] = current_state_.motor_state().at(joint_idx).q();
        target_positions_[i] = current_positions_[i];
      }
    }
    armed_.store(true);
  }

  // Action server callbacks
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const FollowJT::Goal> goal)
  {

    if (!armed_.load())
    {
      RCLCPP_WARN(get_logger(), "Controller not armed");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Validate joint names and trajectory
    if (goal->trajectory.joint_names.size() != 14)
    {
      RCLCPP_ERROR(get_logger(), "Expected 14 joints, got %zu", goal->trajectory.joint_names.size());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->trajectory.points.empty())
    {
      RCLCPP_ERROR(get_logger(), "Empty trajectory");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Check joint name order
    for (size_t i = 0; i < 14; ++i)
    {
      if (goal->trajectory.joint_names[i] != expected_joint_names_[i])
      {
        RCLCPP_ERROR(get_logger(), "Joint name mismatch at index %zu: expected '%s', got '%s'",
                     i, expected_joint_names_[i].c_str(), goal->trajectory.joint_names[i].c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccept(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Execute in separate thread
    std::thread{[this, goal_handle]()
                { executeTrajectory(goal_handle); }}
        .detach();
  }

  void executeTrajectory(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowJT::Feedback>();
    auto result = std::make_shared<FollowJT::Result>();

    cancel_requested_.store(false);
    executing_.store(true);

    RCLCPP_INFO(get_logger(), "Executing trajectory with %zu points", goal->trajectory.points.size());

    // Execute each trajectory point
    for (size_t i = 0; i < goal->trajectory.points.size(); ++i)
    {
      if (goal_handle->is_canceling() || cancel_requested_.load())
      {
        RCLCPP_INFO(get_logger(), "Trajectory cancelled");
        executing_.store(false);
        goal_handle->canceled(result);
        return;
      }

      const auto &point = goal->trajectory.points[i];

      if (point.positions.size() != 14)
      {
        RCLCPP_ERROR(get_logger(), "Invalid position count at point %zu", i);
        executing_.store(false);
        goal_handle->abort(result);
        return;
      }

      // Set target positions with safety limits
      {
        std::lock_guard<std::mutex> lock(position_mutex_);
        for (size_t j = 0; j < 14; ++j)
        {
          target_positions_[j] = std::clamp(point.positions[j],
                                            pos_limits_min_[j], pos_limits_max_[j]);
        }
      }

      // Calculate trajectory duration for this segment
      double duration = (i == 0) ? (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) : ((point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) - (goal->trajectory.points[i - 1].time_from_start.sec + goal->trajectory.points[i - 1].time_from_start.nanosec * 1e-9));

      // Wait for trajectory segment completion
      auto start_time = std::chrono::steady_clock::now();
      rclcpp::Rate rate(50); // 50Hz feedback

      while (rclcpp::ok())
      {
        if (goal_handle->is_canceling() || cancel_requested_.load())
        {
          executing_.store(false);
          goal_handle->canceled(result);
          return;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        double elapsed_sec = std::chrono::duration<double>(elapsed).count();

        // Publish feedback
        feedback->joint_names = expected_joint_names_;
        {
          std::lock_guard<std::mutex> lock(position_mutex_);
          feedback->actual.positions.assign(current_positions_.begin(), current_positions_.end());
          feedback->desired.positions.assign(target_positions_.begin(), target_positions_.end());
          feedback->error.positions.resize(14);
          for (size_t j = 0; j < 14; ++j)
          {
            feedback->error.positions[j] = target_positions_[j] - current_positions_[j];
          }
        }
        goal_handle->publish_feedback(feedback);

        if (elapsed_sec >= duration)
          break;
        rate.sleep();
      }
    }

    executing_.store(false);
    RCLCPP_INFO(get_logger(), "Trajectory execution completed");
    goal_handle->succeed(result);
  }

  void controlLoop()
  {
    if (!armed_.load())
      return;

    unitree_hg::msg::dds_::LowCmd_ cmd;

    // Update current positions from desired (velocity limited)
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      double dt_vel_limit = max_joint_velocity_ * control_dt_;

      for (size_t i = 0; i < 14; ++i)
      {
        double error = target_positions_[i] - current_positions_[i];
        double step = std::clamp(error, -dt_vel_limit, dt_vel_limit);
        current_positions_[i] += step;

        // Apply position limits
        current_positions_[i] = std::clamp(current_positions_[i],
                                           pos_limits_min_[i], pos_limits_max_[i]);
      }
    }

    // Set motor commands
    for (size_t i = 0; i < 14; ++i)
    {
      int joint_idx = arm_joints_[i];
      if (joint_idx < cmd.motor_cmd().size())
      {
        std::lock_guard<std::mutex> lock(position_mutex_);
        cmd.motor_cmd().at(joint_idx).q(static_cast<float>(current_positions_[i]));
        cmd.motor_cmd().at(joint_idx).dq(0.0f);
        cmd.motor_cmd().at(joint_idx).kp(static_cast<float>(kp_[i]));
        cmd.motor_cmd().at(joint_idx).kd(static_cast<float>(kd_[i]));
        cmd.motor_cmd().at(joint_idx).tau(0.0f);
      }
    }

    // Set weight (control enable signal)
    if (kNotUsedJoint < cmd.motor_cmd().size())
    {
      cmd.motor_cmd().at(kNotUsedJoint).q(static_cast<float>(weight_));
    }

    arm_publisher_->Write(cmd);
  }

  // Parameters
  std::string eth_interface_;
  double control_dt_;
  double max_joint_velocity_;
  std::array<double, 14> kp_, kd_;
  double weight_;
  std::vector<double> pos_limits_min_, pos_limits_max_;

  // State
  std::mutex state_mutex_, position_mutex_;
  unitree_hg::msg::dds_::LowState_ current_state_;
  std::array<double, 14> current_positions_{}, target_positions_{};

  // Control flags
  std::atomic<bool> armed_{false};
  std::atomic<bool> executing_{false};
  std::atomic<bool> cancel_requested_{false};

  // ROS components
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> arm_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> state_subscriber_;
  rclcpp_action::Server<FollowJT>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<H1DualArmController>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}