/*
  H1 Dual Arm Recorder - Industrial Safe ROS2 Service Controller
  Records and plays back dual 7-DoF arm trajectories
*/
#include <iomanip>
#include <cstring>
#include <chrono>
#include <thread>
#include <array>
#include <vector>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "h1_interface/srv/h1_modes.hpp"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";

// H1 Joint indices
enum JointIndex
{
    kLeftHipYaw = 0,
    kLeftHipPitch = 1,
    kLeftHipRoll = 2,
    kLeftKnee = 3,
    kLeftAnkle = 4,
    kLeftAnkleRoll = 5,
    kRightHipYaw = 6,
    kRightHipPitch = 7,
    kRightHipRoll = 8,
    kRightKnee = 9,
    kRightAnkle = 10,
    kRightAnkleRoll = 11,
    kWaistYaw = 12,
    kLeftShoulderPitch = 13,
    kLeftShoulderRoll = 14,
    kLeftShoulderYaw = 15,
    kLeftElbow = 16,
    kLeftWristRoll = 17,
    kLeftWristPitch = 18,
    kLeftWristYaw = 19,
    kRightShoulderPitch = 20,
    kRightShoulderRoll = 21,
    kRightShoulderYaw = 22,
    kRightElbow = 23,
    kRightWristRoll = 24,
    kRightWristPitch = 25,
    kRightWristYaw = 26,
    kNotUsedJoint = 27
};

struct TrajectoryPoint
{
    double timestamp;
    std::array<double, 14> positions;
};

class H1DualArmRecorder : public rclcpp::Node
{
public:
    explicit H1DualArmRecorder() : rclcpp::Node("h1_dual_arm_recorder")
    {
        // Parameters
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
            memcpy(&current_state_, state, sizeof(current_state_));

            // CHANGE: Arm and initialize positions ONLY on the first received LowState
            if (!armed_.load()) {
                std::lock_guard<std::mutex> lockp(position_mutex_);
                for (size_t i = 0; i < 14; ++i) {
                    int joint_idx = arm_joints_[i];
                    if (joint_idx < current_state_.motor_state().size()) {
                        const double q = current_state_.motor_state().at(joint_idx).q();
                        current_positions_[i] = q;
                        target_positions_[i]  = q;
                    }
                }
                armed_.store(true);
                RCLCPP_INFO(get_logger(), "\033[32mFirst LowState received; recorder armed from hardware pose\033[0m");
            }
            
            // Record if active
            if (recording_.load()) {
                recordCurrentPosition();
            } }, 1);

        // Services
        record_service_ = create_service<std_srvs::srv::SetBool>(
            "dual_arm/record",
            std::bind(&H1DualArmRecorder::handleRecord, this, std::placeholders::_1, std::placeholders::_2));

        playback_service_ = create_service<h1_interface::srv::H1Modes>(
            "dual_arm/playback",
            std::bind(&H1DualArmRecorder::handlePlayback, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize positions and create save directory
        std::filesystem::create_directories(save_directory_);

        // Control timer for smooth playback
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000)),
            std::bind(&H1DualArmRecorder::controlLoop, this));

        RCLCPP_INFO_STREAM(
            get_logger(),
            "\033[32mH1 Dual Arm Recorder initialized (per-joint gains)\033[0m\n"
            "\033[36mRecord Rate: \033[33m"
                << (1.0 / control_dt_) << " Hz\033[0m\n"
                                          "\033[36mSave Directory: \033[33m"
                << save_directory_ << "\033[0m\n"
                                      "\033[36mNetwork Interface: \033[33m"
                << eth_interface_ << "\033[0m\n"
                                     "\033[36mMax Joint Velocity: \033[33m"
                << max_joint_velocity_ << " rad/s\033[0m\n"
                                          "\033[36mWeight: \033[33m"
                << weight_ << "\033[0m");
    }

private:
    // Joint indices for both arms (14 joints total)
    static constexpr std::array<int, 14> arm_joints_ = {
        kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
        kLeftWristRoll, kLeftWristPitch, kLeftWristYaw,
        kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
        kRightWristRoll, kRightWristPitch, kRightWristYaw};

    void declareParameters()
    {
        eth_interface_ = declare_parameter<std::string>("eth_interface", "eth0");
        control_dt_ = declare_parameter<double>("control_dt", 0.02);
        max_joint_velocity_ = declare_parameter<double>("max_joint_velocity", 0.3);

        // Playback gains - per joint
        auto kp_vec = declare_parameter<std::vector<double>>("kp", std::vector<double>(14, 80.0));
        auto kd_vec = declare_parameter<std::vector<double>>("kd", std::vector<double>(14, 1.5));
        std::copy_n(kp_vec.begin(), 14, kp_.begin());
        std::copy_n(kd_vec.begin(), 14, kd_.begin());

        // Idle gains - per joint
        auto idle_kp_vec = declare_parameter<std::vector<double>>("idle_kp", std::vector<double>(14, 30.0));
        auto idle_kd_vec = declare_parameter<std::vector<double>>("idle_kd", std::vector<double>(14, 2.0));
        std::copy_n(idle_kp_vec.begin(), 14, idle_kp_.begin());
        std::copy_n(idle_kd_vec.begin(), 14, idle_kd_.begin());

        // Record gains - per joint
        auto record_kp_vec = declare_parameter<std::vector<double>>("record_kp", std::vector<double>(14, 0.1));
        auto record_kd_vec = declare_parameter<std::vector<double>>("record_kd", std::vector<double>(14, 0.1));
        std::copy_n(record_kp_vec.begin(), 14, record_kp_.begin());
        std::copy_n(record_kd_vec.begin(), 14, record_kd_.begin());

        weight_ = declare_parameter<double>("weight", 0.8);
        save_directory_ = declare_parameter<std::string>("save_directory", "./h1_trajectories");

        // Safety limits
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

    void handleRecord(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (playing_.load())
        {
            response->success = false;
            response->message = "Cannot record while playing trajectory";
            return;
        }

        if (request->data)
        {
            // Start recording
            {
                std::lock_guard<std::mutex> lock(trajectory_mutex_);
                recorded_trajectory_.clear();
                recording_start_time_ = std::chrono::steady_clock::now();
            }
            recording_.store(true);

            response->success = true;
            response->message = "Recording started";
            RCLCPP_INFO(get_logger(), "\033[32mRecording started\033[0m");
        }
        else
        {
            // Stop recording and save
            recording_.store(false);

            std::string filename = saveTrajectory();
            if (!filename.empty())
            {
                last_recorded_file_ = filename;
                response->success = true;
                response->message = "Recording stopped, saved to: " + filename;
                RCLCPP_INFO(get_logger(), "\033[32mRecording stopped, saved: %s\033[0m", filename.c_str());
            }
            else
            {
                response->success = false;
                response->message = "Recording stopped but failed to save";
                RCLCPP_ERROR(get_logger(), "Failed to save recording");
            }
        }
    }

    void handlePlayback(
        const std::shared_ptr<h1_interface::srv::H1Modes::Request> request,
        std::shared_ptr<h1_interface::srv::H1Modes::Response> response)
    {
        if (recording_.load())
        {
            response->success = false;
            response->reason = "Cannot playback while recording";
            return;
        }

        if (playing_.load())
        {
            response->success = false;
            response->reason = "Already playing a trajectory";
            return;
        }

        std::string filename;
        if (request->request_data.empty())
        {
            // Play last recorded
            filename = last_recorded_file_;
            if (filename.empty())
            {
                response->success = false;
                response->reason = "No recorded trajectory available";
                return;
            }
        }
        else
        {
            // Play specified file
            filename = request->request_data;
            if (filename.size() < 4 || filename.compare(filename.size() - 4, 4, ".csv") != 0)
            {
                filename += ".csv";
            }
        }

        // Load and validate trajectory
        std::vector<TrajectoryPoint> trajectory;
        if (!loadTrajectory(filename, trajectory))
        {
            response->success = false;
            response->reason = "Failed to load trajectory file: " + filename;
            return;
        }

        // Start playback in separate thread
        std::thread([this, trajectory, filename]()
                    { playTrajectory(trajectory, filename); })
            .detach();

        response->success = true;
        response->reason = "Started playback of: " + filename;
        RCLCPP_INFO(get_logger(), "\033[32mStarted playback: %s\033[0m", filename.c_str());
    }

    void recordCurrentPosition()
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);

        TrajectoryPoint point;
        auto now = std::chrono::steady_clock::now();
        point.timestamp = std::chrono::duration<double>(now - recording_start_time_).count();

        for (size_t i = 0; i < 14; ++i)
        {
            int joint_idx = arm_joints_[i];
            if (joint_idx < current_state_.motor_state().size())
            {
                point.positions[i] = current_state_.motor_state().at(joint_idx).q();
                current_positions_[i] = point.positions[i]; // Update current_positions_ during recording
            }
        }

        recorded_trajectory_.push_back(point);
    }

    std::string saveTrajectory()
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);

        if (recorded_trajectory_.empty())
        {
            return "";
        }

        // Generate filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << save_directory_ << "/trajectory_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
        std::string filename = ss.str();

        // Save to CSV
        std::ofstream file(filename);
        if (!file.is_open())
        {
            return "";
        }

        // Header
        file << "timestamp";
        const std::vector<std::string> joint_names = {
            "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
            "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
            "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
            "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"};

        for (const auto &name : joint_names)
        {
            file << "," << name;
        }
        file << "\n";

        // Data
        for (const auto &point : recorded_trajectory_)
        {
            file << point.timestamp;
            for (size_t i = 0; i < 14; ++i)
            {
                file << "," << point.positions[i];
            }
            file << "\n";
        }

        file.close();
        return filename;
    }

    bool loadTrajectory(const std::string &filename, std::vector<TrajectoryPoint> &trajectory)
    {
        std::string full_path = filename;
        if (filename.empty() || filename.front() != '/')
        {
            full_path = save_directory_ + "/" + filename;
        }

        std::ifstream file(full_path);
        if (!file.is_open())
        {
            return false;
        }

        trajectory.clear();
        std::string line;

        // Skip header
        if (!std::getline(file, line))
        {
            return false;
        }

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            TrajectoryPoint point;

            // Parse timestamp
            if (!std::getline(ss, cell, ','))
                break;
            point.timestamp = std::stod(cell);

            // Parse joint positions
            for (size_t i = 0; i < 14; ++i)
            {
                if (!std::getline(ss, cell, ','))
                {
                    return false;
                }
                point.positions[i] = std::clamp(std::stod(cell), pos_limits_min_[i], pos_limits_max_[i]);
            }

            trajectory.push_back(point);
        }

        return !trajectory.empty();
    }

    void playTrajectory(const std::vector<TrajectoryPoint> &trajectory, const std::string &filename)
    {
        playing_.store(true);

        RCLCPP_INFO(get_logger(), "Playing trajectory with %zu points", trajectory.size());

        // Step 1: Smoothly transition to the first trajectory point using velocity limiting
        RCLCPP_INFO(get_logger(), "Blending to trajectory start position...");
        {
            std::array<double, 14> q0, qf = trajectory[0].positions;
            {
                std::lock_guard<std::mutex> lock(position_mutex_);
                q0 = current_positions_;
            }

            double max_delta = 0.0;
            for (size_t i = 0; i < 14; ++i)
                max_delta = std::max(max_delta, std::abs(qf[i] - q0[i]));

            if (max_delta > 1e-3)
            {
                // Set target positions immediately
                {
                    std::lock_guard<std::mutex> lock(position_mutex_);
                    for (size_t i = 0; i < 14; ++i)
                    {
                        target_positions_[i] = qf[i];
                    }
                }

                // Wait until current_positions_ converges to target_positions_ within tolerance
                const double tolerance = 0.001;   // radians
                const double max_duration = 10.0; // Safety timeout (seconds)
                auto start_time = std::chrono::steady_clock::now();

                while (rclcpp::ok() && playing_.load())
                {
                    double max_error = 0.0;
                    {
                        std::lock_guard<std::mutex> lock(position_mutex_);
                        for (size_t i = 0; i < 14; ++i)
                        {
                            max_error = std::max(max_error, std::abs(target_positions_[i] - current_positions_[i]));
                        }
                    }

                    if (max_error < tolerance)
                    {
                        std::lock_guard<std::mutex> lock(position_mutex_);
                        for (size_t i = 0; i < 14; ++i)
                        {
                            current_positions_[i] = qf[i]; // Ensure exact final position
                            target_positions_[i] = qf[i];
                        }
                        break;
                    }

                    double elapsed = std::chrono::duration<double>(
                                         std::chrono::steady_clock::now() - start_time)
                                         .count();
                    if (elapsed > max_duration)
                    {
                        RCLCPP_WARN(get_logger(), "Timeout reaching start position, proceeding anyway...");
                        std::lock_guard<std::mutex> lock(position_mutex_);
                        for (size_t i = 0; i < 14; ++i)
                        {
                            current_positions_[i] = qf[i]; // Force final position
                            target_positions_[i] = qf[i];
                        }
                        break;
                    }

                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000)));
                }
            }
            else
            {
                // If already close, set positions directly
                std::lock_guard<std::mutex> lock(position_mutex_);
                for (size_t i = 0; i < 14; ++i)
                {
                    current_positions_[i] = qf[i];
                    target_positions_[i] = qf[i];
                }
            }
        }
        RCLCPP_INFO(get_logger(), "Reached start position, beginning trajectory playback...");

        // Step 2: Play through EVERY point sequentially
        // Directly set current_positions_ to bypass velocity limiting
        for (size_t idx = 0; idx < trajectory.size() && rclcpp::ok(); ++idx)
        {
            const auto &point = trajectory[idx];

            // Calculate time until this point should be reached
            double time_until_point = 0.0;
            if (idx > 0)
            {
                time_until_point = point.timestamp - trajectory[idx - 1].timestamp;
            }

            // CRITICAL: Set BOTH current_positions_ and target_positions_
            // This bypasses the velocity limiter in controlLoop()
            {
                std::lock_guard<std::mutex> lock(position_mutex_);
                for (size_t i = 0; i < 14; ++i)
                {
                    current_positions_[i] = point.positions[i];
                    target_positions_[i] = point.positions[i];
                }
            }

            // Wait for the recorded time delta (matches recording rate)
            if (time_until_point > 0.0)
            {
                auto sleep_duration = std::chrono::duration<double>(time_until_point);
                std::this_thread::sleep_for(sleep_duration);
            }

            if (idx % 50 == 0) // Progress logging every 50 points
            {
                RCLCPP_INFO(get_logger(), "Playing point %zu/%zu (t=%.3f)",
                            idx, trajectory.size(), point.timestamp);
            }
        }

        // Hold at final position for 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        playing_.store(false);
        RCLCPP_INFO(get_logger(), "\033[32mPlayback completed: %s\033[0m", filename.c_str());
    }

    void controlLoop()
    {
        if (!armed_.load())
        {
            return;
        }

        unitree_hg::msg::dds_::LowCmd_ cmd;

        // Update current positions toward target (velocity limited) during playback
        if (playing_.load())
        {
            std::lock_guard<std::mutex> lock(position_mutex_);
            double dt_vel_limit = max_joint_velocity_ * control_dt_;

            for (size_t i = 0; i < 14; ++i)
            {
                double error = target_positions_[i] - current_positions_[i];
                double step = std::clamp(error, -dt_vel_limit, dt_vel_limit);
                current_positions_[i] += step;
            }
        }

        // Send motor commands
        for (size_t i = 0; i < 14; ++i)
        {
            int joint_idx = arm_joints_[i];
            if (joint_idx < cmd.motor_cmd().size())
            {
                std::lock_guard<std::mutex> lock(position_mutex_);
                cmd.motor_cmd().at(joint_idx).q(static_cast<float>(current_positions_[i]));
                cmd.motor_cmd().at(joint_idx).dq(0.0f);

                // Use low stiffness during recording, normal during playback
                if (recording_.load())
                {
                    cmd.motor_cmd().at(joint_idx).kp(static_cast<float>(record_kp_[i]));
                    cmd.motor_cmd().at(joint_idx).kd(static_cast<float>(record_kd_[i]));
                }
                else if (playing_.load())
                {
                    cmd.motor_cmd().at(joint_idx).kp(static_cast<float>(kp_[i]));
                    cmd.motor_cmd().at(joint_idx).kd(static_cast<float>(kd_[i]));
                }
                else
                {
                    // Idle state
                    cmd.motor_cmd().at(joint_idx).kp(static_cast<float>(idle_kp_[i]));
                    cmd.motor_cmd().at(joint_idx).kd(static_cast<float>(idle_kd_[i]));
                }

                cmd.motor_cmd().at(joint_idx).tau(0.0f);
            }
        }

        // Set weight
        if (kNotUsedJoint < cmd.motor_cmd().size())
        {
            cmd.motor_cmd().at(kNotUsedJoint).q(static_cast<float>(weight_));
        }

        arm_publisher_->Write(cmd);
    }

    // Parameters
    std::string eth_interface_;
    std::string save_directory_;
    double control_dt_;
    double max_joint_velocity_;
    std::array<double, 14> kp_, kd_;
    std::array<double, 14> idle_kp_, idle_kd_;
    std::array<double, 14> record_kp_, record_kd_;
    double weight_;
    std::vector<double> pos_limits_min_, pos_limits_max_;

    // State
    std::mutex state_mutex_, position_mutex_, trajectory_mutex_;
    unitree_hg::msg::dds_::LowState_ current_state_;
    std::array<double, 14> current_positions_{}, target_positions_{};

    // Recording/Playback
    std::atomic<bool> armed_{false};
    std::atomic<bool> recording_{false};
    std::atomic<bool> playing_{false};
    std::vector<TrajectoryPoint> recorded_trajectory_;
    std::chrono::steady_clock::time_point recording_start_time_;
    std::string last_recorded_file_;

    // ROS components
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> arm_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> state_subscriber_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_service_;
    rclcpp::Service<h1_interface::srv::H1Modes>::SharedPtr playback_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        return -1;
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<H1DualArmRecorder>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}