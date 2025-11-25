#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>

#include "gamepad.h"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

template <typename T> class DataBuffer {
public:
  void SetData(const T& newData) {
    std::unique_lock lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock lock(mutex);
    return data ? data : nullptr;
  }

  void Clear() {
    std::unique_lock lock(mutex);
    data = nullptr;
  }

private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

constexpr int G1_NUM_MOTOR = 29;

struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};
struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};
struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

enum class Mode : char {
  PR = 0, // Series Control for Pitch/Roll Joints
  AB = 1  // Parallel Control for A/B Joints
};

enum class G1JointIndex : int {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13,  // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,     // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14, // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,     // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20, // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,   // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27, // NOTE INVALID for g1 23dof
  RightWristYaw = 28    // NOTE INVALID for g1 23dof
};

class G1Move {
public:
  explicit G1Move(const std::string& networkInterface = "lo");

  static void Run();

  void LowStateHandler(const void* message);

  void imuTorsoHandler(const void* message) const;

  void LowCommandWriter();

  void Control();

private:
  double time_;
  double control_dt_; // [2ms]
  double duration_;   // [3 s]
  int counter_;
  Mode mode_pr_;
  uint8_t mode_machine_;

  Gamepad gamepad_;
  REMOTE_DATA_RX rx_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  ChannelPublisherPtr<LowCmd_> low_cmd_publisher_;
  ChannelSubscriberPtr<LowState_> low_state_subscriber_;
  ChannelSubscriberPtr<IMUState_> imu_torso_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_;

  std::shared_ptr<b2::MotionSwitcherClient> msc_;

  // Stiffness for all G1 Joints
  std::array<float, G1_NUM_MOTOR> Kp{
      60, 60, 60, 100, 40, 40,     // legs
      60, 60, 60, 100, 40, 40,     // legs
      60, 40, 40,                  // waist
      40, 40, 40, 40,  40, 40, 40, // arms
      40, 40, 40, 40,  40, 40, 40  // arms
  };

  // Damping for all G1 Joints
  std::array<float, G1_NUM_MOTOR> Kd{
      1, 1, 1, 2, 1, 1,    // legs
      1, 1, 1, 2, 1, 1,    // legs
      1, 1, 1,             // waist
      1, 1, 1, 1, 1, 1, 1, // arms
      1, 1, 1, 1, 1, 1, 1  // arms
  };

  static uint32_t Crc32Core(const uint32_t* ptr, uint32_t len);
};
