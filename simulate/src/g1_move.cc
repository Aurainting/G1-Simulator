#include "g1_move.h"

#include <chrono>
#include <cmath>
#include <thread>

G1Move::G1Move(const std::string& networkInterface)
    : time_(0.0), control_dt_(0.002), duration_(3.0), counter_(0),
      mode_pr_(Mode::PR), mode_machine_(0), rx_{} {
  ChannelFactory::Instance()->Init(0, networkInterface);

  // try to shut down motion control-related service
  msc_ = std::make_shared<b2::MotionSwitcherClient>();
  msc_->SetTimeout(5.0f);
  msc_->Init();
  std::string form, name;
  while (msc_->CheckMode(form, name), !name.empty()) {
    if (msc_->ReleaseMode())
      std::cout << "Failed to switch to Release Mode\n";
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  // create publisher
  low_cmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
  low_cmd_publisher_->InitChannel();

  // create subscriber
  low_state_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
  low_state_subscriber_->InitChannel(
      std::bind(&G1Move::LowStateHandler, this, std::placeholders::_1), 1);

  imu_torso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
  imu_torso_subscriber_->InitChannel(
      std::bind(&G1Move::imuTorsoHandler, this, std::placeholders::_1), 1);

  // create threads
  command_writer_ptr_ = CreateRecurrentThreadEx(
      "command_writer", UT_CPU_ID_NONE, 2000, &G1Move::LowCommandWriter, this);
  control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000,
                                                &G1Move::Control, this);
}

void G1Move::Run() {
  std::cout << "Begin run G1 Move..." << std::endl;
  G1Move instance{};
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
}

void G1Move::LowStateHandler(const void* message) {
  LowState_ low_state = *static_cast<const LowState_*>(message);
  if (low_state.crc() != Crc32Core(reinterpret_cast<uint32_t*>(&low_state),
                                   (sizeof(LowState_) >> 2) - 1)) {
    std::cout << "[ERROR] CRC Error" << std::endl;
    return;
  }

  // get motor state
  MotorState ms_tmp;
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    ms_tmp.q.at(i) = low_state.motor_state()[i].q();
    ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
    if (low_state.motor_state()[i].motorstate() &&
        i <= static_cast<int>(G1JointIndex::RightAnkleRoll))
      std::cout << "[ERROR] motor " << i << " with code "
                << low_state.motor_state()[i].motorstate() << "\n";
  }
  motor_state_buffer_.SetData(ms_tmp);

  // get imu state
  ImuState imu_tmp;
  imu_tmp.omega = low_state.imu_state().gyroscope();
  imu_tmp.rpy = low_state.imu_state().rpy();
  imu_state_buffer_.SetData(imu_tmp);

  // update gamepad
  memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
  gamepad_.update(rx_.RF_RX);

  // update mode machine
  if (mode_machine_ != low_state.mode_machine()) {
    if (mode_machine_ == 0)
      std::cout << "G1 type: " << unsigned(low_state.mode_machine())
                << std::endl;
    mode_machine_ = low_state.mode_machine();
  }

  // report robot status every second
  if (++counter_ % 500 == 0) {
    counter_ = 0;
    // IMU
    auto& rpy = low_state.imu_state().rpy();
    printf("IMU.pelvis.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);

    // RC
    printf("gamepad_.A.pressed: %d\n", static_cast<int>(gamepad_.A.pressed));
    printf("gamepad_.B.pressed: %d\n", static_cast<int>(gamepad_.B.pressed));
    printf("gamepad_.X.pressed: %d\n", static_cast<int>(gamepad_.X.pressed));
    printf("gamepad_.Y.pressed: %d\n", static_cast<int>(gamepad_.Y.pressed));

    // Motor
    auto& ms = low_state.motor_state();
    printf("All %d Motors:", G1_NUM_MOTOR);
    printf("\nmode: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%u,", ms[i].mode());
    printf("\npos: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%.2f,", ms[i].q());
    printf("\nvel: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%.2f,", ms[i].dq());
    printf("\ntau_est: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%.2f,", ms[i].tau_est());
    printf("\ntemperature: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%d,%d;", ms[i].temperature()[0], ms[i].temperature()[1]);
    printf("\nvol: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%.2f,", ms[i].vol());
    printf("\nsensor: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%u,%u;", ms[i].sensor()[0], ms[i].sensor()[1]);
    printf("\nmotorstate: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%u,", ms[i].motorstate());
    printf("\nreserve: ");
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
      printf("%u,%u,%u,%u;", ms[i].reserve()[0], ms[i].reserve()[1],
             ms[i].reserve()[2], ms[i].reserve()[3]);
    printf("\n");
  }
}

void G1Move::imuTorsoHandler(const void* message) const {
  IMUState_ imu_torso = *static_cast<const IMUState_*>(message);
  auto& rpy = imu_torso.rpy();
  if (counter_ % 500 == 0)
    printf("IMU.torso.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);
}

void G1Move::LowCommandWriter() {
  LowCmd_ dds_low_command;
  dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
  dds_low_command.mode_machine() = mode_machine_;

  const std::shared_ptr<const MotorCommand> mc =
      motor_command_buffer_.GetData();
  if (mc) {
    for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
      dds_low_command.motor_cmd().at(i).mode() = 1; // 1:Enable, 0:Disable
      dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
      dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
      dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
      dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
      dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
    }

    dds_low_command.crc() = Crc32Core((uint32_t*)&dds_low_command,
                                      (sizeof(dds_low_command) >> 2) - 1);
    low_cmd_publisher_->Write(dds_low_command);
  }
}

void G1Move::Control() {
  MotorCommand motor_command_tmp;
  const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    motor_command_tmp.tau_ff.at(i) = 0.0;
    motor_command_tmp.q_target.at(i) = 0.0;
    motor_command_tmp.dq_target.at(i) = 0.0;
    motor_command_tmp.kp.at(i) = Kp[i];
    motor_command_tmp.kd.at(i) = Kd[i];
  }

  if (ms) {
    time_ += control_dt_;
    if (time_ < duration_) {
      // [Stage 1]: set robot to zero posture
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
        motor_command_tmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
      }
    } else if (time_ < duration_ * 2) {
      // [Stage 2]: swing ankle using PR mode
      mode_pr_ = Mode::PR;
      double max_P = M_PI * 30.0 / 180.0;
      double max_R = M_PI * 10.0 / 180.0;
      double t = time_ - duration_;
      double L_P_des = max_P * std::sin(2.0 * M_PI * t);
      double L_R_des = max_R * std::sin(2.0 * M_PI * t);
      double R_P_des = max_P * std::sin(2.0 * M_PI * t);
      double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

      motor_command_tmp.q_target.at(
          static_cast<int>(G1JointIndex::LeftAnklePitch)) = L_P_des;
      motor_command_tmp.q_target.at(
          static_cast<int>(G1JointIndex::LeftAnkleRoll)) = L_R_des;
      motor_command_tmp.q_target.at(
          static_cast<int>(G1JointIndex::RightAnklePitch)) = R_P_des;
      motor_command_tmp.q_target.at(
          static_cast<int>(G1JointIndex::RightAnkleRoll)) = R_R_des;
    } else {
      // [Stage 3]: swing ankle using AB mode
      mode_pr_ = Mode::AB;
      double max_A = M_PI * 30.0 / 180.0;
      double max_B = M_PI * 10.0 / 180.0;
      double t = time_ - duration_ * 2;
      double L_A_des = +max_A * std::sin(M_PI * t);
      double L_B_des = +max_B * std::sin(M_PI * t + M_PI);
      double R_A_des = -max_A * std::sin(M_PI * t);
      double R_B_des = -max_B * std::sin(M_PI * t + M_PI);

      motor_command_tmp.q_target.at(
          static_cast<int>(G1JointIndex::LeftAnkleA)) = L_A_des;
      motor_command_tmp.q_target.at(
          static_cast<int>(G1JointIndex::LeftAnkleB)) = L_B_des;
      motor_command_tmp.q_target.at(
          static_cast<int>(G1JointIndex::RightAnkleA)) = R_A_des;
      motor_command_tmp.q_target.at(
          static_cast<int>(G1JointIndex::RightAnkleB)) = R_B_des;
    }

    motor_command_buffer_.SetData(motor_command_tmp);
  }
}

uint32_t G1Move::Crc32Core(const uint32_t* ptr, const uint32_t len) {
  uint32_t x_bit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  for (uint32_t i = 0; i < len; i++) {
    x_bit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      constexpr uint32_t dwPolynomial = 0x04c11db7;
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & x_bit)
        CRC32 ^= dwPolynomial;

      x_bit >>= 1;
    }
  }
  return CRC32;
}
