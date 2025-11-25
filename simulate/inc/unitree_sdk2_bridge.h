#pragma once

#include <mujoco/mujoco.h>

#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/idl/hg/BmsState_.hpp>

constexpr auto MOTOR_SENSOR_NUM = 3;

class UnitreeSDK2BridgeBase {
public:
  UnitreeSDK2BridgeBase(mjModel* model, mjData* data);

  virtual ~UnitreeSDK2BridgeBase() = default;

  virtual void start() {}

  void printSceneInformation() const;

protected:
  int num_motor_ = 0;
  int dim_motor_sensor_ = 0;

  mjData* mj_data_;
  mjModel* mj_model_;

  // Sensor data indices
  int imu_quat_adr_ = -1;
  int imu_gyro_adr_ = -1;
  int imu_acc_adr_ = -1;
  int frame_pos_adr_ = -1;
  int frame_vel_adr_ = -1;

  int secondary_imu_quat_adr_ = -1;
  int secondary_imu_gyro_adr_ = -1;
  int secondary_imu_acc_adr_ = -1;

  std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

  void _check_sensor();
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase {
  using HighState_t = unitree::robot::go2::publisher::SportModeState;
  using WirelessController_t =
      unitree::robot::go2::publisher::WirelessController;

public:
  RobotBridge(mjModel* model, mjData* data)
      : UnitreeSDK2BridgeBase(model, data) {
    lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
    lowstate = std::make_unique<LowState_t>();
    lowstate->joystick = joystick;
    highstate = std::make_unique<HighState_t>();
    wireless_controller = std::make_unique<WirelessController_t>();
    wireless_controller->joystick = joystick;
  }

  void start() override {
    thread_ = std::make_shared<unitree::common::RecurrentThread>(
        "unitree_bridge", UT_CPU_ID_NONE, 1000, [this] { this->run(); });
  }

  virtual void run() {
    if (!mj_data_)
      return;
    if (lowstate->joystick) {
      lowstate->joystick->update();
    }

    // low cmd
    {
      std::lock_guard<std::mutex> lock(lowcmd->mutex_);
      for (int i(0); i < num_motor_; i++) {
        auto& m = lowcmd->msg_.motor_cmd()[i];
        mj_data_->ctrl[i] =
            m.tau() + m.kp() * (m.q() - mj_data_->sensordata[i]) +
            m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
      }
    }

    // low state
    if (lowstate->trylock()) {
      for (int i(0); i < num_motor_; i++) {
        lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
        lowstate->msg_.motor_state()[i].dq() =
            mj_data_->sensordata[i + num_motor_];
        lowstate->msg_.motor_state()[i].tau_est() =
            mj_data_->sensordata[i + 2 * num_motor_];
      }

      if (imu_quat_adr_ >= 0) {
        lowstate->msg_.imu_state().quaternion()[0] =
            mj_data_->sensordata[imu_quat_adr_ + 0];
        lowstate->msg_.imu_state().quaternion()[1] =
            mj_data_->sensordata[imu_quat_adr_ + 1];
        lowstate->msg_.imu_state().quaternion()[2] =
            mj_data_->sensordata[imu_quat_adr_ + 2];
        lowstate->msg_.imu_state().quaternion()[3] =
            mj_data_->sensordata[imu_quat_adr_ + 3];

        double w = lowstate->msg_.imu_state().quaternion()[0];
        double x = lowstate->msg_.imu_state().quaternion()[1];
        double y = lowstate->msg_.imu_state().quaternion()[2];
        double z = lowstate->msg_.imu_state().quaternion()[3];

        lowstate->msg_.imu_state().rpy()[0] =
            atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
        lowstate->msg_.imu_state().rpy()[2] =
            atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
      }

      if (imu_gyro_adr_ >= 0) {
        lowstate->msg_.imu_state().gyroscope()[0] =
            mj_data_->sensordata[imu_gyro_adr_ + 0];
        lowstate->msg_.imu_state().gyroscope()[1] =
            mj_data_->sensordata[imu_gyro_adr_ + 1];
        lowstate->msg_.imu_state().gyroscope()[2] =
            mj_data_->sensordata[imu_gyro_adr_ + 2];
      }

      if (imu_acc_adr_ >= 0) {
        lowstate->msg_.imu_state().accelerometer()[0] =
            mj_data_->sensordata[imu_acc_adr_ + 0];
        lowstate->msg_.imu_state().accelerometer()[1] =
            mj_data_->sensordata[imu_acc_adr_ + 1];
        lowstate->msg_.imu_state().accelerometer()[2] =
            mj_data_->sensordata[imu_acc_adr_ + 2];
      }

      lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
      lowstate->unlockAndPublish();
    }
    
    // high state
    if (highstate->trylock()) {
      if (frame_pos_adr_ >= 0) {
        highstate->msg_.position()[0] =
            mj_data_->sensordata[frame_pos_adr_ + 0];
        highstate->msg_.position()[1] =
            mj_data_->sensordata[frame_pos_adr_ + 1];
        highstate->msg_.position()[2] =
            mj_data_->sensordata[frame_pos_adr_ + 2];
      }
      if (frame_vel_adr_ >= 0) {
        highstate->msg_.velocity()[0] =
            mj_data_->sensordata[frame_vel_adr_ + 0];
        highstate->msg_.velocity()[1] =
            mj_data_->sensordata[frame_vel_adr_ + 1];
        highstate->msg_.velocity()[2] =
            mj_data_->sensordata[frame_vel_adr_ + 2];
      }
      highstate->unlockAndPublish();
    }

    // wireless_controller
    if (wireless_controller->joystick) {
      wireless_controller->unlockAndPublish();
    }
  }

  std::unique_ptr<HighState_t> highstate;
  std::unique_ptr<WirelessController_t> wireless_controller;
  std::shared_ptr<LowCmd_t> lowcmd;
  std::unique_ptr<LowState_t> lowstate;

private:
  unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd,
                              unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd,
                                    unitree::robot::g1::publisher::LowState> {
public:
  G1Bridge(mjModel* model, mjData* data);

  void run() override;

  using BmsState_t =
      unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
  using IMUState_t =
      unitree::robot::RealTimePublisher<unitree_go::msg::dds_::IMUState_>;
  std::unique_ptr<BmsState_t> bmsstate;
  std::unique_ptr<IMUState_t> secondary_imustate;
};
