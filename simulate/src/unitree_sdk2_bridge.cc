#include "unitree_sdk2_bridge.h"

#include <unitree/robot/channel/channel_subscriber.hpp>

UnitreeSDK2BridgeBase::UnitreeSDK2BridgeBase(mjModel* model, mjData* data)
    : mj_model_(model), mj_data_(data) {
  _check_sensor();
  if (param::config.print_scene_information == 1) {
    printSceneInformation();
  }
  if (param::config.use_joystick == 1) {
    if (param::config.joystick_type == "xbox") {
      joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device,
                                                param::config.joystick_bits);
    } else if (param::config.joystick_type == "switch") {
      joystick = std::make_shared<SwitchJoystick>(param::config.joystick_device,
                                                  param::config.joystick_bits);
    } else {
      std::cerr << "Unsupported joystick type: " << param::config.joystick_type
                << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

void UnitreeSDK2BridgeBase::printSceneInformation() const {
  auto printObjects = [this](const char* title, int count, int type,
                             auto getIndex) {
    std::cout << "<<------------- " << title << " ------------->> "
              << std::endl;
    for (int i = 0; i < count; i++) {
      const char* name = mj_id2name(mj_model_, type, i);
      if (name) {
        std::cout << title << "_index: " << getIndex(i) << ", "
                  << "name: " << name;
        if (type == mjOBJ_SENSOR) {
          std::cout << ", dim: " << mj_model_->sensor_dim[i];
        }
        std::cout << std::endl;
      }
    }
    std::cout << std::endl;
  };

  printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
  printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
  printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR,
               [](int i) { return i; });

  int sensorIndex = 0;
  printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
    int currentIndex = sensorIndex;
    sensorIndex += mj_model_->sensor_dim[i];
    return currentIndex;
  });
}

void UnitreeSDK2BridgeBase::_check_sensor() {
  num_motor_ = mj_model_->nu;
  dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;

  // Find sensor addresses by name
  int sensor_id = -1;

  // IMU quaternion
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
  if (sensor_id >= 0) {
    imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
  }

  // IMU gyroscope
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
  if (sensor_id >= 0) {
    imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
  }

  // IMU accelerometer
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
  if (sensor_id >= 0) {
    imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
  }

  // Frame position
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
  if (sensor_id >= 0) {
    frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
  }

  // Frame velocity
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
  if (sensor_id >= 0) {
    frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
  }

  // Secondary IMU quaternion
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
  if (sensor_id >= 0) {
    secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
  }

  // Secondary IMU gyroscope
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
  if (sensor_id >= 0) {
    secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
  }

  // Secondary IMU accelerometer
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
  if (sensor_id >= 0) {
    secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
  }
}

G1Bridge::G1Bridge(mjModel* model, mjData* data) : RobotBridge(model, data) {
  if (param::config.robot.find("g1") != std::string::npos) {
    if (auto* g1_lowstate = lowstate.get()) {
      auto scene = param::config.robot_scene.filename().string();
      g1_lowstate->msg_.mode_machine() =
          scene.find("23") != std::string::npos ? 4 : 5;
    }
  }

  bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
  bmsstate->msg_.soc() = 100;

  secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
}

void G1Bridge::run() {
  RobotBridge::run();

  // secondary IMU state
  if (secondary_imustate->trylock()) {
    if (secondary_imu_quat_adr_ >= 0) {
      secondary_imustate->msg_.quaternion()[0] =
          mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
      secondary_imustate->msg_.quaternion()[1] =
          mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
      secondary_imustate->msg_.quaternion()[2] =
          mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
      secondary_imustate->msg_.quaternion()[3] =
          mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

      double w = secondary_imustate->msg_.quaternion()[0];
      double x = secondary_imustate->msg_.quaternion()[1];
      double y = secondary_imustate->msg_.quaternion()[2];
      double z = secondary_imustate->msg_.quaternion()[3];

      secondary_imustate->msg_.rpy()[0] =
          atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
      secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
      secondary_imustate->msg_.rpy()[2] =
          atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }

    if (secondary_imu_gyro_adr_ >= 0) {
      secondary_imustate->msg_.gyroscope()[0] =
          mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
      secondary_imustate->msg_.gyroscope()[1] =
          mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
      secondary_imustate->msg_.gyroscope()[2] =
          mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
    }

    if (secondary_imu_acc_adr_ >= 0) {
      secondary_imustate->msg_.accelerometer()[0] =
          mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
      secondary_imustate->msg_.accelerometer()[1] =
          mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
      secondary_imustate->msg_.accelerometer()[2] =
          mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
    }

    secondary_imustate->unlockAndPublish();
  }

  // In practice, bmsstate is sent at a low frequency; here it is sent with
  // the main loop
  bmsstate->unlockAndPublish();
}
