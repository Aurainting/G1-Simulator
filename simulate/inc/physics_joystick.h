#pragma once

#include <memory>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

#include "joystick.h"

class XBoxJoystick : public unitree::common::UnitreeJoystick {
public:
  explicit XBoxJoystick(const std::string& device, int bits = 15);

  void update() override;

private:
  std::unique_ptr<Joystick> js_;
  int max_value_;
};

class SwitchJoystick : public unitree::common::UnitreeJoystick {
public:
  explicit SwitchJoystick(const std::string& device, int bits = 15);

  void update() override;

private:
  std::unique_ptr<Joystick> js_;
  int max_value_;
};
