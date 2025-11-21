#include "physics_joystick.h"

XBoxJoystick::XBoxJoystick(const std::string& device, const int bits)
    : UnitreeJoystick() {
  js_ = std::make_unique<Joystick>(device);
  if (!js_->isFound()) {
    std::cout << "Error: Joystick open failed." << std::endl;
    exit(1);
  }
  max_value_ = 1 << (bits - 1);
}

void XBoxJoystick::update() {
  js_->getState();
  back(js_->button_[6]);
  start(js_->button_[7]);
  LB(js_->button_[4]);
  RB(js_->button_[5]);
  A(js_->button_[0]);
  B(js_->button_[1]);
  X(js_->button_[2]);
  Y(js_->button_[3]);
  up(js_->axis_[7] < 0);
  down(js_->axis_[7] > 0);
  left(js_->axis_[6] < 0);
  right(js_->axis_[6] > 0);
  LT(js_->axis_[2] > 0);
  RT(js_->axis_[5] > 0);
  lx(double(js_->axis_[0]) / max_value_);
  ly(-double(js_->axis_[1]) / max_value_);
  rx(double(js_->axis_[3]) / max_value_);
  ry(-double(js_->axis_[4]) / max_value_);
}

SwitchJoystick::SwitchJoystick(const std::string& device, const int bits)
    : UnitreeJoystick() {
  js_ = std::make_unique<Joystick>(device);
  if (!js_->isFound()) {
    std::cout << "Error: Joystick open failed." << std::endl;
    exit(1);
  }
  max_value_ = 1 << (bits - 1);
}

void SwitchJoystick::update() {
  js_->getState();
  back(js_->button_[10]);
  start(js_->button_[11]);
  LB(js_->button_[6]);
  RB(js_->button_[7]);
  A(js_->button_[0]);
  B(js_->button_[1]);
  X(js_->button_[3]);
  Y(js_->button_[4]);
  up(js_->axis_[7] < 0);
  down(js_->axis_[7] > 0);
  left(js_->axis_[6] < 0);
  right(js_->axis_[6] > 0);
  LT(js_->axis_[5] > 0);
  RT(js_->axis_[4] > 0);
  lx(double(js_->axis_[0]) / max_value_);
  ly(-double(js_->axis_[1]) / max_value_);
  rx(double(js_->axis_[2]) / max_value_);
  ry(-double(js_->axis_[3]) / max_value_);
}
