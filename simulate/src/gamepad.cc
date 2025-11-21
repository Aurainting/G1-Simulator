#include "gamepad.h"

#include <cmath>

void Button::update(const bool state) {
  on_press = state ? state != pressed : false;
  on_release = state ? false : state != pressed;
  pressed = state;
}

void Gamepad::update(const xRockerBtnDataStruct& key_data) {
  lx = lx * (1 - smooth) +
       (std::fabs(key_data.lx) < dead_zone ? 0.0 : key_data.lx) * smooth;
  rx = rx * (1 - smooth) +
       (std::fabs(key_data.rx) < dead_zone ? 0.0 : key_data.rx) * smooth;
  ry = ry * (1 - smooth) +
       (std::fabs(key_data.ry) < dead_zone ? 0.0 : key_data.ry) * smooth;
  l2 = l2 * (1 - smooth) +
       (std::fabs(key_data.L2) < dead_zone ? 0.0 : key_data.L2) * smooth;
  ly = ly * (1 - smooth) +
       (std::fabs(key_data.ly) < dead_zone ? 0.0 : key_data.ly) * smooth;

  R1.update(key_data.btn.components.R1);
  L1.update(key_data.btn.components.L1);
  start.update(key_data.btn.components.start);
  select.update(key_data.btn.components.select);
  R2.update(key_data.btn.components.R2);
  L2.update(key_data.btn.components.L2);
  F1.update(key_data.btn.components.F1);
  F2.update(key_data.btn.components.F2);
  A.update(key_data.btn.components.A);
  B.update(key_data.btn.components.B);
  X.update(key_data.btn.components.X);
  Y.update(key_data.btn.components.Y);
  up.update(key_data.btn.components.up);
  right.update(key_data.btn.components.right);
  down.update(key_data.btn.components.down);
  left.update(key_data.btn.components.left);
}
