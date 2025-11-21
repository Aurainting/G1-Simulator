#pragma once

#include <cstdint>

// bytecode mapping for raw joystick data
// 16b
typedef union {
  struct {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

// 40 Byte (now used 24B)
typedef struct {
  uint8_t head[2];
  xKeySwitchUnion btn;
  float lx;
  float rx;
  float ry;
  float L2;
  float ly;

  uint8_t idle[16];
} xRockerBtnDataStruct;

typedef union {
  xRockerBtnDataStruct RF_RX;
  uint8_t buff[40];
} REMOTE_DATA_RX;

class Button {
public:
  Button() = default;

  void update(bool state);

  bool pressed = false;
  bool on_press = false;
  bool on_release = false;
};

class Gamepad {
public:
  Gamepad() = default;

  void update(const xRockerBtnDataStruct& key_data);

  float lx = 0.;
  float rx = 0.;
  float ry = 0.;
  float l2 = 0.;
  float ly = 0.;

  float smooth = 0.03;
  float dead_zone = 0.01;

  Button R1;
  Button L1;
  Button start;
  Button select;
  Button R2;
  Button L2;
  Button F1;
  Button F2;
  Button A;
  Button B;
  Button X;
  Button Y;
  Button up;
  Button right;
  Button down;
  Button left;
};
