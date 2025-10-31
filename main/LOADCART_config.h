#ifndef LOADCART_CONFIG_H
#define LOADCART_CONFIG_H
#include <stdint.h>
namespace HSU {
  const int Control = 4;
  const int LEFT_LOADCELL_DOUT_PIN = 2;
  const int LEFT_LOADCELL_SCK_PIN = 3;
  const int RIGHT_LOADCELL_DOUT_PIN = 8;
  const int RIGHT_LOADCELL_SCK_PIN = 9;

  const float leftCalibrationFactor = 420000; 
  const float rightCalibrationFactor = 420000;

  // 모터 속도 기본값 및 조정값 정의 CartState.cpp에서 사용
  const int BASE_LOW_SPEED = 50;
  const int BASE_MEDIUM_SPEED = 70;
  const int BASE_HIGH_SPEED = 90;
  const int TURNING_ADJUSTMENT = 20;
}

#endif