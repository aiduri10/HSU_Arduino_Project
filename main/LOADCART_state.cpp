#include "LoadCart_state.h"
#include <cmath>
#include <Arduino.h>
#include <ModbusMaster.h>
#include <HX711.h>
#include "LoadCart_config.h"

const char* HSU::SPEED_LEVEL_STRINGS[] = {
  "무속", "저속", "중속", "고속"
};
const char* HSU::TURNING_LEVEL_STRINGS[] = {
  "직진", "좌회전", "우회전"
};
const char* HSU::MOVEMENT_LEVEL_STRINGS[] = {
  "정지", "전진", "후진"
};


void HSU::LoadCartState::calculateSpeedLevel(void)
{
  int absLeftWeight = std::fabs(leftWeight);
  int absRightWeight = std::fabs(rightWeight);
  // 절대값이 500이면 무속도로 설정
  if (absLeftWeight < 500 || absRightWeight < 500) {
    speedLevel = SpeedLevel::None;
  } else {
    int avgAbsWeight = (absLeftWeight + absRightWeight) / 2;
    // 절대값의 평균값이 500 ~ 2000이면 저속도로 설정
    if (avgAbsWeight >= 500 && avgAbsWeight < 2000) {
      speedLevel = SpeedLevel::Low;
    // 절대값의 평균값이 2000 ~ 4000이면 중속도로 설정
    } else if (avgAbsWeight >= 2000 && avgAbsWeight < 4000) {
      speedLevel = SpeedLevel::Medium;
    // 절대값의 평균값이 4000 이상이면 고속도로 설정
    } else if (avgAbsWeight >= 4000) {
      speedLevel = SpeedLevel::High;
    }
  }
}

void HSU::LoadCartState::calculateTurningLevel(void)
{
  if (speedLevel == SpeedLevel::None) {
    turningLevel = TurningLevel::STRAIGHT;
    return;
  }
  int diff = rightWeight - leftWeight;
  if (diff > 500) {
    turningLevel = TurningLevel::LEFT;
  } else if (diff < -500) {
    turningLevel = TurningLevel::RIGHT;
  } else {
    turningLevel = TurningLevel::STRAIGHT;
  }
}

void HSU::LoadCartState::calculateMovementLevel(void)
{
  // 안전장치: 양쪽 무게중 하나가 500 미만이면 정지 상태로 설정
  if (leftWeight >= 500 && rightWeight >= 500) {
    movementLevel = MovementLevel::FORWARD;
  } else if (leftWeight <= -500 && rightWeight <= -500) {
    movementLevel = MovementLevel::BACKWARD;
  } else {
    movementLevel = MovementLevel::STOPPED;    
  }
}

void HSU::LoadCartState::calculateMotorSpeed(void)
{
  if (movementLevel == MovementLevel::STOPPED || speedLevel == SpeedLevel::None) {
    leftSpeed = 0;
    rightSpeed = 0;
    return;
    }

  // 2. 기본 속도 설정
switch (speedLevel) {
  case SpeedLevel::Low:
    leftSpeed = BASE_LOW_SPEED;
    rightSpeed = BASE_LOW_SPEED;
    break;
  case SpeedLevel::Medium:
    leftSpeed = BASE_MEDIUM_SPEED;
    rightSpeed = BASE_MEDIUM_SPEED;
    break;
  case SpeedLevel::High:
    leftSpeed = BASE_HIGH_SPEED;
    rightSpeed = BASE_HIGH_SPEED;
    break;
  default:
    break;
  }

  if (turningLevel == TurningLevel::LEFT) {
    rightSpeed += TURNING_ADJUSTMENT;
    leftSpeed -= TURNING_ADJUSTMENT;
  } else if (turningLevel == TurningLevel::RIGHT) {
    leftSpeed += TURNING_ADJUSTMENT;
    rightSpeed -= TURNING_ADJUSTMENT;
  }

  if (movementLevel == MovementLevel::BACKWARD) {
    uint16_t temp;
    temp = -leftSpeed;
    leftSpeed = -rightSpeed;
    rightSpeed = temp;
  }
}

void HSU::LoadCartState::updateState(int leftWeight, int rightWeight)
{
  this->leftWeight = leftWeight;
  this->rightWeight = rightWeight;
  calculateSpeedLevel();
  calculateTurningLevel();
  calculateMovementLevel();
  calculateMotorSpeed();
}
int16_t HSU::LoadCartState::getLeftMotorSpeed(void)
{
  // 왼쪽 모터 속도는 반대 방향이므로 음수로 반환
  return this->leftSpeed * -1;
}

int16_t HSU::LoadCartState::getRightMotorSpeed(void)
{
  return this->rightSpeed;
}
void HSU::LoadCartState::printState(void)
{
  const char* speedStr = SPEED_LEVEL_STRINGS[static_cast<int>(speedLevel)];
  const char* turningStr = TURNING_LEVEL_STRINGS[static_cast<int>(turningLevel)];
  const char* movementStr = MOVEMENT_LEVEL_STRINGS[static_cast<int>(movementLevel)];

  Serial.print("Speed: ");
  Serial.print(speedStr);

  Serial.print(" | Turning: ");
  Serial.print(turningStr);

  Serial.print(" | Movement: ");
  Serial.println(movementStr);

  Serial.print("Left Motor Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | ");
  Serial.print("Right Motor Speed: ");
  Serial.println(rightSpeed);
  Serial.println("--------------------");
}