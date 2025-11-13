#include "LOADCART_modbus.h"
#include "LOADCART_config.h"
#include <Arduino.h>
#include <ModbusMaster.h>


void HSU::LoadCartMotor::writeControlEnable()
{
  // 모터를 활성화하기 위해 제어 워드 레지스터에 enable 값(0x08)을 씁니다.
  leftMotor.writeSingleRegister(REG_ADDR_CONTROL_WORD, CTRL_WORD_ENABLE_SERVO);
  rightMotor.writeSingleRegister(REG_ADDR_CONTROL_WORD, CTRL_WORD_ENABLE_SERVO);
}

void HSU::LoadCartMotor::writeMode(uint16_t operatingMode)
{
  // 지정된 동작 모드 값을 동작 모드 레지스터에 씁니다.
  leftMotor.writeSingleRegister(REG_ADDR_OPERATING_MODE, operatingMode);
  rightMotor.writeSingleRegister(REG_ADDR_OPERATING_MODE, operatingMode);
}

void HSU::LoadCartMotor::writeVelocity(int16_t leftSpeed, int16_t rightSpeed)
{
  // 목표 속도 값을 해당 레지스터에 씁니다. 이 값은 모터가 도달하고자 하는 속도를 나타냅니다.
  leftMotor.writeSingleRegister(REG_ADDR_TARGET_SPEED, leftSpeed);
  rightMotor.writeSingleRegister(REG_ADDR_TARGET_SPEED, rightSpeed);
}

void HSU::LoadCartMotor::writeAccelerationTime(uint16_t accelTime)
{
  // 가속 시간 값을 해당 레지스터에 씁니다. 이 값은 모터가 최고 속도까지 가속하는 데 걸리는 시간을 결정합니다.
  leftMotor.writeSingleRegister(REG_ADDR_ACCEL_TIME, accelTime);
  rightMotor.writeSingleRegister(REG_ADDR_ACCEL_TIME, accelTime);
}

void HSU::LoadCartMotor::writeDecelerationTime(uint16_t decelTime)
{
  // 감속 시간 값을 해당 레지스터에 씁니다. 이 값은 모터가 정지하는 데 걸리는 시간을 결정합니다.
  leftMotor.writeSingleRegister(REG_ADDR_DECEL_TIME, decelTime);
  rightMotor.writeSingleRegister(REG_ADDR_DECEL_TIME, decelTime);
}


void HSU::LoadCartMotor::readControl()
{
  // 현재 제어 워드 값을 레지스터에서 읽어옵니다.
  uint8_t leftResult = leftMotor.readHoldingRegisters(REG_ADDR_CONTROL_WORD, 1);
  uint8_t rightResult = rightMotor.readHoldingRegisters(REG_ADDR_CONTROL_WORD, 1);

  if (leftResult == leftMotor.ku8MBSuccess) {
    leftControl = leftMotor.getResponseBuffer(0);
    leftControlReadError = false;
  } else {
    leftControlReadError = true;
  }
  if (rightResult == rightMotor.ku8MBSuccess) {
    rightControl = rightMotor.getResponseBuffer(0);
    rightControlReadError = false;
  } else {
    rightControlReadError = true;
  }
}

void HSU::LoadCartMotor::readMode()
{
  uint8_t leftResult = leftMotor.readHoldingRegisters(REG_ADDR_OPERATING_MODE, 1);
  uint8_t rightResult = rightMotor.readHoldingRegisters(REG_ADDR_OPERATING_MODE, 1);

  if (leftResult == leftMotor.ku8MBSuccess) {
    leftMode = leftMotor.getResponseBuffer(0);
    leftModeReadError = false;
  } else {
    leftModeReadError = true;
  }
  if (rightResult == rightMotor.ku8MBSuccess) {
    rightMode = rightMotor.getResponseBuffer(0);
    rightModeReadError = false;
  } else {
    rightModeReadError = true;
  }
}

void HSU::LoadCartMotor::readActualVelocity()
{
  // 실제 속도 레지스터 읽기
  uint8_t leftResult = leftMotor.readHoldingRegisters(REG_ADDR_ACTUAL_SPEED, 1);
  uint8_t rightResult = rightMotor.readHoldingRegisters(REG_ADDR_ACTUAL_SPEED, 1);

  // (수정) 각각의 멤버 변수에 저장
  if (leftResult == leftMotor.ku8MBSuccess) {
    leftActualVelocity = (int16_t)leftMotor.getResponseBuffer(0);
    leftVelocityReadError = false;
  } else {
    leftVelocityReadError = true;
  }
  
  if (rightResult == rightMotor.ku8MBSuccess) {
    rightActualVelocity = (int16_t)rightMotor.getResponseBuffer(0);
    rightVelocityReadError = false;
  } else {
    rightVelocityReadError = true;
  }
}

void HSU::LoadCartMotor::readActualPosition()
{
  // 실제 위치 (High/Low word) 읽기 - 32bit
  uint8_t leftResult = leftMotor.readHoldingRegisters(REG_ADDR_ACTUAL_POSITION_H, 2);
  uint8_t rightResult = rightMotor.readHoldingRegisters(REG_ADDR_ACTUAL_POSITION_H, 2);

  // (수정) 각각의 멤버 변수에 저장
  if (leftResult == leftMotor.ku8MBSuccess) {
    leftActualPosition = ((int32_t)leftMotor.getResponseBuffer(0) << 16) | leftMotor.getResponseBuffer(1);
  }
  if (rightResult == rightMotor.ku8MBSuccess) {
    rightActualPosition = ((int32_t)rightMotor.getResponseBuffer(0) << 16) | rightMotor.getResponseBuffer(1);
  }
}

void HSU::LoadCartMotor::readActualTorque()
{
  // 실제 토크 읽기
  uint8_t leftResult = leftMotor.readHoldingRegisters(REG_ADDR_ACTUAL_TORQUE, 1);
  uint8_t rightResult = rightMotor.readHoldingRegisters(REG_ADDR_ACTUAL_TORQUE, 1);

  // (수정) 각각의 멤버 변수에 저장
  if (leftResult == leftMotor.ku8MBSuccess) {
    leftActualTorque = (int16_t)leftMotor.getResponseBuffer(0);
  }
  if (rightResult == rightMotor.ku8MBSuccess) {
    rightActualTorque = (int16_t)rightMotor.getResponseBuffer(0);
  }
}

void HSU::LoadCartMotor::preTransmission()
{
  digitalWrite(Control, 1);
}

void HSU::LoadCartMotor::postTransmission()
{
  digitalWrite(Control, 0);
  delay(10);
}

void HSU::LoadCartMotor::init()
{
  pinMode(Control,OUTPUT);
  digitalWrite(Control,LOW);

  Serial1.begin(115200);
  
  leftMotor.begin(5, Serial1);
  rightMotor.begin(4, Serial1);

  leftMotor.preTransmission(preTransmission);
  leftMotor.postTransmission(postTransmission);
  rightMotor.preTransmission(preTransmission);
  rightMotor.postTransmission(postTransmission);

  writeDecelerationTime(100);
  writeAccelerationTime(100);

  writeControlEnable();
  writeMode(OPERATING_MODE_VELOCITY);
}
