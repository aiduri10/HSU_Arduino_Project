#include "LOADCART.h"
#include "LOADCART_loadcell.h"
#include "LOADCART_modbus.h"
#include "LOADCART_state.h"
#include "LOADCART_config.h"
#include <Arduino.h>

void HSU::LoadCart::init()
{
  Serial.begin(115200);
  delay(100);
  loadCell.init();
  delay(100);
  motor.init();
  delay(100);
}

void HSU::LoadCart::stateMachine()
{
  if (current_state == State::IDLE) {
    unsigned long currentTime = millis();

    // 스케줄링 1: 모터 점검 (가장 낮은 우선순위)
    if (currentTime - lastMotorCheckTime >= MOTOR_CHECK_INTERVAL) {
      lastMotorCheckTime = currentTime;
      current_state = State::CHECK_MOTOR_ENABLE;
    }
    
    // 스케줄링 2: 로깅 (중간 우선순위)
    else if (currentTime - lastLogTime >= LOG_INTERVAL) {
      lastLogTime = currentTime;
      current_state = State::MONITOR_STATUS;
    }

    // 스케줄링 3: 센서 및 제어 (가장 높은 우선순위)
    // (다른 작업이 예약되지 않았을 때만 실행)
    else if (currentTime - lastSensorTime >= SENSOR_AND_CONTROL_INTERVAL) {
      lastSensorTime = currentTime;
      current_state = State::UPDATE_LOADCELL;
    }
  }
  switch (current_state) {
    case State::IDLE:
      break;
    case State::CHECK_MOTOR_ENABLE:
      motor.readControl();
      if (motor.getLeftControl() == 0x08 && motor.getRightControl() == 0x08) {
        current_state = State::IDLE;
      }
      else {
        Serial.println("Enabling motors...");
        current_state = State::ENABLE_MOTOR;
      }
      break;

    case State::ENABLE_MOTOR:
      motor.writeControlEnable();
      current_state = State::IDLE;
      break;

    case State::UPDATE_LOADCELL:
      loadCell.readWeight();
      leftWeight = loadCell.getLeftWeight();
      rightWeight = loadCell.getRightWeight();
      current_state = State::CALCULATE_SPEED;
      break;

    case State::CALCULATE_SPEED:
      state.updateState(leftWeight, rightWeight);
      current_state = State::SET_MOTOR_SPEED;
      break;

    case State::SET_MOTOR_SPEED:
      motor.writeVelocity(state.getLeftMotorSpeed(), state.getRightMotorSpeed());
      current_state = State::IDLE;
      break;

    case State::MONITOR_STATUS:
      printLog();
      current_state = State::IDLE;
  }
}

void HSU::LoadCart::printLog()
{
  Serial.print("Left Weight: ");
  Serial.print(leftWeight);
  Serial.print(" g, Right Weight: ");
  Serial.print(rightWeight);
  Serial.print(" g, Left Speed: ");
  Serial.print(state.getLeftMotorSpeed());
  Serial.print(" , Right Speed: ");
  Serial.println(state.getRightMotorSpeed());
  if (motor.getLeftVelocityReadError()) {
    Serial.println("Error reading left motor velocity");
  }
  else {
    Serial.print("Left Motor Velocity: ");
    Serial.println(motor.getLeftVelocity());
  }
  if (motor.getRightVelocityReadError()) {
    Serial.println("Error reading right motor velocity");
  }
  else {
    Serial.print("Right Motor Velocity: ");
    Serial.println(motor.getRightVelocity());
  }
}