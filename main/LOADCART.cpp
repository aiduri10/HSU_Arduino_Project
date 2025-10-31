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

void HSU::LoadCart::update()
{
  loadCell.readWeight();
  leftWeight = loadCell.getLeftWeight();
  rightWeight = loadCell.getRightWeight();
  state.updateState(leftWeight, rightWeight);
  motor.writeVelocity(state.getLeftMotorSpeed(), state.getRightMotorSpeed());
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