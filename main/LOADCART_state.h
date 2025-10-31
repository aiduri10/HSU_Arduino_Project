#ifndef LOADCART_STATE_H
#define LOADCART_STATE_H
#include <stdint.h>
#include <ModbusMaster.h>
#include <HX711.h>
namespace HSU {
// 속도의 단계를 정의한다. 
enum class SpeedLevel {
  None,
  Low,
  Medium,
  High
};
// 회전 방향을 정의한다.
enum class TurningLevel {
  STRAIGHT,
  LEFT,
  RIGHT
};
// 전후 방향 상태를 정의한다.
enum class MovementLevel {
  STOPPED,
  FORWARD,
  BACKWARD
};
// 속도, 회전 방향, 전후 방향의 문자열을 정의한다.
extern const char* SPEED_LEVEL_STRINGS[];
extern const char* TURNING_LEVEL_STRINGS[];
extern const char* MOVEMENT_LEVEL_STRINGS[];

class LoadCartState {
public:
  void updateState(int leftWeight, int rightWeight);
  void printState(void);
  int16_t getLeftMotorSpeed(void);
  int16_t getRightMotorSpeed(void);
private:

  int leftWeight;
  int rightWeight;

  int16_t leftSpeed;
  int16_t rightSpeed;

  SpeedLevel speedLevel;
  TurningLevel turningLevel;
  MovementLevel movementLevel;

  void calculateMotorSpeed(void);
  void calculateSpeedLevel(void);
  void calculateTurningLevel(void);
  void calculateMovementLevel(void);
};
}
#endif