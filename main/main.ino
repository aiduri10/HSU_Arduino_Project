#include <HX711.h>
#include "HSU_config.h"
#include "HSU_modbus.h"
#include "HSU_calc.h"
#include "HSU_init.h"

HX711 scale1;
HX711 scale2;
ArduinoLEDMatrix matrix;
ModbusMaster Lnode;
ModbusMaster Rnode;
STATE Device = {0, 0, 0, 0, STOP, STRAIGHT};
int state = 0;

void setup()
{
  init();
  Serial.println("Setup complete");
}

void loop()
{
  Device.Lweight = -scale1.get_units() * 1000;
  Device.Rweight = scale2.get_units() * 1000;
  calculateSpeed(&Device);
  calculateVelocity(&Device);
  DirectionImage(&Device, matrix);
  print_log();
      
  switch (state)
  {
    case 0:
      if (getControl(Lnode) == CTRL_WORD_ENABLE_SERVO)
        state = 1;
      else {
        Serial.println("Servo 4 ON");
        setControlEnable(Lnode);
      }
      break;

    case 1:
      if (getControl(Rnode) == CTRL_WORD_ENABLE_SERVO)
        state = 2;
      else {
        Serial.println("Servo 5 ON");
        setControlEnable(Rnode);
      }
      break;

    case 2:
      if (getMode(Lnode) == OPERATING_MODE_VELOCITY)
        state =3;
      else {
        Serial.println("Mode set for Servo 4");
        setMode(Lnode, OPERATING_MODE_VELOCITY);
      }
      break;

    case 3:
      if (getMode(Rnode) == OPERATING_MODE_VELOCITY)
        state = 4;
      else {
        Serial.println("Mode set for Servo 5");
        setMode(Rnode, OPERATING_MODE_VELOCITY);
      }
      break;

    case 4:
      setVelocity(Lnode, -Device.Lvelocity);
      setVelocity(Rnode, Device.Rvelocity);
      break;
  }
}