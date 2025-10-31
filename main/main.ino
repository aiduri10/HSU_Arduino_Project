#include "LOADCART.h"
HSU::LoadCart loadCart;
void setup()
{
  loadCart.init();
}

int state = 0;
void loop()
{
  loadCart.update();
  if (state % 10 == 0) {
    loadCart.printLog();
    state = 0;
  }
  state++;
  delay(100);
}