#include "LOADCART.h"
HSU::LoadCart loadCart;
void setup()
{
  loadCart.init();
}

void loop()
{
  loadCart.stateMachine();  
}