#ifndef LOADCART_H
#define LOADCART_H
#include "LOADCART_loadcell.h"
#include "LOADCART_modbus.h"
#include "LOADCART_state.h"
namespace HSU {
class LoadCart {
public:
    void init();
    void update();
    void printLog();
private:
    LoadCartLoadCell loadCell;
    LoadCartMotor motor;
    LoadCartState state;
    int leftWeight;
    int rightWeight;
};
}
#endif
