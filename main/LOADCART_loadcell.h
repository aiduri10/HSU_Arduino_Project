#ifndef LOADCART_LOADCELL_H
#define LOADCART_LOADCELL_H
#include <HX711.h>
#include "LOADCART_config.h"
namespace HSU {
class LoadCartLoadCell {
public:
    void init();
    void readWeight();
    int getLeftWeight() const { return leftWeight; }
    int getRightWeight() const { return rightWeight; }
private:
    HX711 leftLoadCell;
    HX711 rightLoadCell;
    int leftWeight;
    int rightWeight;
};
}
#endif