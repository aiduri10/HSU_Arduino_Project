#include "LOADCART_loadcell.h"
#include <HX711.h>
#include "LOADCART_config.h"

void HSU::LoadCartLoadCell::init()
{
  leftLoadCell.begin(HSU::LEFT_LOADCELL_DOUT_PIN, HSU::LEFT_LOADCELL_SCK_PIN);
  leftLoadCell.tare();
  leftLoadCell.set_scale(HSU::leftCalibrationFactor);

  rightLoadCell.begin(HSU::RIGHT_LOADCELL_DOUT_PIN, HSU::RIGHT_LOADCELL_SCK_PIN);
  rightLoadCell.tare();
  rightLoadCell.set_scale(HSU::rightCalibrationFactor);
}

void HSU::LoadCartLoadCell::readWeight()
{
  if (leftLoadCell.is_ready()) {
    leftWeight = -leftLoadCell.get_units() * 1000;
  }
  if (rightLoadCell.is_ready()) {
    rightWeight = rightLoadCell.get_units() * 1000;
  }
}