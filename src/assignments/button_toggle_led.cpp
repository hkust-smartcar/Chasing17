/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "assignments/button_toggle_led.h"

#include "libsc/button.h"
#include "libsc/led.h"

using libsc::Button;
using libsc::Led;

// Pointer to led1, for access to both the listener and the main function
namespace {
Led *pled1;

/**
 * Listener for button 1
 * @param id ID of the button
 */
void B1Listener(const uint8_t id) {
  pled1->Switch();
}
}  // namespace

void ButtonToggleLed() {
  Led::Config config_led;
  config_led.id = 0;
  config_led.is_active_low = true;
  Led led1(config_led);

  pled1 = &led1;

  Button::Config config_button;
  config_button.id = 0;
  config_button.is_active_low = true;
  config_button.is_use_pull_resistor = true;
  config_button.listener = &B1Listener;
  config_button.listener_trigger = Button::Config::Trigger::kBoth;
  Button btn1(config_button);

  while (true) {}
}
