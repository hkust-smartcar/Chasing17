/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "assignments/camera_to_lcd.h"

#include "libbase/k60/mcg.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/ov7725.h"

#include "util/util.h"

using libsc::Lcd;
using libsc::LcdConsole;
using libsc::Led;
using libsc::St7735r;
using libsc::System;
using libsc::Timer;
using libsc::k60::Ov7725;

namespace {
constexpr size_t kCameraWidth = 80;
constexpr size_t kCameraHeight = 60;
constexpr size_t kBufferSize = kCameraWidth * kCameraHeight / 8;
}  // namespace

void CameraToLcd() {
  // initialize LEDs
  Led::Config config_led;
  config_led.is_active_low = true;
  config_led.id = 0;
  Led led1(config_led);  // main loop heartbeat
  config_led.id = 1;
  Led led2(config_led);  // unused
  config_led.id = 2;
  Led led3(config_led);  // unused
  config_led.id = 3;
  Led led4(config_led);  // initialization

  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);
  led4.SetEnable(true);

  // initialize camera
  Ov7725::Config config_camera;
  config_camera.id = 0;
  config_camera.w = kCameraWidth;
  config_camera.h = kCameraHeight;
  Ov7725 camera(config_camera);

  // initialize LCD
  St7735r::Config config_lcd;
  config_lcd.fps = 60;
  St7735r lcd(config_lcd);

  // start the camera and wait until it's ready
  camera.Start();
  while (!camera.IsAvailable()) {}

  Timer::TimerInt timeImg = System::Time();  // current execution time

  led4.SetEnable(false);

  // main loop
  while (true) {
    // limit max refresh time to 1ms
    if (timeImg != System::Time()) {
      // update the cycle
      timeImg = System::Time();

      led1.SetEnable(timeImg % 1000 >= 500);

      // attempt to refresh the buffer at every 100th millisecond
      if ((System::Time() % 100) == 0) {
        // lock the buffer and copy it
        const Byte *pBuffer = camera.LockBuffer();
        std::array<Byte, kBufferSize> bufferArr{};
        /*for (uint16_t i = 0; i < kBufferSize; ++i) {
          bufferArr[i] = pBuffer[i];
        }*/
        util::CopyByteArray(*pBuffer, &bufferArr);

        // unlock the buffer now that we have the data
        camera.UnlockBuffer();

        // rewrite lcd with new data
        lcd.SetRegion(Lcd::Rect(0, 0, kCameraWidth, kCameraHeight));
        lcd.FillBits(Lcd::kBlack, Lcd::kWhite, bufferArr.data(), kBufferSize * 8);
      }
    }
  }
}
