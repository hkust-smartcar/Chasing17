/*
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: David Mak (Derppening)
 */

#include "algorithm/centerline_method.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <string>

#include "libsc/alternate_motor.h"
#include "libsc/futaba_s3010.h"
#include "libsc/lcd.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/tower_pro_mg995.h"
#include "libsc/k60/ov7725.h"

#include "tuning_constants.h"
#include "util/encoder_proportional_controller.h"
#include "util/util.h"

using libsc::AlternateMotor;
using libsc::FutabaS3010;
using libsc::Lcd;
using libsc::LcdConsole;
using libsc::Led;
using libsc::St7735r;
using libsc::System;
using libsc::Timer;
using libsc::TowerProMg995;
using libsc::k60::Ov7725;
using std::array;
using std::bitset;
using std::string;
using std::unique_ptr;
using util::ByteTo2DBitArray;
using util::CopyByteArray;
using util::EncoderPController;
using util::EncoderPControllerDebug;
using util::FindElement;
using util::MedianFilter;

namespace {
struct {
  Uint track_count;    // Number of pixels of the track
  Uint left_bound;    // Left boundary of the track
  Uint right_bound;   // Right boundary of the track
  Uint center_point;  // Index of the center point
} RowInfo[tuning::kCameraHeight];

void CommitParameters(AlternateMotor *motor, FutabaS3010 *servo, uint16_t *steer_value, int32_t *target_degree,
                      uint8_t valid_count) {
  using namespace tuning;

  // set the target degree
  if (*steer_value == 0) {
    // no valid data - use previous data
    *target_degree = servo->GetDegree();
  } else if (valid_count < kCameraMinSrcConfidence) {
    // insufficient data - use previous data
    *target_degree = servo->GetDegree();
  } else if (*steer_value < (kCameraWidth * 9 / 20) || *steer_value > (kCameraWidth * 11 / 20)) {
    // edge 90% for either side - steep steering
    *target_degree = kServoCenter - ((*steer_value - (kCameraWidth / 2)) * ServoFactor::kFactorHigh);
  } else if (*steer_value < (kCameraWidth * 8 / 20) || *steer_value > (kCameraHeight * 12 / 20)) {
    // middle 10% for either side - medium steering
    *target_degree = kServoCenter - ((*steer_value - (kCameraWidth / 2)) * ServoFactor::kFactorMid);
  } else {
    // center 10% for either side - adjustment
    *target_degree = kServoCenter - ((*steer_value - (kCameraWidth / 2)) * ServoFactor::kFactorLow);
  }

  // commit motor and servo changes
  if (*target_degree > kServoLeftBound) {
    // value > left bound: higher speed for compensation
    *target_degree = kServoLeftBound;
    motor->SetPower(MotorSpeed::kSpeedMid);
  } else if (*target_degree < kServoRightBound) {
    // value < right bound: higher speed for compensation
    *target_degree = kServoRightBound;
    motor->SetPower(MotorSpeed::kSpeedMid);
  } else if (*target_degree < (kServoCenter + kServoRightBound) / 2) {
    // value < half of right steer: slower speed to decrease turning radius
    motor->SetPower(MotorSpeed::kSpeedLow);
  } else if (*target_degree > (kServoCenter + kServoLeftBound) / 2) {
    // value > half of left steer: slower speed to decrease turning radius
    motor->SetPower(MotorSpeed::kSpeedLow);
  } else {
    motor->SetPower(MotorSpeed::kSpeedMid);
  }
  servo->SetDegree(static_cast<uint16_t>(*target_degree));
}
}  // namespace

void CenterLineMethod() {
  using namespace tuning;

  // initialize LEDs
  Led::Config led_config;
  led_config.is_active_low = true;
  led_config.id = 0;
  Led led1(led_config);  // main loop heartbeat
  led_config.id = 1;
  Led led2(led_config);  // buffer processing led
  led_config.id = 2;
  Led led3(led_config);  // unused
  led_config.id = 3;
  Led led4(led_config);  // initialization led
  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);
  led4.SetEnable(true);

  // initialize camera
  Ov7725::Config camera_config;
  camera_config.id = 0;
  camera_config.w = kCameraWidth;
  camera_config.h = kCameraHeight;
  unique_ptr<Ov7725> camera(new Ov7725(camera_config));
  if (kBufferSize != camera->GetBufferSize()) {
    return;
  }
  camera->Start();
  while (!camera->IsAvailable()) {}

  // initialize servo
  FutabaS3010::Config servo_config;
  servo_config.id = 0;
  FutabaS3010 servo(servo_config);
  servo.SetDegree(kServoCenter);

  // initialize motors
  AlternateMotor::Config altmotor_config;
  altmotor_config.id = 0;
  AlternateMotor motor(altmotor_config);
  motor.SetClockwise(false);
  motor.SetPower(0);

  // initialize LCD
  St7735r::Config lcd_config;
  lcd_config.fps = 10;
  unique_ptr<St7735r> lcd(new St7735r(lcd_config));
  lcd->Clear();

  Timer::TimerInt time_img = System::Time();  // current execution time
  uint16_t steer_value = kCameraWidth / 2;
  int32_t target_degree = kServoCenter;

  led4.SetEnable(false);

  while (true) {
    if (time_img != System::Time()) {
      time_img = System::Time();

      // heartbeat
      led1.SetEnable(time_img % 1000 >= 500);

      // copy the buffer for processing
      const Byte *pbuffer = camera->LockBuffer();
      array<Byte, kBufferSize> image1d{};
      CopyByteArray(*pbuffer, &image1d);
      camera->UnlockBuffer();

      led2.SetEnable(true);

      // 1d to 2d array
      array<array<bool, kCameraWidth>, kCameraHeight> image2d;
      ByteTo2DBitArray(image1d, &image2d);

      // apply median filter
      array<array<bool, kCameraWidth>, kCameraHeight> image2d_median;
      MedianFilter(image2d, &image2d_median);

      // fill in row information
      for (int rowIndex = kCameraHeight - 1; rowIndex >= 0; --rowIndex) {
        if (rowIndex == kCameraHeight - 1) {  // row closest to car
          if (!image2d_median.at(rowIndex).at(kCameraWidth / 2)) {  // middle is track - do normal processing
            RowInfo[rowIndex].right_bound = static_cast<Uint>(
                FindElement(image2d_median.at(rowIndex), kCameraWidth / 2, kCameraWidth - 1, true, true));
            RowInfo[rowIndex].left_bound = static_cast<Uint>(
                FindElement(image2d_median.at(rowIndex), kCameraWidth / 2, 0, true, true));
          } else {  // middle is not track! - alternate processing
            // take the previous steer_value, and assume the track will be at that side as well
            if (steer_value > kCameraWidth / 2) {  // assumes track at right side
              RowInfo[rowIndex].left_bound = static_cast<Uint>(
                  FindElement(image2d_median.at(rowIndex), kCameraWidth / 2, kCameraWidth - 1, false, true));
              RowInfo[rowIndex].right_bound = static_cast<Uint>(
                  FindElement(image2d_median.at(rowIndex), RowInfo[rowIndex].left_bound, kCameraWidth - 1, true, true));
            } else if (steer_value < kCameraWidth / 2) {  // assumes track at left side
              RowInfo[rowIndex].right_bound =
                  static_cast<Uint>(FindElement(image2d_median.at(rowIndex), kCameraWidth / 2, 0, false, true));
              RowInfo[rowIndex].left_bound = static_cast<Uint>(
                  FindElement(image2d_median.at(rowIndex), RowInfo[rowIndex].right_bound, 0, true, true));
            }
          }
        } else {  // all other rows - dependent on the closest row
          RowInfo[rowIndex].right_bound = static_cast<Uint>(
              FindElement(image2d_median.at(rowIndex),
                          RowInfo[rowIndex + 1].center_point,
                          kCameraWidth - 1,
                          true,
                          true));
          RowInfo[rowIndex].left_bound = static_cast<Uint>(
              FindElement(image2d_median.at(rowIndex), RowInfo[rowIndex + 1].center_point, 0, true, true));
        }
        // calculate the dependencies
        RowInfo[rowIndex].center_point = (RowInfo[rowIndex].left_bound + RowInfo[rowIndex].right_bound) / 2;
        RowInfo[rowIndex].track_count = RowInfo[rowIndex].right_bound - RowInfo[rowIndex].left_bound;
      }

      led2.SetEnable(false);

      // render center line on lcd
      if (kEnableLcd) {
        lcd->SetRegion(Lcd::Rect(0, 0, kCameraWidth, kCameraHeight));
        lcd->FillBits(Lcd::kBlack, Lcd::kWhite, image1d.data(), kBufferSize * 8);
        for (Uint i = kCameraHeight - 1; i > kCameraMaxSrcHeight; --i) {
          if (RowInfo[i].track_count < kCameraMinPixelCount) {
            break;
          }
          if (!image2d_median[i][RowInfo[i].center_point] && i >= 60) {
            lcd->SetRegion(Lcd::Rect(RowInfo[i].center_point, i, 1, 1));
            lcd->FillColor(Lcd::kRed);
          } else if (!image2d_median[i][RowInfo[i].center_point] && i >= 40) {
            lcd->SetRegion(Lcd::Rect(RowInfo[i].center_point, i, 1, 1));
            lcd->FillColor(Lcd::kYellow);
          }
        }
        lcd->SetRegion(Lcd::Rect(steer_value, 79, 1, 40));
        lcd->FillColor(Lcd::kBlack);
      }

      // average the center points to construct an average direction to move in
      uint8_t valid_count = 0;
      for (Uint i = kCameraHeight - 1; i > kCameraMaxSrcHeight; --i) {
        if (RowInfo[i].track_count < kCameraMinPixelCount) {
          // leave the loop if row is not part of the track anymore
          break;
        }
        if (!image2d_median[i][RowInfo[i].center_point]) {
          steer_value += RowInfo[i].center_point;
          ++valid_count;
        }
      }
      steer_value /= valid_count;

      // render steer_value and wipe target_degree
      if (kEnableLcd) {
        lcd->SetRegion(Lcd::Rect(steer_value, kCameraHeight - 1, 1, 40));
        lcd->FillColor(Lcd::kWhite);
        lcd->SetRegion(Lcd::Rect(static_cast<Uint>(64 - (target_degree / 28)), 119, 1, 40));
        lcd->FillColor(Lcd::kBlack);
      }

      CommitParameters(&motor, &servo, &steer_value, &target_degree, valid_count);

      // render target_degree relative to the image
      if (kEnableLcd) {
        lcd->SetRegion(Lcd::Rect(static_cast<Uint>(64 - (target_degree / 28)), 119, 1, 40));
        lcd->FillColor(Lcd::kWhite);
      }
    }
  }
}

void CenterLineMethodTest() {
  // import constants from tuning_constants.h
  using tuning::kBufferSize;
  using tuning::kCameraHeight;
  using tuning::kCameraMaxSrcHeight;
  using tuning::kCameraMinSrcConfidence;
  using tuning::kCameraMinPixelCount;
  using tuning::kCameraWidth;
  using tuning::ServoFactor;

  // define new parameters for Leslie's car
  enum EncoderSpeed {
    kSpeedLow = 4500,
    kSpeedMid = 5000,
    kSpeedHigh = 5500,
  };
  constexpr int kServoCenter = 665;
  constexpr int kServoLeftBound = 900;
  constexpr int kServoRightBound = 430;
  constexpr bool kEnableLcd = false;

  // initialize LEDs
  Led::Config led_config;
  led_config.is_active_low = true;
  led_config.id = 0;
  Led led1(led_config);  // main loop heartbeat
  led_config.id = 1;
  Led led2(led_config);  // buffer processing led
  led_config.id = 2;
  Led led3(led_config);  // unused
  led_config.id = 3;
  Led led4(led_config);  // initialization led
  led1.SetEnable(false);
  led2.SetEnable(false);
  led3.SetEnable(false);
  led4.SetEnable(true);

  // initialize camera
  Ov7725::Config camera_config;
  camera_config.id = 0;
  camera_config.w = kCameraWidth;
  camera_config.h = kCameraHeight;
  unique_ptr<Ov7725> camera(new Ov7725(camera_config));
  if (kBufferSize != camera->GetBufferSize()) {
    return;
  }
  camera->Start();
  while (!camera->IsAvailable()) {}

  // initialize servo
  FutabaS3010::Config servo_config;
  servo_config.id = 0;
  FutabaS3010 servo(servo_config);
  servo.SetDegree(kServoCenter);

  // initialize motors
  AlternateMotor::Config altmotor_config;
  altmotor_config.id = 0;
  AlternateMotor motor1(altmotor_config);
  altmotor_config.id = 1;
  AlternateMotor motor2(altmotor_config);
  motor1.SetClockwise(false);
  motor2.SetClockwise(true);
  motor1.SetPower(0);
  motor2.SetPower(0);

  // initialize LCD
  St7735r::Config lcd_config;
  lcd_config.fps = 10;
  lcd_config.is_revert = true;
  unique_ptr<St7735r> lcd(new St7735r(lcd_config));
  lcd->Clear();

  // initialize lcd console
  LcdConsole::Config console_config;
  console_config.lcd = lcd.get();
  unique_ptr<LcdConsole> console(new LcdConsole(console_config));

  // initialize encoder
  unique_ptr<EncoderPController> epc1(new EncoderPController((uint8_t) 0, &motor1));
  unique_ptr<EncoderPController> epc2(new EncoderPController((uint8_t) 1, &motor2));
  unique_ptr<EncoderPControllerDebug> epc1_d(new EncoderPControllerDebug(epc1.get()));
  unique_ptr<EncoderPControllerDebug> epc2_d(new EncoderPControllerDebug(epc2.get()));

  Timer::TimerInt time_img = System::Time();  // current execution time
  uint16_t steer_value = kCameraWidth / 2;
  int32_t target_degree = kServoCenter;

  led4.SetEnable(false);

  while (true) {
    if (time_img != System::Time()) {
      time_img = System::Time();

      // heartbeat
      led1.SetEnable(time_img % 1000 >= 500);

      // copy the buffer for processing
      const Byte *pbuffer = camera->LockBuffer();
      std::array<Byte, kBufferSize> image1d{};
      CopyByteArray(*pbuffer, &image1d);
      camera->UnlockBuffer();

      led2.SetEnable(true);

      // 1d to 2d array
      array<array<bool, kCameraWidth>, kCameraHeight> image2d;
      ByteTo2DBitArray(image1d, &image2d);

      // apply median filter
      array<array<bool, kCameraWidth>, kCameraHeight> image2d_median;
      MedianFilter(image2d, &image2d_median);

      // fill in row information
      for (int rowIndex = kCameraHeight - 1; rowIndex >= 0; --rowIndex) {
        if (rowIndex == kCameraHeight - 1) {  // row closest to car
          if (!image2d_median.at(rowIndex).at(kCameraWidth / 2)) {  // middle is track - do normal processing
            RowInfo[rowIndex].right_bound = static_cast<Uint>(
                FindElement(image2d_median.at(rowIndex), kCameraWidth / 2, kCameraWidth - 1, true, true));
            RowInfo[rowIndex].left_bound = static_cast<Uint>(
                FindElement(image2d_median.at(rowIndex), kCameraWidth / 2, 0, true, true));
          } else {  // middle is not track! - alternate processing
            // take the previous steer_value, and assume the track will be at that side as well
            if (steer_value > kCameraWidth / 2) {  // assumes track at right side
              RowInfo[rowIndex].left_bound = static_cast<Uint>(
                  FindElement(image2d_median.at(rowIndex), kCameraWidth / 2, kCameraWidth - 1, false, true));
              RowInfo[rowIndex].right_bound = static_cast<Uint>(
                  FindElement(image2d_median.at(rowIndex), RowInfo[rowIndex].left_bound, kCameraWidth - 1, true, true));
            } else if (steer_value < kCameraWidth / 2) {  // assumes track at left side
              RowInfo[rowIndex].right_bound =
                  static_cast<Uint>(FindElement(image2d_median.at(rowIndex), kCameraWidth / 2, 0, false, true));
              RowInfo[rowIndex].left_bound = static_cast<Uint>(
                  FindElement(image2d_median.at(rowIndex), RowInfo[rowIndex].right_bound, 0, true, true));
            }
          }
        } else {  // all other rows - dependent on the closest row
          RowInfo[rowIndex].right_bound = static_cast<Uint>(
              FindElement(image2d_median.at(rowIndex),
                          RowInfo[rowIndex + 1].center_point,
                          kCameraWidth - 1,
                          true,
                          true));
          RowInfo[rowIndex].left_bound = static_cast<Uint>(
              FindElement(image2d_median.at(rowIndex), RowInfo[rowIndex + 1].center_point, 0, true, true));
        }
        // calculate the dependencies
        RowInfo[rowIndex].center_point = (RowInfo[rowIndex].left_bound + RowInfo[rowIndex].right_bound) / 2;
        RowInfo[rowIndex].track_count = RowInfo[rowIndex].right_bound - RowInfo[rowIndex].left_bound;
      }

      led2.SetEnable(false);

      // render center line on lcd
      if (kEnableLcd) {
        lcd->SetRegion(Lcd::Rect(0, 0, kCameraWidth, kCameraHeight));
        lcd->FillColor(Lcd::kWhite);
        // lcd->FillBits(Lcd::kBlack, Lcd::kWhite, image1d.data(),  kBufferSize * 8);
        for (Uint i = kCameraHeight - 1; i > 0; --i) {
          if (RowInfo[i].track_count < kCameraMinPixelCount) {
            break;
          }
          lcd->SetRegion(Lcd::Rect(RowInfo[i].center_point, i, 1, 1));
          lcd->FillColor(Lcd::kRed);
          lcd->SetRegion(Lcd::Rect(RowInfo[i].left_bound, i, 1, 1));
          lcd->FillColor(Lcd::kBlack);
          lcd->SetRegion(Lcd::Rect(RowInfo[i].right_bound, i, 1, 1));
          lcd->FillColor(Lcd::kBlack);
        }
        lcd->SetRegion(Lcd::Rect(steer_value, 79, 1, 40));
        lcd->FillColor(Lcd::kBlack);
      }

      // average the center points to construct an average direction to move in
      uint8_t valid_count = 0;
      for (Uint i = kCameraHeight - 1; i > kCameraMaxSrcHeight; --i) {
        if (RowInfo[i].track_count < kCameraMinPixelCount) {
          // leave the loop if row is not part of the track anymore
          break;
        }
        if (!image2d_median[i][RowInfo[i].center_point]) {
          steer_value += RowInfo[i].center_point;
          ++valid_count;
        }
      }

      // TODO: Add weighting to the center points at different heights
      steer_value /= valid_count;

      // set the target degree
      if (steer_value == 0) {
        // no valid data - use previous data
        target_degree = servo.GetDegree();
      } else if (valid_count < kCameraMinSrcConfidence) {
        // insufficient data - use previous data
        target_degree = servo.GetDegree();
      } else if (steer_value < (kCameraWidth * 6 / 20) || steer_value > (kCameraWidth * 14 / 20)) {
        // edge 90% for either side - steep steering
        target_degree = kServoCenter - ((steer_value - (kCameraWidth / 2)) * ServoFactor::kFactorHigh);
      } else if (steer_value < (kCameraWidth * 8 / 20) || steer_value > (kCameraWidth * 12 / 20)) {
        // middle 10% for either side - medium steering
        target_degree = kServoCenter - ((steer_value - (kCameraWidth / 2)) * ServoFactor::kFactorMid);
      } else {
        // center 10% for either side - adjustment
        target_degree = kServoCenter - ((steer_value - (kCameraWidth / 2)) * ServoFactor::kFactorLow);
      }

      // commit motor and servo changes
      if (target_degree > kServoLeftBound) {
        // value > left bound: higher speed for compensation
        target_degree = kServoLeftBound;
        epc1->SetTargetSpeed(EncoderSpeed::kSpeedMid);
        epc2->SetTargetSpeed(-EncoderSpeed::kSpeedMid);
      } else if (target_degree < kServoRightBound) {
        // value < right bound: higher speed for compensation
        target_degree = kServoRightBound;
        epc1->SetTargetSpeed(EncoderSpeed::kSpeedMid);
        epc2->SetTargetSpeed(-EncoderSpeed::kSpeedMid);
      } else if (target_degree < (kServoCenter + kServoRightBound) / 2) {
        // value < half of right steer: slower speed to decrease turning radius
        epc1->SetTargetSpeed(EncoderSpeed::kSpeedLow);
        epc2->SetTargetSpeed(-EncoderSpeed::kSpeedLow);
      } else if (target_degree > (kServoCenter + kServoLeftBound) / 2) {
        // value > half of left steer: slower speed to decrease turning radius
        epc1->SetTargetSpeed(EncoderSpeed::kSpeedLow);
        epc2->SetTargetSpeed(-EncoderSpeed::kSpeedLow);
      } else {
        epc1->SetTargetSpeed(EncoderSpeed::kSpeedHigh);
        epc2->SetTargetSpeed(-EncoderSpeed::kSpeedHigh);
      }
      servo.SetDegree(static_cast<uint16_t>(target_degree));
    }

    epc1_d->OutputEncoderMotorValues(console.get());
  }
}
