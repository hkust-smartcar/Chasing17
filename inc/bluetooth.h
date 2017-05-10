/* bluetooth.h
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), David Mak (Derppening)
 *
 * BTComm class
 * Interface for Bluetooth communication between two devices. While both
 * devices are not required to use this class, communication between these
 * devices are required to send/receive according to the protocol established
 * in this class. This interface also stores data values sent from other
 * devices and provides function to access them.
 *
 * Usage:
 * Pass an already-defined libsc::k60::JyMcuBt106::Config struct into the
 * class. No additional setup will be needed.
 *
 * All send... functions will send the current values of the given parameter
 * to the other device.
 * All req... functions will request the values of the given parameter from
 * the other device.
 * All get... functions will retrieve the values of the given parameter of the
 * other device from the buffer.
 *
 * Prerequisites:
 * - CarManager
 *
 */

#ifndef CHASING17_BLUETOOTH_H_
#define CHASING17_BLUETOOTH_H_

#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <map>
#include <memory>

#include "libsc/led.h"
#include "libsc/system.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/misc_types.h"

#include "car_manager.h"
#include "util/util.h"

class BTComm {
 public:
  BTComm(libsc::k60::JyMcuBt106::Config bt_config) {
    bt_config.rx_isr = BTListener;
    bluetooth_ = util::make_unique<libsc::k60::JyMcuBt106>(bt_config);
    ptrBT_ = bluetooth_.get();
    srand(libsc::System::Time());
  };

  void sendByte(Byte data) {
    ptrBT_->SendBuffer(&data, 1);
  }


  static int fetchSize(){
	  return NAKbuffer_.size();
  }

  /**
   * Send current speed to the other car through bluetooth.
   *
   * @param speed Current speed
   */
  static void sendSpeed(uint8_t speed) {
    sendData(DataType::kSpeed, static_cast<uint16_t>(speed));
  }

  /**
   * Get opponent's current speed in buffer.
   *
   * @return Opponent's current speed in buffer
   */
  uint8_t getBufferSpeed() {
    return speed_;
  };

  /**
   * Request opponent's current speed to buffer.
   */
  void reqSpeed(){
    sendData(DataType::kReq, ReqType::kSpeed);
  }

  /**
   * Send current slope degree to the other car through bluetooth.
   *
   * @param slope_ Current slope degree
   */
  static void sendSlopeDeg(int8_t slope_deg) {
    sendData(DataType::kSlopeDeg, static_cast<uint16_t>(slope_deg));
  }

  /**
   * Get opponent's current slope in buffer.
   *
   * @return Opponent's current slope in buffer
   */
  int8_t getBufferSlopeDeg() {
    return slope_deg_;
  };

  /**
   * Request opponent's current slope to buffer.
   */
  void reqSlopeDeg(){
    sendData(DataType::kReq, ReqType::kSlopeDeg);
  }

  /**
   * Send current distance from the other car to the other cat through bluetooth.
   *
   * @param Current distance from the other car
   */
  static void sendDist(uint8_t dist) {
    sendData(DataType::kDist, static_cast<uint16_t>(dist));
  }

  /**
   * Get opponent's current distance from this car in buffer
   *
   * @return Opponent's current distance from this car in buffer
   */
  uint8_t getBufferDist();

  /**
   * Request opponent's current distance.
   */
  void reqDist(){
    sendData(DataType::kReq, ReqType::kDist);
  }

  /**
   * Send current road feature to the other car through bluetooth.
   *
   * @param Current road feature
   */
  static void sendFeature(CarManager::Feature feat) {
    sendData(DataType::kFeature, static_cast<uint16_t>(feat));
  }

  /**
   * Get opponent's current road feature in buffer.
   *
   * @return Opponent's current road feature
   */
  CarManager::Feature getBufferFeature() {
    return feat_;
  };

  /**
   * Request opponent's current feature.
   */
  void reqFeature(){
    sendData(DataType::kReq, ReqType::kFeature);
  }

  /**
   * Send current position (left or right) to the other car through bluetooth.
   *
   * @param side Current position (left or right)
   */
  static void sendSide(CarManager::Side side) {
    sendData(DataType::kSide, static_cast<uint16_t>(side));
  }

  /**
   * Get opponent's current position (left or right) in buffer.
   *
   * @return Opponent's current position (left or right)
   */
  CarManager::Side getBufferSide() {
    return side_;
  };

  /**
   * Request opponent's current side.
   */
  void reqSide(){
    sendData(DataType::kReq, ReqType::kSide);
  }

  /**
   * Send current car information to the other car through buffer.
   * Should be used mainly by the car behind.
   *
   * @param speed Current speed
   * @param slope_deg Current slope deg
   * @param dist Current distance from the other car
   * @param side Current position (left or right)
   */
  void sendInfo(uint8_t speed, int8_t slope_deg, uint8_t dist, CarManager::Side side) {
    sendSpeed(speed);
    sendSlopeDeg(slope_deg);
    sendDist(dist);
    sendSide(side);
  };

  enum struct OvertakeStatus {
    kRequested = 0,
    kAgreed,
    kRejected,
    kNeutral
  };

  /**
   * Send a request for overtake.
   */
  void reqOvertake() {
    sendData(DataType::kReq, ReqType::kOvertake);
  }

  /**
   * Check if there is a request for overtake.
   *
   * @note should be called with short time intervals
   * @return True if there is a request, False otherwise
   */
  OvertakeStatus getOvertakeReq() {
    return OvertakeReq_;
  };

  /**
   * Reply to the overtake request.
   *
   * @param agreed True if agree to be overtook, False otherwise
   */
  void replyOvertakeReq(bool agreed) {
    if (agreed) {
      sendData(DataType::kReq, ReqType::kOvertakeAgr);
    } else {
      sendData(DataType::kReq, ReqType::kOvertakeRej);
    }
  }

  /**
   * Send a notification for identity switching.
   */
  void ReqSwitchID() {
    sendData(DataType::kReq, ReqType::kIDSwitch);
  }

  /**
   * Check if there is a notification for identity switching.
   *
   * @note should be called with short time intervals
   * @return True if there is a notification, False otherwise
   */
  bool hasSwitchIDReq() {
    return SwitchIDReq_;
  };

  /**
   * Reset overtake request buffer.
   */
  void resetOvertakeReq() {
    OvertakeReq_ = OvertakeStatus::kNeutral;
  }

  /**
   * Reset switch identity notification buffer.
   */
  void resetSwitchIDReq() {
    SwitchIDReq_ = false;
  }

  /**
   * Send start request
   */
  void sendStartReq(){
	sendData(DataType::kReq, ReqType::kStart);
  }

  /**
   * Check if there is a start request
   */
  bool hasStartReq(){
	return hasStartReq_;
  }

  /**
   * Resend those information that are unacknowledged.
   *
   * @note should be called with short time intervals.
   */
  void resendNAKData();

  static libsc::Led* led_ptr;

 private:
  struct BitConsts {
    static constexpr Byte kHandshake = 0xF0;
    static constexpr Byte kEND = 0xFF;
    static constexpr Byte kACK = 0xF1;
  };

  struct DataType {
    static constexpr Byte kSpeed = 0x10;
    static constexpr Byte kSlopeDeg = 0x11;
    static constexpr Byte kDist = 0x12;
    static constexpr Byte kFeature = 0x13;
    static constexpr Byte kSide = 0x14;
    static constexpr Byte kReq = 0x15;
  };

  struct ReqType {
    static constexpr Byte kOvertake = 0xE0;
    static constexpr Byte kOvertakeRej = 0xEE;
    static constexpr Byte kOvertakeAgr = 0xEF;
    static constexpr Byte kIDSwitch = 0xD0;
    static constexpr Byte kSpeed = 0xC0;
    static constexpr Byte kSlopeDeg = 0xC1;
    static constexpr Byte kDist = 0xC2;
    static constexpr Byte kFeature = 0xC3;
    static constexpr Byte kSide = 0xC4;
    static constexpr Byte kStart = 0xB0;
  };

  std::unique_ptr<libsc::k60::JyMcuBt106> bluetooth_;
  static libsc::k60::JyMcuBt106 *ptrBT_;

  static bool BTListener(const Byte *data, const size_t size);
  //1. Receive DATA -> Update corresponding variables
  //   -> ACK format: kACK (1B) | ID (1B) | 0x00 (3B) | kEND (1B), total 6B
  //   If not enough 8B or no kEND within 10ms -> Disregard message
  //2. Receive ACK -> Clear corresponding entry in NAKbuffer_
  //   If not enough 4B or no kEND within 10ms -> Disregard message

  static uint8_t speed_;
  static int8_t slope_deg_;
  static uint8_t dist_;
  static CarManager::Feature feat_;
  static CarManager::Side side_;

  static OvertakeStatus OvertakeReq_;
  static bool SwitchIDReq_;

  static bool hasStartReq_;

  static Byte dataArray_[6];
  static uint8_t dataIndex_;

  static uint8_t mapIndex_;

  static bool testBool;

  /**
   * Generic send data through bluetooth function.
   *
   * @param type Types of information
   * @param data Data to be transmitted
   */
  static void sendData(unsigned char type, uint16_t data);
  //1. Generate ID and push ID + content to NAKBuffer_
  //2. Data format: kHandshake (1B) | ID (1B) | Type (1B) | Data (2B) | kEnd (1B), total 6B

  /**
   * Buffer map for info that are unacknowledged.
   *
   * @key Unique random number for identifying package
   * @value Pair of data type (in char) and data (in uint16_t)
   */
  static std::map<int, std::pair<char, uint16_t>> NAKbuffer_;

  friend class Overtake;
};

#endif  //CHASING17_BLUETOOTH_H_
