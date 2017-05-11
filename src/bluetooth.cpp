/* bluetooth.cpp
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Peter Tse (mcreng), David Mak (Derppening)
 *
 * Implementation for BTComm class.
 *
 */

#include "bluetooth.h"
#include "algorithm/overtake.h"
#include "libsc/system.h"

// static variables
uint8_t BTComm::speed_ = 0;
int8_t BTComm::slope_deg_ = 0;
uint8_t BTComm::dist_ = 0;
CarManager::Feature BTComm::feat_;
CarManager::Side BTComm::side_ = CarManager::Side::kMiddle;
BTComm::OvertakeStatus BTComm::OvertakeReq_ = BTComm::OvertakeStatus::kNeutral;
bool BTComm::SwitchIDReq_ = false;
libsc::k60::JyMcuBt106* BTComm::ptrBT_ = nullptr;
std::map<int, std::pair<char, uint16_t>> BTComm::NAKbuffer_{};
Byte BTComm::dataArray_[6]{0, 0, 0, 0, 0, 0};
uint8_t BTComm::dataIndex_ = 0;
uint8_t BTComm::mapIndex_ = 0;
libsc::Led* BTComm::led_ptr = nullptr;
bool BTComm::hasStartReq_ = false;
bool BTComm::hasFinishedOvertake_ = false;

void BTComm::sendData(unsigned char type, uint16_t data) {
  uint8_t ID = (mapIndex_ %= 10)++;
  Byte dataArray[6];
  dataArray[0] = BitConsts::kHandshake;
  dataArray[1] = ID;
  dataArray[2] = type;
  dataArray[3] = static_cast<Byte>(data >> 8);
  dataArray[4] = static_cast<Byte>(data);
  dataArray[5] = BitConsts::kEND;
  ptrBT_->SendBuffer(dataArray, 6);
  auto a = NAKbuffer_.emplace(ID, std::make_pair(type, data));
  if (!a.second){
	  NAKbuffer_.erase(a.first);
	  NAKbuffer_.emplace(ID, std::make_pair(type,data));
  }
}

void BTComm::resendNAKData() {
  Byte dataArray[6];
  for (auto&& e : NAKbuffer_) {
    dataArray[0] = BitConsts::kHandshake;
    dataArray[1] = static_cast<Byte>(e.first);
    dataArray[2] = (e.second).first;
    uint16_t data = (e.second).second;
    dataArray[3] = static_cast<Byte>(data >> 8);
    dataArray[4] = static_cast<Byte>(data);
    dataArray[5] = BitConsts::kEND;
    ptrBT_->SendBuffer(dataArray, 6);
  }
}

bool BTComm::BTListener(const Byte* data, size_t size) {
  if (led_ptr != nullptr) {
    led_ptr->Switch();
  }
  if (data == nullptr) return true; //just a basic check, should not be triggered anyways
  if ((data[0] == BitConsts::kHandshake || data[0] == BitConsts::kACK) && (dataIndex_ == 0)) {
    dataArray_[0] = data[0];
    dataIndex_ = 1;
    return true;
  } else if ((data[0] == BitConsts::kEND) && (dataIndex_ == 5)) {
    dataArray_[5] = data[0];
    dataIndex_ = 0;
  } else if (dataIndex_ != 0 && dataIndex_ != 5) {
    dataArray_[dataIndex_] = data[0];
    dataIndex_++;
    return true;
  } else {
    for (int i = 0; i < 6; i++) {
      dataArray_[i] = 0;
      dataIndex_ = 0;
    }
    return true;
  }

  //fetch info
  Byte ID = dataArray_[1];
  Byte type = dataArray_[2];
  uint16_t value = (dataArray_[3] << 8) + dataArray_[4];

  //if ACK
  if (dataArray_[0] == BitConsts::kACK) {
    //invalid package
    if (type != 0x00 && value != 0x00) {
      return true;
    }
    //erase map info
    /*if (NAKbuffer_.count(dataArray_[1]) > 0) {
      NAKbuffer_.erase(dataArray_[1]);
    }*/
    auto find = NAKbuffer_.find(dataArray_[1]);
    if (find != NAKbuffer_.end()) {
      NAKbuffer_.erase(find);
    }

    return true;
  }

  //if data
  //match datatype and reqtype
  switch (type) {
    case DataType::kSpeed:
      speed_ = static_cast<uint8_t>(value);
      break;
    case DataType::kSlopeDeg:
      slope_deg_ = static_cast<int8_t>(value);
      break;
    case DataType::kDist:
      dist_ = static_cast<uint8_t>(value);
      break;
    case DataType::kFeature:
      feat_ = static_cast<CarManager::Feature>(value);
      break;
    case DataType::kSide:
      side_ = static_cast<CarManager::Side>(value);
      break;
    case DataType::kReq:
      switch (value) {
        case ReqType::kOvertake:
          OvertakeReq_ = OvertakeStatus::kRequested;
          break;
        case ReqType::kFinishOvertake:
          hasFinishedOvertake_ = true;
          break;
        case ReqType::kOvertakeAgr:
          OvertakeReq_ = OvertakeStatus::kAgreed;
          break;
        case ReqType::kOvertakeRej:
          OvertakeReq_ = OvertakeStatus::kRejected;
          break;
        case ReqType::kIDSwitch:
          SwitchIDReq_ = true;
          break;
        case ReqType::kSpeed:
          sendSpeed(CarManager::GetLeftSpeed());
          break;
        case ReqType::kSlopeDeg:
          sendSlopeDeg(CarManager::GetSlope());
          break;
        case ReqType::kDist:
          sendDist(Overtake::GetDist());
          break;
        case ReqType::kFeature:
          sendFeature(CarManager::GetFeature());
          break;
        case ReqType::kSide:
          sendSide(CarManager::GetSide());
          break;
        case ReqType::kStart:
          hasStartReq_ = true;
          break;
        default:
          //invalid package
          return true;
      }
      break;
    default:
      //invalid package
      return true;
  }

  //send ACK package
  Byte ACKArray[6];
  ACKArray[0] = BitConsts::kACK;
  ACKArray[1] = ID;
  ACKArray[2] = 0x00;
  ACKArray[3] = 0x00;
  ACKArray[4] = 0x00;
  ACKArray[5] = BitConsts::kEND;

  ptrBT_->SendBuffer(ACKArray, 6);

  return true;
}
