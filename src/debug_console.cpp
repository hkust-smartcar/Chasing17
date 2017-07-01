/*
 * debug_console.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Dipsy Wong (dipsywong98)
 *
 * Implementation for DebugConsole (v4.2) class.
 *
 */

#include "debug_console.h"

#include <cstdio>

#include "libsc/joystick.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libbase/k60/flash.h"

using libsc::Joystick;
using libsc::Lcd;
using libsc::LcdTypewriter;
using libsc::St7735r;
using libsc::System;

DebugConsole::DebugConsole(Joystick* joystick, St7735r* lcd, LcdTypewriter* writer, int displayLength)
    : joystick(joystick), lcd(lcd), writer(writer), displayLength(displayLength) {}

void DebugConsole::EnterDebug(char* leave_msg) {
  flag = true;
  Item item;
  item.text = leave_msg;
  items.insert(items.begin() + 0, item);

  int index = items.size();
  Load();
  ListItems();
  while (flag) {
    Listen();
//    if(System::Time()%100==0)
    Save();
  }
  items.erase(items.begin());
  Clear();
}

void DebugConsole::Listen() {
  if (jState != joystick->GetState()) {
    jState = joystick->GetState();
    tFlag = System::Time() + threshold;
    ListenerDo(jState);
  } else if (System::Time() > tFlag) {
    tFlag = System::Time() + cd;
    ListenerDo(jState);
  }
}

void DebugConsole::PushItem(char* text, uint16_t* valuePtr, float interval) {
  Item item;
  item.text = text;
  item.type = VarType::kUint16;
  item.vIndex = uint16t_values.size();
  item.interval = interval;
  uint16t_values.push_back(valuePtr);
  items.push_back(item);
  flash_sum += sizeof(*valuePtr);
}
void DebugConsole::PushItem(char* text, int32_t* valuePtr, float interval) {
  Item item;
  item.text = text;
  item.type = VarType::kInt32;
  item.vIndex = int32t_values.size();
  item.interval = interval;
  int32t_values.push_back(valuePtr);
  items.push_back(item);
  flash_sum += sizeof(*valuePtr);
}
void DebugConsole::PushItem(char* text, float* valuePtr, float interval) {
  Item item;
  item.text = text;
  item.type = VarType::kFloat;
  item.vIndex = float_values.size();
  item.interval = interval;
  float_values.push_back(valuePtr);
  items.push_back(item);
  flash_sum += sizeof(*valuePtr);
}
void DebugConsole::PushItem(char* text, bool* valuePtr, char* true_text, char* false_text) {
  Item item;
  item.text = text;
  item.type = VarType::kBool;
  item.vIndex = bool_values.size();
  item.true_text = true_text;
  item.false_text = false_text;
  bool_values.push_back(valuePtr);
  items.push_back(item);
  flash_sum += sizeof(*valuePtr);
}
void DebugConsole::PushItem(char* text, int32_t* valuePtr, char* true_text, char* false_text) {
  Item item;
  item.text = text;
  item.type = VarType::kBS;
  item.vIndex = int32t_values.size();
  item.bsIndex = 0;
  item.true_text = true_text;
  item.false_text = false_text;
  int32t_values.push_back(valuePtr);
  items.push_back(item);
  flash_sum += sizeof(*valuePtr);
}

void DebugConsole::SetItem(int index, Item item) {
  items[index].text = item.text;
  items[index].type = item.type;
  items[index].vIndex = item.vIndex;
  items[index].interval = item.interval;
  items[index].listener = item.listener;
  items[index].bsIndex = item.bsIndex; //for bitstring
  items[index].true_text = item.true_text;
  items[index].false_text = item.false_text;
}
Item DebugConsole::GetItem(int index) {
  return items[index];
}

void DebugConsole::ListItems() {
  Clear();
  for (int i = topIndex; i < (items.size() < topIndex + displayLength ? items.size() : topIndex + displayLength); i++) {
    PrintItem(i, i == focus);
  }
}

void DebugConsole::ListItemValues() {
  Clear();
  for (int i = topIndex; i < (items.size() < topIndex + displayLength ? items.size() : topIndex + displayLength); i++) {
    PrintItemValue(i, i == focus);
  }
}

void DebugConsole::ChangeItemValue(int index, bool IsIncrement) {
  Item item = items[index];
  float c = IsIncrement ? 1.0 : -1.0;
  switch (item.type) {
    case VarType::kNan:
      return;
      break;
    case VarType::kUint16:
      *uint16t_values[item.vIndex] += int(c * item.interval);
      break;
    case VarType::kInt32:
      *int32t_values[item.vIndex] += int(c * item.interval);
      break;
    case VarType::kFloat:
      *float_values[item.vIndex] += float(c * item.interval);
      break;
    case VarType::kBool:
      *bool_values[item.vIndex] = !*bool_values[item.vIndex];
      break;
    default:
      return;
      break;
  }
}

void DebugConsole::PrintItem(int index, bool isInverted) {
  Printxy(0, index - topIndex, items[index].text, isInverted);
  PrintItemValue(index, isInverted);
}

void DebugConsole::PrintItemValue(int index, bool isInverted) {

  char buff[20];
  Item item = items[index];
  switch (item.type) {
    case VarType::kNan:
      return;
      break;
    case VarType::kUint16:
      sprintf(buff, "%d", *uint16t_values[item.vIndex]);
      break;
    case VarType::kInt32:
      sprintf(buff, "%d", *int32t_values[item.vIndex]);
      break;
    case VarType::kFloat:
      sprintf(buff, "%.3lf", *float_values[item.vIndex]);
      break;
    case VarType::kBool:
      sprintf(buff, "%s", *bool_values[item.vIndex] ? item.true_text : item.false_text);
      break;
    case VarType::kBS:
      sprintf(buff,
              "%d:%s",
              item.bsIndex,
              (*int32t_values[item.vIndex] >> item.bsIndex) & 1 ? item.true_text : item.false_text);
      Printxy(7, index - topIndex, buff, isInverted);
      return;
      break;
    default:
      return;
      break;
  }
  Printxy(9, index - topIndex, buff, isInverted);
  return;
}

void DebugConsole::Load() {

  if (flash == nullptr) return;
  int start = 0;
  Byte* buff = new Byte[flash_sum];
  flash->Read(buff, flash_sum);
  for (int i = 0; i < uint16t_values.size(); i++) {
    uint16_t* v = uint16t_values[i];
    uint16_t temp = 0;
    memcpy((unsigned char*) &temp, buff + start, sizeof(*v));
    start += sizeof(*v);
    if (temp == temp)*v = temp;
  }
  for (int i = 0; i < int32t_values.size(); i++) {
    int32_t* v = int32t_values[i];
    int32_t temp = 0;
    memcpy((unsigned char*) &temp, buff + start, sizeof(*v));
    start += sizeof(*v);
    if (temp == temp)*v = temp;
  }
  for (int i = 0; i < float_values.size(); i++) {
    float* v = float_values[i];
    float temp = 0;
    memcpy((unsigned char*) &temp, buff + start, sizeof(*v));
    start += sizeof(*v);
    if (temp == temp)*v = temp;
  }
  for (int i = 0; i < bool_values.size(); i++) {
    bool* v = bool_values[i];
    bool temp = 0;
    memcpy((unsigned char*) &temp, buff + start, sizeof(*v));
    start += sizeof(*v);
    if (temp == temp)*v = temp;
  }
  delete[] buff;
}

void DebugConsole::Save() {
  if (flash == nullptr) return;
  int start = 0;
  Byte* buff = new Byte[flash_sum];
  for (int i = 0; i < uint16t_values.size(); i++) {
    uint16_t* v = uint16t_values[i];
    memcpy(buff + start, (unsigned char*) v, sizeof(*v));
    start += sizeof(*v);
  }
  for (int i = 0; i < int32t_values.size(); i++) {
    int32_t* v = int32t_values[i];
    memcpy(buff + start, (unsigned char*) v, sizeof(*v));
    start += sizeof(*v);
  }
  for (int i = 0; i < float_values.size(); i++) {
    float* v = float_values[i];
    memcpy(buff + start, (unsigned char*) v, sizeof(*v));
    start += sizeof(*v);
  }
  for (int i = 0; i < bool_values.size(); i++) {
    bool* v = bool_values[i];
    memcpy(buff + start, (unsigned char*) v, sizeof(*v));
    start += sizeof(*v);
  }
  flash->Write(buff, flash_sum);
  System::DelayMs(100);
  delete[] buff;
}

DebugConsole* DebugConsole::SetDisplayLength(int length) {
  displayLength = length;
  return this;
}

DebugConsole* DebugConsole::SetLongClickThreshold(int threshold) {
  this->threshold = threshold;
  return this;
}

DebugConsole* DebugConsole::SetLongClickCd(int cd) {
  this->cd = cd;
  return this;
}

DebugConsole* DebugConsole::SetOffset(int offset) {
  this->offset = offset;
  return this;
}

DebugConsole* DebugConsole::SetAutoFlash(bool flag) {
  this->auto_flash = flag;
  return this;
}

void DebugConsole::Printxy(int x, int y, char* c, int inverted) {
  if (inverted) {
    writer->SetTextColor(0x0000);
    writer->SetBgColor(0xFFFF);
  }
  lcd->SetRegion(Lcd::Rect(5 + x * 10, offset + y * 15, 128 - 5 - x * 10, 15));
  writer->WriteString(c);
  if (inverted) {
    writer->SetTextColor(0xFFFF);
    writer->SetBgColor(0x0000);
  }
}

void DebugConsole::Clear() {
  lcd->SetRegion(Lcd::Rect(0, offset, 128, displayLength * 15));
  lcd->FillColor(0x0000);
}

void DebugConsole::ListenerDo(Joystick::State key) {
  Item item = items[focus];
  switch (key) {
    case Joystick::State::kDown:
      PrintItem(focus);    //remove highLighting
      focus++;                //move down focus
      if (focus >= items.size()) {    //focus below last item then jump back to first
        topIndex = 0;
        focus = 0;
        ListItems();
      } else if (focus >= topIndex + displayLength) {    //next page
        topIndex += displayLength;
        ListItems();
      } else {
        PrintItem(focus, true);    //update highlighted item
      }
      break;
    case Joystick::State::kUp:
      PrintItem(focus);
      focus--;
      if (focus < 0) {
        focus = items.size() - 1;
        topIndex = items.size() - displayLength;
        topIndex = (topIndex > 0 ? topIndex : 0);
        ListItems();
      } else if (focus < topIndex) {
        topIndex -= displayLength;
        topIndex = (topIndex > 0 ? topIndex : 0);
        ListItems();
      } else {
        PrintItem(focus, true);
      }
      break;
    case Joystick::State::kSelect:
      if (item.type == VarType::kBS) {
        *int32t_values[item.vIndex] = *int32t_values[item.vIndex] ^ (1 << item.bsIndex);
        PrintItemValue(focus, true);
      } else if (item.listener != nullptr) {
        item.listener();
      } else if (flag && focus == 0)//leave item click
        flag = false;
      break;
    case Joystick::State::kLeft:
      if (flag && focus == 0)//leave item click
        flag = false;
      else if (item.type == VarType::kBS) {
        --items[focus].bsIndex %= 32;
        PrintItem(focus, true);
      } else if (item.type != VarType::kNan) {
        ChangeItemValue(focus, 0);
        PrintItemValue(focus, true);
      }
      break;
    case Joystick::State::kRight:
      if (flag && focus == 0)//leave item click
        flag = false;
      else if (item.type == VarType::kBS) {
        items[focus].bsIndex = (items[focus].bsIndex + 1) % 32;
        PrintItem(focus, true);
      } else if (item.type != VarType::kNan) {
        ChangeItemValue(focus, 1);
        PrintItemValue(focus, true);
      }
      break;
    default:
      return;
  }
  return;
}
