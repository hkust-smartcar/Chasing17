/*
 * debug_console.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Dipsy Wong (dipsywong98)
 *
 * Implementation for DebugConsole (v2) class.
 *
 */

#include "debug_console.h"

#include <cstdio>

#include "libsc/joystick.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"

using libsc::Joystick;
using libsc::Lcd;
using libsc::LcdTypewriter;
using libsc::St7735r;
using libsc::System;

Item* Item::setText(char* t) {
  text = t;
  return this;
}

Item* Item::setListener(Fptr fptr) {
  listener = fptr;
  return this;
}

Item* Item::setValuePtr(float* v) {
  value = v;
  return this;
}

Item* Item::setValue(float v) {
  if (value != nullptr) *value = v;
  return this;
}

Item* Item::setInterval(float v) {
  interval = v;
  return this;
}

Item* Item::setReadOnly(bool isReadOnly) {
  readOnly = isReadOnly;
  return this;
}

DebugConsole::DebugConsole(Joystick* joystick, St7735r* lcd, LcdTypewriter* writer, int displayLength)
    : joystick(joystick), lcd(lcd), writer(writer), displayLength(displayLength) {}

void DebugConsole::EnterDebug() {
  int flag = 1;
  Item item(">>exit<<");
  PushItem(item);
  int index = items.size();
  ListItems();
  while (flag) {
    Listen();
  }
  items.erase(items.end() - (index - items.size()));
  clear();
}

void DebugConsole::Listen() {
  if (jState != joystick->GetState()) {
    jState = joystick->GetState();
    tFlag = System::Time() + threshold;
    listenerDo(jState);
  } else if (System::Time() > tFlag) {
    tFlag = System::Time() + cd;
    listenerDo(jState);
  }
}

void DebugConsole::ListItems() {
  clear();
  for (int i = topIndex; i < (items.size() < topIndex + displayLength ? items.size() : topIndex + displayLength); i++) {
    printItem(i, i == focus);
  }
}

void DebugConsole::ListItemValues() {
  clear();
  for (int i = topIndex; i < (items.size() < topIndex + displayLength ? items.size() : topIndex + displayLength); i++) {
    printItemValue(i, i == focus);
  }
}

void DebugConsole::printItem(int index, bool isInverted) {
  printxy(0, index - topIndex, items[index].getText(), isInverted);
  printItemValue(index, isInverted);
}

void DebugConsole::printItemValue(int index, bool isInverted) {
  if (items[index].getValuePtr() != nullptr) {
    char buff[20];
    sprintf(buff, "%.5lf", items[index].getValue());
    printxy(7, index - topIndex, buff, isInverted);
  }
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

void DebugConsole::printxy(int x, int y, char* c, int inverted) {
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

void DebugConsole::clear() {
  lcd->SetRegion(Lcd::Rect(0, offset, 128, displayLength * 15));
  lcd->FillColor(0x0000);
}

int DebugConsole::listenerDo(Joystick::State key) {
  Item item = items[focus];
  switch (key) {
    case Joystick::State::kDown:
      printItem(focus);    //remove highLighting
      focus++;                //move down focus
      if (focus >= items.size()) {    //focus below last item then jump back to first
        topIndex = 0;
        focus = 0;
        ListItems();
      } else if (focus >= topIndex + displayLength) {    //next page
        topIndex += displayLength;
        ListItems();
      } else {
        printItem(focus, true);    //update highlighted item
      }
      break;
    case Joystick::State::kUp:
      printItem(focus);
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
        printItem(focus, true);
      }
      break;
    case Joystick::State::kSelect:
      if (item.getListener() != nullptr) {
        item.getListener()();
      } else if (item.getText() == ">>exit debug<<")
        return 0;
      break;
    case Joystick::State::kLeft:
      if (item.getValuePtr() != nullptr && !item.isReadOnly()) {
        item.setValue(item.getValue() - item.getInterval());
        printItemValue(focus, true);
      }
      break;
    case Joystick::State::kRight:
      if (item.getValuePtr() != nullptr && !item.isReadOnly()) {
        item.setValue(item.getValue() + item.getInterval());
        printItemValue(focus, true);
      }
      break;
    default:
      return 1;
  }
  return 1;
}
