/*
 * debug_console.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Dipsy Wong (dipsywong98)
 *
 * Implementation for DebugConsole (v3) class.
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

Item* Item::SetText(char* t) {
  text = t;
  return this;
}

Item* Item::SetListener(Fptr fptr) {
  listener = fptr;
  return this;
}

Item* Item::SetValuePtr(float* v) {
  value = v;
  return this;
}

Item* Item::SetValue(float v) {
  if (value != nullptr) *value = v;
  return this;
}

Item* Item::SetInterval(float v) {
  interval = v;
  return this;
}

Item* Item::SetReadOnly(bool isReadOnly) {
  readOnly = isReadOnly;
  return this;
}

DebugConsole::DebugConsole(Joystick* joystick, St7735r* lcd, LcdTypewriter* writer, int displayLength)
    : joystick(joystick), lcd(lcd), writer(writer), displayLength(displayLength) {}

void DebugConsole::EnterDebug() {
  flag = true;
  Item item(">>exit<<");
  PushItem(item);
  int index = items.size();
  ListItems();
  while (flag) {
    Listen();
//    if(System::Time()%100==0)
//    	Save();
  }
  items.erase(items.end() - (index - items.size()));
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

void DebugConsole::PrintItem(int index, bool isInverted) {
  Printxy(0, index - topIndex, items[index].GetText(), isInverted);
  PrintItemValue(index, isInverted);
}

void DebugConsole::PrintItemValue(int index, bool isInverted) {
  if (items[index].GetValuePtr() != nullptr) {
    char buff[20];
    sprintf(buff, "%.3lf", items[index].GetValue());
    Printxy(9, index - topIndex, buff, isInverted);
  }
}

void DebugConsole::Load(){

	if(flash == nullptr) return;
	int start=0;
	Byte* buff = new Byte[flash_sum*4];
	flash->Read(buff,flash_sum*4);
	for(int i=0;i<items.size();i++){
		if(!items[i].IsFlashable()) continue;
		float* v=items[i].GetValuePtr();
		if(v==nullptr)continue;
		memcpy((unsigned char*) v, buff+start, 4);
		start+=4;
		if(*v!=*v)*v=0;
	}
	delete [] buff;
}

void DebugConsole::Save(){
	if(flash == nullptr) return;
	int start=0;
	Byte* buff = new Byte[flash_sum*4];
	for(int i=0; i<items.size();i++){
		if(!items[i].IsFlashable()) continue;
		float* v = items[i].GetValuePtr();
		if(v==nullptr)continue;
		memcpy(buff + start, (unsigned char*) v, 4);
		start+=4;
	}
	flash->Write(buff, flash_sum*4);
	System::DelayMs(100);
	delete [] buff;
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
      if (item.GetListener() != nullptr) {
        item.GetListener()();
      } else if (item.GetText() == ">>exit<<")
        flag=false;
      break;
    case Joystick::State::kLeft:
      if (item.GetText() == ">>exit<<")
        flag=false;
      if (item.GetValuePtr() != nullptr && !item.IsReadOnly()) {
        item.SetValue(item.GetValue() - item.GetInterval());
        PrintItemValue(focus, true);
      }
      break;
    case Joystick::State::kRight:
      if (item.GetText() == ">>exit<<")
        flag=false;
      if (item.GetValuePtr() != nullptr && !item.IsReadOnly()) {
        item.SetValue(item.GetValue() + item.GetInterval());
        PrintItemValue(focus, true);
      }
      break;
    default:
      return;
  }
  return;
}
