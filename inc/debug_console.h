/*
 * debug_console.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Dipsy Wong (dipsywong98)
 *
 * DebugConsole (v2) class
 * Console-based GUI for debugging purposes.
 *
 */

#ifndef CHASING17_DEBUG_CONSOLE_H_
#define CHASING17_DEBUG_CONSOLE_H_

#include <vector>

#include "libsc/joystick.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/st7735r.h"

typedef void(* Fptr)();

class Item {
 public:
  Item(char* text = nullptr, float* value = nullptr, bool readOnly = false)
      : text(text), value(value), readOnly(readOnly) {}

  //display text methods
  char* getText() { return text; }
  Item* setText(char* t);

  //listener methods
  Fptr getListenerptr() { return listener; }
  Fptr getListener() { return *listener; }
  Item* setListener(Fptr fptr);

  //value methods
  Item* setValuePtr(float* v);
  Item* setValue(float v);
  float valueIncre() { return *value += interval; }
  float valueDecre() { return *value -= interval; }
  float* getValuePtr() { return value; }
  float getValue() { return value != nullptr ? *value : 0; }
  Item* setInterval(float v);
  float getInterval() { return interval; }

  bool isReadOnly() { return readOnly; }
  Item* setReadOnly(bool isReadOnly);

 private:
  char* text;
  float* value = nullptr;
  bool readOnly;
  Fptr listener = nullptr;
  float interval = 1;
  void init() {}
};

class DebugConsole {
 public:
  /**
   * constructor of debug console
   * joystick, lcd, writer pointer and the display length, which limit the display size of debug console
   */
  DebugConsole(libsc::Joystick* joystick, libsc::St7735r* lcd, libsc::LcdTypewriter* writer, int displayLength);

  /**
   * pause and start debugging
   */
  void EnterDebug();

  /**
   * just listen and do, no pause
   */
  void Listen();

  /**
   * adding items to debug console
   */
  void PushItem(Item item) { items.push_back(item); }

  void InsertItem(Item item, int index = 0) { items.insert(items.begin() + index, item); }

  /**
   * print item start from topIndex, total amount displayLength
   */
  void ListItems();

  /**
   * print item start from topIndex, total amount displayLength
   */
  void ListItemValues();

  /**
   * print a single item's text and value
   */
  void printItem(int index, bool isInverted = false);

  /**
   * print a single item's value
   */
  void printItemValue(int index, bool isInverted = false);

  //parameters setters
  DebugConsole* SetDisplayLength(int length);
  DebugConsole* SetLongClickThreshold(int threshold);
  DebugConsole* SetLongClickCd(int cd);
  DebugConsole* SetOffset(int offset);

  //parameters getters
  int GetDisplayLength() { return displayLength; }
  int GetLongClickThershold() { return threshold; }
  int GetLongClickCd() { return cd; }
  int GetOffset() { return offset; }

 private:
  int focus = 0;
  int topIndex = 0;
  std::vector<Item> items;
  libsc::Joystick* joystick;
  libsc::St7735r* lcd;
  libsc::LcdTypewriter* writer;
  libsc::Joystick::State jState = libsc::Joystick::State::kIdle;
  int tFlag = 0;
  int threshold = 1000; //long click thershold
  int displayLength;    //limit the region to display
  int cd = 0;    //time needed to trigger next long click listener
  int offset = 0; //distance away from top of lcd

  void printxy(int x, int y, char* c, int inverted = false);

  void clear();

  int listenerDo(libsc::Joystick::State key);
};

#endif // CHASING17_DEBUG_CONSOLE_H_
