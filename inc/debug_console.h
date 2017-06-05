/*
 * debug_console.h
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Dipsy Wong (dipsywong98)
 *
 * DebugConsole class
 * Console-based GUI for debugging purposes.
 *
 */

#ifndef CHASING17_DEBUG_CONSOLE_H_
#define CHASING17_DEBUG_CONSOLE_H_

#include "libsc/joystick.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"

#define START_IDLE    0
#define DOWN_SELECT   1
#define DOWN_LEFT     2
#define DOWN_RIGHT    3
//just up or end5
#define END_IDLE      4
#define UP_SELECT     5
#define UP_LEFT       6
#define UP_RIGHT      7
//down for a while
#define LONG_IDLE     8
#define LONG_SELECT   9
#define LONG_LEFT     10
#define LONG_RIGHT    11

#define IDLE          0
#define SELECT        1
#define LEFT          2
#define RIGHT         3

#define START         0
#define END           1
#define DOWN          0
#define UP            1
#define LONG          2

#define GET_TIME System::Time()

class DebugConsole {
  typedef void(* Fptr)();
 public:
  class Item {
   public:

    Item(char* text = NULL, int* value = NULL, bool readOnly = false);

    //display text methods
    char* getText() { return text; }
    void setText(char* t) { text = t; }

    //listener methods
    Fptr getListener(int type) { return listeners[type]; }
    Fptr getListeners() { return *listeners; }
    void setListener(int type, Fptr fptr) { listeners[type] = fptr; }

    //value methods
    void setValuePtr(int* v) { value = v; }
    void setValue(int v);
    int* getValuePtr() { return value; }
    int getValue();

    bool isReadOnly() { return readOnly; }
    void setReadOnly(bool isReadOnly) { readOnly = isReadOnly; }

   private:
    Fptr listeners[12];
    char* text;
    int* value = NULL;
    bool readOnly;
    bool upLongExclusive;
    void init();
  };

  DebugConsole(libsc::Joystick* joystick, libsc::St7735r* lcd, libsc::LcdTypewriter* writer);

  void enterDebug();

  void pushItem(Item item);
  void insertItem(Item item, int index = 0);
  void listItems(int start = 0);
  void printItem(int index);

 private:
  void printxy(int x, int y, char* c, int l = 100);
  void showFocus(bool flag = true);
  void clear();
  int listen(libsc::Joystick::State key, int state);

  int length = 0;
  int focus = 0;
  int topIndex = 0;
  Item items[50];
  libsc::Joystick* joystick;
  libsc::St7735r* lcd;
  libsc::LcdTypewriter* writer;
  int threshold = 1000;
};

#endif /* CHASING17_DEBUG_CONSOLE_H_ */
