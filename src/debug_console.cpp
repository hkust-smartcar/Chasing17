/*
 * debug_console.cpp
 *
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Author: Dipsy Wong (dipsywong98)
 *
 * Implementation for DebugConsole class.
 *
 */

#include "debug_console.h"

#include <cstdio>

#include "libsc/joystick.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libutil/misc.h"

using libsc::Joystick;
using libsc::Lcd;
using libsc::LcdTypewriter;
using libsc::St7735r;
using libsc::System;
using libutil::Clamp;
using std::sprintf;

DebugConsole::DebugConsole(Joystick* joystick, St7735r* lcd, LcdTypewriter* writer)
    : joystick(joystick), lcd(lcd), writer(writer) {
  Item item(">>exit debug<<");
  pushItem(item);
}

void DebugConsole::enterDebug() {
  listItems();
  int flag = 1, time_next, time_img = 0;
  Joystick::State key_img;
  while (flag) {
    if (System::Time() != time_img) {
      time_img = System::Time();
      if (joystick->GetState() != Joystick::State::kIdle) {
        Joystick::State key = joystick->GetState();
        Item item = items[focus];
        if (key != key_img) {
          key_img = key;
          flag = listen(key, DOWN);
          time_next = System::Time() + threshold;
        } else if (time_img > time_next && time_img % 10 == 0) {
          flag = listen(key, LONG);
        }

      } else {
        flag = listen(key_img, UP);
        key_img = Joystick::State::kIdle;
      }
    }

  }
  clear();
}

void DebugConsole::pushItem(Item item) {
  insertItem(item, length - 1);
}

void DebugConsole::insertItem(Item item, int index) {
  Clamp<int>(0, index, length);
  for (int i = length++; i >= index; i--)
    items[i] = items[i - 1];
  items[index] = item;
}

void DebugConsole::listItems(int start) {
  clear();
  for (int i = start; i < (length < start + 10 ? length : start + 10); i++) {
    printItem(i);
  }
  showFocus(0);
}

void DebugConsole::printItem(int index) {
  if (items[index].getValuePtr() != NULL) {
    char buff[20];
    sprintf(buff, "%s%d      ", items[index].getText(), items[index].getValue());
    printxy(1, index - topIndex, buff);
  } else
    printxy(1, index - topIndex, items[index].getText());
}

void DebugConsole::printxy(int x, int y, char* c, int l) {
  lcd->SetRegion(Lcd::Rect(x * 10, y * 15, l, 15));
  writer->WriteString(c);
}

void DebugConsole::showFocus(bool flag) {
  if (flag) writer->WriteString(" ");
  lcd->SetRegion(Lcd::Rect(0, (focus - topIndex) * 15, 10, 15));
  writer->WriteString(">");
}

void DebugConsole::clear() {
  lcd->Clear();
}

int DebugConsole::listen(Joystick::State key, int state) {
  Item item = items[focus];
  switch (key) {
    case Joystick::State::kDown:
      if (state != UP) {
        printxy(0, focus, " ", 10);
        focus = (focus + 1) % length;
        if (focus - topIndex > 8 && focus != length - 1) {
          topIndex++;
          listItems(topIndex);
        } else if (focus == 0) {
          topIndex = 0;
          listItems(topIndex);
        }
      }
      printItem(focus);
      break;
    case Joystick::State::kUp:
      if (state != UP) {
        printxy(0, focus, " ", 10);
        if (focus == 0) {
          focus = length - 1;
          if (length > 9) {
            topIndex = length - 10;
            listItems(topIndex);
          }
        } else {
          focus--;
          if (focus - topIndex < 1 && topIndex > 0) {
            topIndex--;
            listItems(topIndex);
          }
        }

      }
      printItem(focus);
      break;
    case Joystick::State::kSelect:
      if (item.getListener(SELECT + state * 4) != NULL) {
        item.getListener(SELECT + state * 4)();
        listItems(topIndex);
      } else if (item.getText() == ">>exit debug<<")
        return 0;
      break;
    case Joystick::State::kLeft:
      if (item.getListener(LEFT + state * 4) != NULL) {
        item.getListener(LEFT + state * 4)();
        listItems(topIndex);
      } else if (item.getValuePtr() != NULL && state != UP && !item.isReadOnly()) {
        item.setValue(item.getValue() - 1);
        printItem(focus);
      }

      break;
    case Joystick::State::kRight:
      if (item.getListener(RIGHT + state * 4) != NULL) {
        item.getListener(RIGHT + state * 4)();
        //listItems(topIndex);
      } else if (item.getValuePtr() != NULL && state != UP && !item.isReadOnly()) {
        item.setValue(item.getValue() + 1);
        printItem(focus);
      }

      break;
    default:
      return 1;
  }
  //showFocus(1);
  printxy(0, focus, ">", 10);
  //listItems(topIndex);
  return 1;
}

DebugConsole::Item::Item(char* text, int* value, bool readOnly)
    : text(text), value(value), readOnly(readOnly) {
  init();
}

void DebugConsole::Item::setValue(int v) {
  if (value == nullptr) return;
  *value = v;
}

int DebugConsole::Item::getValue() {
  if (value == nullptr) return 0;
  return *value;
}

void DebugConsole::Item::init() {
  for (int i = 0; i < 12; i++)
    listeners[i] = nullptr;
}
