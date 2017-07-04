
// created by Peter on 30/5/2017

#include "algorithm/distance.h"

#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_console.h"
#include "libsc/system.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/joystick.h"

#include "fc_yy_us_v4.h"
#include "util/util.h"

using namespace libsc;
using namespace std;

namespace algorithm {
void USIRDemo() {
  Led::Config config_led;
  config_led.is_active_low = true;
  config_led.id = 0;
  Led led1(config_led);
  config_led.id = 3;
  Led led4(config_led);

  FcYyUsV4 US(Pin::Name::kPtb0);

  Joystick::Config config_joystick;
  config_joystick.id = 0;
//  config_joystick.dispatcher = [&US](const uint8_t id, const Joystick::State which){US.resetFilter();};

  St7735r::Config lcd_config;
  lcd_config.is_revert = true;
  St7735r lcd(lcd_config);
  St7735r* pLcd = &lcd;
  lcd.Clear();

  LcdTypewriter::Config writerConfig;
  writerConfig.lcd = pLcd;
  LcdTypewriter writer(writerConfig);
  LcdTypewriter* pWriter = &writer;

  k60::JyMcuBt106::Config btConfig;
  btConfig.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
  btConfig.id = 0;
  k60::JyMcuBt106 bt(btConfig);



  Timer::TimerInt time = 0;

  while (true) {
    if (time != System::Time()) {
      time = System::Time();
      if (time % 10 == 0) {
        unsigned int dist = US.GetDistance();
        char temp[100];
        sprintf(temp, "%d\n", dist);
        bt.SendStr(temp);
//        pLcd->SetRegion(Lcd::Rect(0, 0, 128, 15));
//        pWriter->WriteString(temp);
        led1.Switch();
      }
    }
  }
}
}
