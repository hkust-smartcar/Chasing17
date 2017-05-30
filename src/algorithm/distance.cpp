// created by Peter on 30/5/2017

#include "algorithm/distance.h"
#include "fc_yy_us_v4.h"

using namespace libsc;
using namespace std;


using namespace libsc;
using namespace std;

namespace algorithm {
void BluetoothDemo(bool has_encoder) {
  Led::Config config_led;
  config_led.is_active_low = true;
  config_led.id = 0;
  Led led1(config_led);

  FcYyUsV4 US(Pin::Name::kPtb0);

  St7735r::Config lcd_config;
  lcd_config.is_revert = true;
  St7735r lcd(lcd_config);
  lcd.Clear();

  LcdConsole::Config console_config;
  console_config.lcd = &lcd;
  LcdConsole console(console_config);

  while (true) {
    if (time != System::Time()) {
      time = System::Time();
      if (time % 100 == 0) {
		  US.GetDistance();
    	  led1.Switch();
      }
    }
  }
}
}
