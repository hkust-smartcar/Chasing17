#include "algorithm/king/main.h"

#include "libsc/alternate_motor.h"
#include "libsc/dir_encoder.h"
#include "libsc/futaba_s3010.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/lcd_console.h"
#include "libsc/led.h"
#include "libsc/st7735r.h"
#include "libsc/system.h"
#include "libsc/k60/ov7725.h"
#include "libsc/joystick.h"

#include "bluetooth.h"
#include "car_manager.h"
#include "algorithm/king/Moving.h"
//#include "util/mpc.h"

using namespace libsc;
using namespace libsc::k60;
using namespace std;

namespace algorithm {
namespace king {
void main(bool has_encoder) {
	// initialize LEDs
	Led::Config ledConfig;
	ledConfig.is_active_low = true;
	ledConfig.id = 0;
	Led led1(ledConfig);  // main loop
	ledConfig.id = 1;
	Led led2(ledConfig);  // unused
	ledConfig.id = 2;
	Led led3(ledConfig);  // unused
	ledConfig.id = 3;
	Led led4(ledConfig);  // unused

	led1.SetEnable(false);
	led2.SetEnable(false);
	led3.SetEnable(false);
	led4.SetEnable(false);
	// initialize camera
	Ov7725::Config cameraConfig;
	cameraConfig.id = 0;
	cameraConfig.w = 80;  // downscale the width to 80
	cameraConfig.h = 60;  // downscale the height to 60
	cameraConfig.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
	Ov7725 camera(cameraConfig);

	// initialize LCD
	St7735r::Config lcdConfig;
	lcdConfig.is_revert = true;
	//	lcdConfig.fps = 10;
	St7735r lcd(lcdConfig);

	//  // initialize LCD console
	LcdConsole::Config console_config;
	console_config.lcd = &lcd;
	LcdConsole console(console_config);

	// initialize Servo console
	FutabaS3010::Config ServoConfig;
	ServoConfig.id = 0;
	FutabaS3010 servo(ServoConfig);

	// initialize Encoder
	DirEncoder::Config EncoderConfig_1;
	EncoderConfig_1.id = 0;
	DirEncoder::Config EncoderConfig_2;
	EncoderConfig_2.id = 1;
	DirEncoder encoderA(EncoderConfig_1);
	DirEncoder encoderB(EncoderConfig_2);

	AlternateMotor::Config config;
	config.multiplier = 100;
	config.id = 0;
	AlternateMotor motor_left(config);
	config.id = 1;
	AlternateMotor motor_right(config);

	//  // initialize speed control
	//  Mpc(&encoderA, &motor_left);

	motor_left.SetClockwise(true);
	motor_right.SetClockwise(false);

	motor_left.SetPower(250);
	motor_right.SetPower(250);
	//	servo.SetDegree(StraightDegree);
	//	while (true){}

	k60::JyMcuBt106::Config bt_config;
	bt_config.id = 0;
	bt_config.baud_rate = Uart::Config::BaudRate::k115200;
	BTComm bluetooth(bt_config);

	camera.Start();
	while (!camera.IsAvailable()) {}
	//Initiate a car
	Moving car;
	Timer::TimerInt timeImg = System::Time();  // current execution time
	Timer::TimerInt startTime;  // starting time for read+copy buffer
	const Timer::TimerInt time_ms = 5;  // testing case in ms
	lcd.Clear();
	/*Testing before starting the moving*/
	servo.SetDegree(ServoLeftBoundary);
	System::DelayMs(1000);
	servo.SetDegree(ServoRightBoundary);
	System::DelayMs(1000);
	servo.SetDegree(ServoStraightDegree);
	System::DelayMs(1000);
	/*main loop*/
	while (true) {
		if (timeImg != System::Time()) {
			timeImg = System::Time();
			if ((timeImg % time_ms) == 0) {
				/*Motor Protection*/
						encoderA.Update();
						encoderB.Update();
						if(encoderA.GetCount() == 0 || encoderB.GetCount() == 0){
							motor_left.SetPower(0);
							motor_right.SetPower(0);
						}
				/*--------------------------------------------------------------record the starting time
        startTime = System::Time();
----------------------------------------------------------------*/
				startTime = System::Time();
				CarManager::Feature feature;
				const Byte* camBuffer = camera.LockBuffer();
				led1.Switch();

				car.extract_cam(camBuffer);
				//      car.printCameraImage(camBuffer, lcd);
				camera.UnlockBuffer();
				car.NormalMovingTestingVersionOld(servo, lcd, motor_right, motor_left);

				/*Servo tuning*/
//				car.ServoAngleTuning(servo, lcd);

				//Print it on LCD
				//      car.Print2Darray(led4, lcd);
				//		Timer::TimerInt timeTaken = System::Time() - startTime;
				//		string s = "Time: " + to_string(timeTaken) + "\n";
				//		console.SetCursorRow(0);
				//		console.WriteString(s.c_str());

				/*---------------------------------------------------------------------------
        Timer::TimerInt timeTaken = System::Time() - startTime;
        string s = "Time: " + to_string(timeTaken) + "\n";
        console.WriteString(s.c_str());
---------------------------------------------------------------------------*/
				//        if(timeImg %100 == 0){
				//        	bluetooth.sendSpeed(motor_left.GetPower()/10);
				//        	bluetooth.sendSlopeDeg((servo.GetDegree() - 900)/10);
				//        }
				//
				//        bluetooth.sendSpeed(motor_left.GetPower() / 10);
				//        if((feature != CarManager::Feature::kRoundabout) && (feature != CarManager::Feature::kCross)){
				//        	bluetooth.sendFeature(feature);
				//        }
				//        else{
				//        	bluetooth.sendFeature(CarManager::Feature::kStraight);
				//        }
			}
		}
	}
}
}
}
