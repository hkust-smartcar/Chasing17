/*
 * main.cpp
 *
 * Author: Gordon
 * Copyright (c) 2014-2017 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

//BLACK is TRUE, WHITE IS FALSE
#define BLACK true
#define WHITE false

#include <cassert>
#include <cstring>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/button.h>
#include <libbase/k60/gpio.h>
#include <libbase/k60/pin.h>
#include <libsc/st7735r.h>
#include <libsc/k60/ov7725.h>
#include <libsc/lcd.h>
#include <libsc/servo.h>
#include <libsc/alternate_motor.h>
#include <libsc/motor.h>
#include <libsc/lcd_console.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/ab_encoder.h>
#include <libsc/joystick.h>
#include <libsc/futaba_s3010.h>
//#include <algorithm>

#define WIDTH 120
#define HEIGHT 90
#define AREA_KP 1.1
#define SPEEDAREA 3
#define LEFT_ANGLE 1150
#define RIGHT_ANGLE 450
#define CENTER_ANGLE 800

namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 150000;
			return config;
		}

	}
}

using namespace libsc;
using namespace libbase::k60;



int max(int a,int b){return (a>b?a:b);}
int min(int a,int b){return (a<b?a:b);}


//void analysis(const Byte* buff);
bool get_bit(const Byte* buff, int x, int y){
	if (x<0||x>=WIDTH-1||y<0||y>=HEIGHT-1) return true; //out of view is black
	return buff[y*WIDTH/8+x/8] & 0x80>>x%8;
}
bool median(const Byte* buff, int x, int y){

	int count=0,total=0;

	for(int i=max(0,x-1);i<min(WIDTH-1,x+1);i++){
		for(int j=max(0,y-1);j<min(HEIGHT-1,y+1);j++){
			total++;
			count+=get_bit(buff,i,j);
		}
	}
	return count>total/2;
}

bool get_pkbit(const Byte* buff, int x, int y){
	if (x<0||x>=WIDTH-1||y<0||y>=HEIGHT-1) return true; //out of view is black
	y=HEIGHT-1-y;
	return median(buff,x,y);
}

/*
class ImageManager{
public:
	const Byte* buff;
	ImageManager(const Byte* buff):buff(buff){
	}
	bool get_bit(int x, int y){
		return buff[y*WIDTH/8+x/8] & 0x80>>x%8;
	}
	bool get_pkbit(int x, int y){
		y=HEIGHT-1-y;
		return buff[y*WIDTH/8+x/8] & 0x80>>x%8;
	}
};
*/

//void JoyStickDebugListen(Joystick::State state);

k60::Ov7725* cam;
St7735r* lcd;
AlternateMotor* motor;
Servo* servo;
Joystick* joystickptr;
LcdTypewriter* writerptr;

int servo_target=900, servo_pos=900, motor_target = 200, motor_speed = 200;
int corner_upper_limit=100;
Joystick::State jState=Joystick::State::kIdle;

void pause(Joystick& joystick, LcdTypewriter& writer, char* buff){
	lcd->SetRegion(Lcd::Rect(0,HEIGHT+30,120,15));
	writer.WriteBuffer(buff, 12);
	System::DelayMs(300);
}

bool map[HEIGHT][WIDTH];

int main(void){
   	System::Init();

	//init LED
	Led::Config config;
	config.id=0;
	Led led1(config);
	config.id=1;
	Led led2(config);
	config.id=2;
	Led led3(config);
	config.id=3;
	Led led4(config);

	led1.SetEnable(true);
	led2.SetEnable(true);
	led3.SetEnable(true);
	led4.SetEnable(true);

/*
	//init MOTOR
	AlternateMotor::Config motor_config;
	motor_config.id=1;12003	qa
	AlternateMotor motor1(motor_config);
	motor = &motor1;
	motor->SetPower(150);
	//init servo
	Servo::Config servo_config;
	servo_config.id=0;
	servo_config.period=40000;
	servo_config.min_pos_width=910;
	servo_config.max_pos_width=2100;
	Servo servo1(servo_config);
	servo = &servo1;
	servo->SetDegree(900);
*/
	FutabaS3010::Config ConfigServo;
	ConfigServo.id=0;
	FutabaS3010 Servo(ConfigServo);
	servo = &Servo;

	//init LCD
	St7735r::Config lcd_config;
	St7735r lcd1(lcd_config);
	lcd = &lcd1;

	Lcd::Rect r(0,0,WIDTH,HEIGHT);
	lcd->SetRegion(r);

	uint32_t time_img=0;

	//LcdConsole::Config console_config;
	//console_config.lcd=&lcd1;
	//LcdConsole console(console_config);

	LcdTypewriter::Config writer_config;
	writer_config.lcd = &lcd1;
	LcdTypewriter writer(writer_config);
	writerptr=&writer;

	//init camera
	k60::Ov7725::Config cam_config;
	cam_config.id=0;
	cam_config.w=WIDTH;
	cam_config.h=HEIGHT;
	cam_config.fps = k60::Ov7725::Config::Fps::kHigh;
	k60::Ov7725 cam1(cam_config);
	cam = &cam1;

	cam->Start();

	//init encoder
	//AbEncoder::Config encoder_config;
	//encoder_config.id = 0;
	//Encoder::Initializer encoder_init(encoder_config);
	//AbEncoder encoder1(encoder_config);

	//init joystick

	Joystick::Config joystick_config;
	joystick_config.id = 0;
	joystick_config.is_active_low = true;
	Joystick joystick(joystick_config);
	Joystick::State state;
	joystickptr = &joystick;

	int left_edge[400][2], right_edge[400][2], mid[400][2], left_from[400], right_from[400];

	bool found[WIDTH][HEIGHT];
	for(int i=0;i<WIDTH;i++){
		for(int j=0; j<HEIGHT; j++){
			found[i][j]=false;
		}
	}

	left_from[0]=right_from[0]=0;

		while(!cam->IsAvailable());
		{
			const Byte* byte = cam->LockBuffer();

			//locate the base edge
			//locate the right first
			right_edge[0][0]=WIDTH-1;
			while(get_pkbit(byte,--right_edge[0][0],0)&&right_edge[0][0]>0);	//from right to left, the first white is right edge base
			left_edge[0][0]=0;
			while(get_pkbit(byte,++left_edge[0][0],0)&&left_edge[0][0]<WIDTH-1);		//from left to right, the first white is left edge base
			left_edge[0][1]=0;
			right_edge[0][1]=0;

			cam->UnlockBuffer();
			cam->Stop();
			cam->Start();
		}

	servo->SetDegree(1150);
	System::DelayMs(1000);
	servo->SetDegree(450);
	System::DelayMs(1000);
	servo->SetDegree(800);

	while(true){

		//led1.Switch();
		//led2.Switch();
		//led3.Switch();
		//led4.Switch();
		//System::DelayMs(250);
		//continue;

		//JoyStickDebugListen(joystick.GetState());

		if (System::Time()!=time_img){
			time_img=System::Time();

			if (cam->IsAvailable()&&time_img%50==0){
				//int start = System::Time();
				const Byte* byte = cam->LockBuffer();

				//print image

				//lcd->SetRegion(Lcd::Rect(0, 0, 80, 60));
				//lcd->FillColor(Lcd::kBlack);

				lcd->SetRegion(Lcd::Rect(0,0,WIDTH,HEIGHT));
				lcd->FillBits(0x0000,0xFFFF,byte,cam->GetBufferSize()*8);





				int timeStart = System::Time();

				//int left_edge[200][2], right_edge[200][2], mid[200][2];

				int error=0;

				bool find_left=true;
				bool find_right=true;

				//locate the base edge
				//locate the right first
				/*
				if(get_pkbit(byte,left_edge[0][0],left_edge[0][1]))
					while(get_pkbit(byte,++left_edge[0][0],left_edge[0][1]));
				else
					while(!get_pkbit(byte,--left_edge[0][0]-1,left_edge[0][1]));
				if(get_pkbit(byte,right_edge[0][0],right_edge[0][1]))
					while(get_pkbit(byte,--right_edge[0][0],right_edge[0][1]));
				else
					while(!get_pkbit(byte,++right_edge[0][0]+1,right_edge[0][1]));
					*/
				const int base_mid_x= (left_edge[0][0]+right_edge[0][0])/2;
				left_edge[0][0]=base_mid_x;right_edge[0][0]=base_mid_x;

				while(!get_pkbit(byte,--left_edge[0][0]-1,left_edge[0][1])){
					if(left_edge[0][0]<=0){
						if(left_edge[0][1]<HEIGHT-1){
							left_edge[0][1]++;
							left_edge[0][0]=base_mid_x;
						}
						else{
							find_left=false;
							find_right=false;
							left_edge[0][0]=base_mid_x;
						}
					}
				}
				while(!get_pkbit(byte,++right_edge[0][0]+1,right_edge[0][1])){
					if(right_edge[0][0]<=0){
						if(right_edge[0][1]<HEIGHT-1){
							right_edge[0][1]++;
							right_edge[0][0]=base_mid_x;
						}
						else{
							find_left=false;
							find_right=false;
							right_edge[0][0]=base_mid_x;
						}
					}
				}

				//declare variables for searching in directions
				const int dx[8]={ 0,-1,-1,-1, 0, 1, 1, 1};
				const int dy[8]={-1,-1, 0, 1, 1, 1, 0,-1};



				//loop for finding edge, maximum find 200 points for each edge
				for (int count=0;count<400-1;count++){
					if(left_edge[count][0]==right_edge[count][0]&&left_edge[count][1]==right_edge[count][1]) {
						lcd->SetRegion(Lcd::Rect(left_edge[count][0],left_edge[count][1],5,5));
						lcd->FillColor(Lcd::kYellow);
						break;
					}


					bool flag=2;//to break

					if(find_left){

						find_left=0;

					//x y are the last point of edge
					const int& x= left_edge[count][0];
					const int& y= left_edge[count][1];

					//if(y>HEIGHT*5/7||y<0||x<0||x>WIDTH-1)
					//	find_left=false;

					//paint it
					lcd->SetRegion(Lcd::Rect(x,HEIGHT-y-1,2,2));
					lcd->FillColor(Lcd::kRed);


					//corner
					if(x>4&&x<WIDTH-5&&y>4&&y<HEIGHT-5){

						int sum=0;
						int ratio=1;
						int total=0;
						for(int r=y-4;r<y+5;r++){
							for(int c=x-4;c<x+5;c++){
								sum+=ratio*get_pkbit(byte,c,r);
								total++;
							}
							ratio+=2;
						}

						if(sum>90&& sum<corner_upper_limit){
							lcd->SetRegion(Lcd::Rect(x,HEIGHT-y-1,5,5));
							lcd->FillColor(Lcd::kBlue);
							while(!get_pkbit(byte,left_edge[count][0],++left_edge[count][1])){
								lcd->SetRegion(Lcd::Rect(left_edge[count][0],HEIGHT-left_edge[count][1]-1,2,2));
								lcd->FillColor(Lcd::kBlue);
							}
							left_from[count]=2;
						}


					}

led1.SetEnable(1);
					//search in directions, from last direction clockwise to last direction
					for (int i=left_from[count]+1; i<left_from[count]+9;i++){

						const int j=(8+(i%8))%8;

						//char wbuff[12];
						//sprintf(wbuff,"%d:%d,%d;%d,%d",j,x,y,x+dx[j],y+dy[j]);
						//pause(joystick, writer, wbuff);
						if(!get_pkbit(byte, x+dx[j],y+dy[j])){//if the point is white, it is a new point of edge
							left_edge[count+1][0]=x+dx[j];
							left_edge[count+1][1]=y+dy[j];
							flag-=1;
							left_from[count+1]=(j+4)%8;

							find_left=1;

							//if((left_from[count+1]-left_from[count])%8>4){
							//	lcd->SetRegion(Lcd::Rect(x,HEIGHT-y-1,3,3));
							//	lcd->FillColor(Lcd::kRed);
							//}

							if(j==0)find_left=false;

							break;
						}
					}

					led1.SetEnable(0);

					if(found[x][y]){
						find_left=false;
						lcd->SetRegion(Lcd::Rect(left_edge[count][0],HEIGHT-1-left_edge[count][1],5,5));
						lcd->FillColor(Lcd::kYellow);
					}
					found[x][y]=true;
					}

					if(find_right){

						find_right=0;

					//x1 y1 are the last point of edge
					int& x1=right_edge[count][0];
					int& y1=right_edge[count][1];

					//if(y1>HEIGHT*5/7||y1<0||x1<0||x1>WIDTH-1)
						//					find_right=false;

					//paint it
					lcd->SetRegion(Lcd::Rect(x1,HEIGHT-y1-1,2,2));
					lcd->FillColor(Lcd::kPurple);

					//corner
					if(x1>4&&x1<WIDTH-5&&y1>4&&y1<HEIGHT-5){
						int sum=0;
						int ratio=1;
						int total=0;
						for(int r=y1-4;r<y1+5;r++){
							for(int c=x1-4;c<x1+5;c++){
								sum+=ratio*get_pkbit(byte,c,r);
								total++;
							}
							ratio++;
						}
						if(sum>50&&sum<60){
							lcd->SetRegion(Lcd::Rect(x1,HEIGHT-y1-1,5,5));
							lcd->FillColor(Lcd::kGreen);
							while(!get_pkbit(byte,right_edge[count][0],++right_edge[count][1])){
								lcd->SetRegion(Lcd::Rect(right_edge[count][0],HEIGHT-right_edge[count][1]-1,2,2));
								lcd->FillColor(Lcd::kGreen);
							}
							right_from[count]=6;
						}
					}

					//search in directions, from last direction anti clockwise to last direction
					for (int i=right_from[count]+7; i>=right_from[count];i--){



						const int j=i%8;
						if(!get_pkbit(byte, x1+dx[j],y1+dy[j])){//if the point is white, it is a new point of edge
							right_edge[count+1][0]=x1+dx[j];
							right_edge[count+1][1]=y1+dy[j];
							flag-=1;
							right_from[count+1]=j-4;

							find_right=1;

							if(j==0) find_right=false;

							break;
						}
					}
					if(found[x1][y1]){
						find_right=false;
						lcd->SetRegion(Lcd::Rect(right_edge[count][0],HEIGHT-1-right_edge[count][1],5,5));
						lcd->FillColor(Lcd::kWhite);}
					found[x1][y1]=true;
					}

					//find the mid point
					/*
					if (y>0&&y1>0){
						lcd->SetRegion(Lcd::Rect((x1+x)/2,HEIGHT-1-(y1+y)/2,1,1));
						mid[count][0]=(x1+x)/2;
						mid[count][1]=(y1+y)/2;
						error+=WIDTH/2-mid[count][0];
						lcd->FillColor(Lcd::kBlue);
					}*/


					//break condition: left edge right edge meet or fail to find new edge
					if(left_edge[count+1][0]==right_edge[count+1][0]&&left_edge[count+1][1]==right_edge[count+1][1]) break;
					//if(flag==0)break;
					if(!(find_left||find_right))break;
					//if(System::Time()-timeStart>200)break;
				}

				//print variables
				char buff[10];
				sprintf(buff," %d,%d",System::Time()-timeStart,error);
				lcd->SetRegion(Lcd::Rect(0,HEIGHT,100,15));
				writer.WriteBuffer(buff,10);
				cam->UnlockBuffer();
				//cam->Stop();
				//cam->Start();

				for(int i=0;i<WIDTH;i++){
					for(int j=0; j<HEIGHT;j++){
						found[i][j]=false;
					}
				}

				if(jState!=joystick.GetState()){
					jState=joystick.GetState();
					if(jState==Joystick::State::kLeft)corner_upper_limit--;
					else if(jState==Joystick::State::kRight)corner_upper_limit++;

					sprintf(buff," %d ;",corner_upper_limit);
					lcd->SetRegion(Lcd::Rect(0,HEIGHT+15,100,15));
					writer.WriteBuffer(buff,10);
				}

			}




		}
	}

	return 0;
}
