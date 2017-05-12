/*
 * debug_console.h
 *
 *  Created on: 2017Äê5ÔÂ11ÈÕ
 *      Author: WONG
 */

#ifndef INC_DEBUG_CONSOLE_H_
#define INC_DEBUG_CONSOLE_H_

#include <libsc/joystick.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/st7735r.h>

#define START_IDLE 	0
#define DOWN_SELECT 1
#define DOWN_LEFT 	2
#define DOWN_RIGHT 	3
//just up or end5
#define END_IDLE 	4
#define UP_SELECT 	5
#define UP_LEFT 	6
#define UP_RIGHT 	7
//down for a while
#define LONG_IDLE 	8
#define LONG_SELECT 9
#define LONG_LEFT 	10
#define LONG_RIGHT 	11

#define IDLE 	0
#define SELECT 	1
#define LEFT 	2
#define RIGHT	3

#define START 	0
#define END 	1
#define DOWN 	0
#define UP		1
#define LONG 	2

using namespace libsc;
using namespace libbase::k60;

#define GET_TIME 	System::Time()


class DebugConsole{

	public:
		typedef void(*Fptr)();
		class Item{
			public:

			//Item(char* n):text(n){init();}
			Item(char* text=NULL,int* value=NULL,bool readOnly=false):text(text),value(value),readOnly(readOnly){init();}
			//Item(){init();}

			//display text methods
			char* getText(){return text;}
			void setText(char* t){text=t;}

			//listener methods
			Fptr getListener(int type){return listeners[type];}
			Fptr getListeners(){return *listeners;}
			void setListener(int type, Fptr fptr){listeners[type]=fptr;}

			//value methods
			void setValuePtr(int* v){value=v;}
			void setValue(int v){if(value==NULL)return;*value=v;}
			int* getValuePtr(){return value;}
			int getValue(){if(value==NULL)return 0;return *value;}

			bool isReadOnly(){return readOnly;}
			void setReadOnly(bool isReadOnly){readOnly=isReadOnly;}

			private:
				Fptr listeners[12];
				char* text;
				int* value=NULL;
				bool readOnly;
				bool upLongExclusive;
				void init(){
					for(int i=0;i<12;i++)
						listeners[i]=NULL;
				}
		};
	public:


		DebugConsole(Joystick* joystick,St7735r* lcd, LcdTypewriter* writer):
			length(0),focus(0),topIndex(0),joystick(joystick),lcd(lcd),writer(writer),threshold(1000){
			Item item(">>exit debug<<");
			pushItem(item);
		}


		void enterDebug(){
			listItems();
			int flag=1,time_next,time_img=0;
			Joystick::State key_img;
			while(flag){
				if(System::Time()!=time_img){
					time_img=System::Time();
					if(joystick->GetState()!=Joystick::State::kIdle){
						Joystick::State key=joystick->GetState();
						Item item=items[focus];
						if (key!=key_img){
							key_img=key;
							flag = listen(key,DOWN);
							time_next=GET_TIME+threshold;
						}
						else if(time_img>time_next&&time_img%10==0){
							flag = listen(key,LONG);
						}

					}
					else{
						flag = listen(key_img,UP);
						key_img=Joystick::State::kIdle;
					}
				}



			}
			clear();
		}

		void pushItem(Item item){
			insertItem(item,length-1);
			//items[length++]=item;
		}

		void insertItem(Item item, int index=0){
			index = (index<0?0:(index>length?length:index));
			for(int i=length++;i>=index;i--)
				items[i]=items[i-1];
			items[index]=item;
		}

		void listItems(int start=0){
			clear();
			for(int i=start;i<(length<start+10?length:start+10);i++){
				printItem(i);
			}
			showFocus(0);
		}

		void printItem(int index){
			if(items[index].getValuePtr()!=NULL){
					char buff[20];
					sprintf(buff,"%s%d      ",items[index].getText(),items[index].getValue());
					printxy(1,index-topIndex,buff);
				}else
					printxy(1,index-topIndex,items[index].getText());
		}

	private:

		int length;
		int focus;
		int topIndex;
		Item items[50];
		Joystick* joystick;
		St7735r* lcd;
		LcdTypewriter* writer;
		int threshold;



		void printxy(int x, int y, char* c, int l=100){
			lcd->SetRegion(Lcd::Rect(x*10,y*15,l,15));
			writer->WriteString(c);
			//showFocus(0);
		}

		void showFocus(bool flag=1){
			if(flag) writer->WriteString(" ");
			lcd->SetRegion(Lcd::Rect(0,(focus-topIndex)*15,10,15));
			writer->WriteString(">");
		}

		void clear(){
			//lcd->SetRegion(Lcd::Rect(0,0,128,120));
			lcd->Clear();
		}

		int listen(Joystick::State key,int state){
			Item item=items[focus];
			switch(key){
				case Joystick::State::kDown:
					if(state!=UP){
						printxy(0,focus," ",10);
						focus=(focus+1)%length;
						if(focus-topIndex>8&&focus!=length-1){
							topIndex++;
							listItems(topIndex);
						}else if(focus==0){
							topIndex=0;
							listItems(topIndex);
						}
					}
					printItem(focus);
					break;
				case Joystick::State::kUp:
					if(state!=UP){
						printxy(0,focus," ",10);
						if (focus==0){
							focus=length-1;
							if(length>9){
								topIndex=length-10;
								listItems(topIndex);
							}
						}else{
							focus--;
							if(focus-topIndex<1&&topIndex>0){
								topIndex--;
								listItems(topIndex);
							}
						}


					}
					printItem(focus);
					break;
				case Joystick::State::kSelect:
					if(item.getListener(SELECT+state*4)!=NULL){
						item.getListener(SELECT+state*4)();
						listItems(topIndex);
					}
					else if(item.getText()==">>exit debug<<")
						return 0;
					break;
				case Joystick::State::kLeft:
					if(item.getListener(LEFT+state*4)!=NULL){
						item.getListener(LEFT+state*4)();
						listItems(topIndex);
					}
					else if(item.getValuePtr()!=NULL&&state!=UP&&!item.isReadOnly()){
						item.setValue(item.getValue()-1);
						printItem(focus);
					}

					break;
				case Joystick::State::kRight:
					if(item.getListener(RIGHT+state*4)!=NULL){
						item.getListener(RIGHT+state*4)();
						//listItems(topIndex);
					}
					else if(item.getValuePtr()!=NULL&&state!=UP&&!item.isReadOnly()){
						item.setValue(item.getValue()+1);
						printItem(focus);
					}

					break;
				default:
					return 1;
			}
			//showFocus(1);
			printxy(0,focus,">",10);
			//listItems(topIndex);
			return 1;
		}
};



#endif /* INC_DEBUG_CONSOLE_H_ */
