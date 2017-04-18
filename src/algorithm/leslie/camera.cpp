/*
 * main.cpp
 *
 * Author: Leslie
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include "../../../inc/algorithm/leslie/camera.h"
#include <cassert>
#include <cstring>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>

#include <functional>

//Looper Header File----------------------------------------------------------------------------------------------
//#include <libutil/looper.h>

//led Header File-----------------------------------------------------------------------------------------------------------
//#include <libsc/led.h>

//LCD Header File-----------------------------------------------------------------------------------------------------------
//#include<libsc/st7735r.h>
//#include<libsc/lcd_console.h>

//Camera Header File--------------------------------------------------------------------------------------------------------
//#include<libsc/k60/ov7725.h>

//namespace-----------------------------------------------------------------------------------------------------------------
using namespace libsc;
using namespace libsc::k60;
using namespace libbase::k60;
using namespace libutil;
using namespace std;

//Global variable-----------------------------------------------------------------------------------------------------------
const Uint CamHeight=60;
const Uint CamWidth=80;
const Uint MotorPower=25;
const Uint MotorStSpeed=25;
const Uint MotorSlowDownPower=25;
#define ServoConstant 1.2;
#define MotorConstant 0.001;

//Change Camera Buffer from 1D array to 2D array----------------------------------------------------------------------------
//row is row number
//col is column number
//CamByte is the Byte CameraBuffer
bool Cam2DArray[CamHeight][CamWidth];
void Camera2DConverter(const Byte* CameraBuffer){
	Byte CamByte;
	for(Uint row=0;row<CamHeight;row++){
		for(Uint col=0;col<CamWidth;col+=8){
			CamByte=CameraBuffer[row*CamWidth/8+col/8];
			Cam2DArray[row][col]=CamByte&0x80;
			Cam2DArray[row][col+1]=CamByte&0x40;
			Cam2DArray[row][col+2]=CamByte&0x20;
			Cam2DArray[row][col+3]=CamByte&0x10;
			Cam2DArray[row][col+4]=CamByte&0x08;
			Cam2DArray[row][col+5]=CamByte&0x04;
			Cam2DArray[row][col+6]=CamByte&0x02;
			Cam2DArray[row][col+7]=CamByte&0x01;
		}
	}
}

//Image filter (Median Filter)----------------------------------------------------------------------------------------------
//row is row number
//col is column number
void CameraFilter(){
	for(Uint row=1;row<(CamHeight-1);row++){
		for(Uint col=1;col<(CamWidth-1);col++){
			Uint temp=0;
			temp=Cam2DArray[row-1][col-1]+Cam2DArray[row-1][col]+Cam2DArray[row-1][col+1]
			     +Cam2DArray[row][col-1]+Cam2DArray[row][col]+Cam2DArray[row][col+1]
				 +Cam2DArray[row+1][col-1]+Cam2DArray[row+1][col]+Cam2DArray[row+1][col+1];
			if(temp>4){
				Cam2DArray[row][col]=1;
			}else{
				Cam2DArray[row][col]=0;
			}
		}
	}
}

//Center Line finding function------------------------------------------------------------------------------------
void CenterLine(){
	int MidPt=CamWidth/2;
	int LeftEdge=0;
	int RightEdge=0;
	int RowClear=0;
	int ClearRowHeight=0;
	bool PrevRowClear=false;
	for(int y=CamHeight-2;y>-1;y--){
		bool LeftClear=false;
		bool RightClear=false;
		if(Cam2DArray[y][MidPt]==1){
			break;
		}
		for(int x=MidPt;x>-1;x--){
			if(Cam2DArray[y][x]==1){
				LeftEdge=x+1;
				break;
			}
			if(x==0){
				LeftEdge=0;
				LeftClear=true;
				break;
			}
		}
		for(int x=MidPt;x<CamWidth;x++){
			if(Cam2DArray[y][x]==1){
				RightEdge=x-1;
				break;
			}
			if(x==CamWidth-1){
				RightEdge=CamWidth-1;
				RightClear=true;
				break;
			}
		}
		MidPt=(RightEdge+LeftEdge)/2;
		Cam2DArray[y][MidPt]=2;
		if(LeftClear&&!RightClear){
			ClearRowHeight=y;
			for(int i=y;i>0;i--){
				if(Cam2DArray[i][0]){
					ClearRowHeight=i-ClearRowHeight;
					break;
				}
			}
		}else if(!LeftClear&&RightClear){
			ClearRowHeight=y;
			for(int i=y;i>0;i--){
				if(Cam2DArray[i][CamWidth-1]){
					ClearRowHeight=i-ClearRowHeight;
					break;
				}
			}
		}
		if(LeftClear!=true||RightClear!=true){
			PrevRowClear=false;
		}else if(LeftClear==true&&RightClear==true&&PrevRowClear==true){
			PrevRowClear=true;
			RowClear++;
		}else if(LeftClear==true&&RightClear==true){
			PrevRowClear=true;
		}
		if(RowClear>1&&LeftClear==true&&RightClear==true){
			Cam2DArray[y][MidPt]=3;
		}
	}
}

//void scanHorizontial(int col,int row,const Byte *camBuffer){
//	int leftEdgeX=col;
//	bool notFind=true;
//	for(int y=col;y>=0;y++){
//		for(int x=leftEdgeX+3;x>3;x--){
//			notFind=true;
//			if(camPointCheck(y,x,camBuffer)){
//				leftEdgeX=x;
//				notFind=false;
//				break;
//			}
//		}
//		if(notFind){
//			scanVertical(leftEdgeX,y,camBuffer);
//		}
//	}
//}
//
//void scanVertical(int col,int row,const Byte *camBuffer){
//
//}

int camPointCheck(int row,int col,const Byte *camBuffer){
	bool output;
	Byte camByte=camBuffer[col/8+row*CamWidth/8];
	switch (col%8){
	case 0:
		output=camByte&0x80;
		break;
	case 1:
		output=camByte&0x40;
		break;
	case 2:
		output=camByte&0x20;
		break;
	case 3:
		output=camByte&0x10;
		break;
	case 4:
		output=camByte&0x08;
		break;
	case 5:
		output=camByte&0x04;
		break;
	case 6:
		output=camByte&0x02;
		break;
	case 7:
		output=camByte&0x01;
		break;
	}
	if(output){
		return 1;
	}else{
		return 0;
	}
}

int featureIdentify(const Byte *camBuffer){
	int rightEdge=CamWidth-1;
	int leftEdge=0;
	int midPt=CamWidth/2;
	int trackWidth;
	int prevTrackWidth;
	bool isCrossOrRoundabout=false;
	bool isRoundabout=false;
	bool isCross=false;
	for(int x=CamWidth-1;x>0;x--){
		if(!camPointCheck(CamHeight-2,x,camBuffer)){
			rightEdge=x;
			break;
		}
	}
	for(int x=0;x<CamWidth;x++){
		if(!camPointCheck(CamHeight-2,x,camBuffer)){
			leftEdge=x;
			break;
		}
	}
	trackWidth=rightEdge-leftEdge;
	for(int y=CamWidth-3;y>0;y--){
		prevTrackWidth=trackWidth;
		if(camPointCheck(y,rightEdge,camBuffer)){
			for(int x=rightEdge;x>0;x--){
				if(!camPointCheck(y,x,camBuffer)){
					rightEdge=x;
					break;
				}
			}
		}else{
			for(int x=rightEdge;x<CamWidth;x++){
				if(camPointCheck(y,x,camBuffer)){
					rightEdge=x-1;
					break;
				}
			}
		}
		if(camPointCheck(y,leftEdge,camBuffer)){
			for(int x=leftEdge;x<CamWidth;x++){
				if(!camPointCheck(y,x,camBuffer)){
					leftEdge=x;
					break;
				}
			}
		}else{
			for(int x=leftEdge;x>0;x--){
				if(camPointCheck(y,x,camBuffer)){
					rightEdge=x+1;
					break;
				}
			}
		}
		trackWidth=rightEdge-leftEdge;
		if(trackWidth>prevTrackWidth){
			isCrossOrRoundabout=true;
		}else if(isCrossOrRoundabout){
			int count=0;
			for(int x=leftEdge;x<rightEdge;x++){
				if(camPointCheck(y,x,camBuffer)!=camPointCheck(y,x+1,camBuffer)){
					count++;
				}
				if(count==2){
					isRoundabout=true;
					return 1;
					break;
				}else{
					isCross=true;
					return 2;
					break;
				}
			}
		}
		if(camPointCheck(y,(rightEdge+leftEdge)/2,camBuffer)){
			//return 0;
			break;
		}
	}
	return 0;
}

void resetCam2DArray(){
	for(int y=0;y<CamHeight;y++){
		for(int x=0;x<CamWidth;x++){
			Cam2DArray[y][x]=0;
		}
	}
}

bool roundabout=false;

//void EdgeFinder(const Byte *camBuffer,St7735r *lcd,FutabaS3010 *servo){
//int midAngle=800;
//#define servoAngle 400;
//	int error=0;
//	int rowFinished=0;
//	int leftEdge=0;
//	int prevLeftEdge=0;
//	int leftCornerX;
//	int leftCornerY;
//	bool likelyLeftCorner=false;
//	bool findLeftCorner=false;
//	int rightEdge=CamWidth-1;
//	int prevRightEdge=CamWidth-1;
//	int rightCornerX;
//	int rightCornerY;
//	bool likelyRightCorner=false;
//	bool findRightCorner=false;
//	int midPt=CamWidth/2;
//	bool isRoundabout=false;
//	for(int y=CamHeight-1;y>=0;y--){
//		prevLeftEdge=leftEdge;
//		if(camPointCheck(y,leftEdge,camBuffer)){
//			for(int x=leftEdge;x<CamWidth;x++){
//				leftEdge=x;
//				if(!camPointCheck(y,leftEdge,camBuffer)){
//					leftEdge--;
//					break;
//				}
//			}
//		}else{
//			for(int x=leftEdge;x>=0;x--){
//				leftEdge=x;
//				if(camPointCheck(y,leftEdge,camBuffer)){
//					break;
//				}
//			}
//		}
//		if(leftEdge<prevLeftEdge&&!likelyLeftCorner){
//			likelyLeftCorner=true;
//			leftCornerX=prevLeftEdge;
//			leftCornerY=y-1;
//		}else if(leftEdge<=leftCornerX&&likelyLeftCorner){
//			findLeftCorner=true;
//		}else if(leftCornerX>leftEdge&&likelyLeftCorner&&!findLeftCorner){
//			likelyLeftCorner=false;
//		}
//		prevRightEdge=rightEdge;
//		if(camPointCheck(y,rightEdge,camBuffer)){
//			for(int x=rightEdge;x>=0;x--){
//				rightEdge=x;
//				if(!camPointCheck(y,rightEdge,camBuffer)){
//					rightEdge++;
//					break;
//				}
//			}
//		}else{
//			for(int x=rightEdge;x<CamWidth;x++){
//				rightEdge=x;
//				if(camPointCheck(y,rightEdge,camBuffer)){
//					break;
//				}
//			}
//		}
//		midPt=(rightEdge+leftEdge)/2;
//		if(camPointCheck(y,midPt,camBuffer)){
//			break;
//		}
//		error+=CamWidth/2-midPt;
//		rowFinished++;
//		lcd->SetRegion(Lcd::Rect(midPt,y,1,1));
//		lcd->FillColor(Lcd::kGreen);
//		if(rightEdge>prevRightEdge&&!likelyRightCorner){
//			likelyRightCorner=true;
//			rightCornerX=prevRightEdge;
//			rightCornerY=y-1;
//		}else if(rightEdge>=rightCornerX&&likelyRightCorner){
//			findRightCorner=true;
//		}else if(rightCornerX>rightEdge&&likelyRightCorner&&!findRightCorner){
//			likelyRightCorner=false;
//		}
//		if(findLeftCorner&&findRightCorner&&!roundabout){
//			if(((rightCornerY-leftCornerY)>5||(rightCornerY-leftCornerY)<-5)||((rightCornerX-leftCornerX)<5&&(rightCornerX-leftCornerX)>-5)){
//				findLeftCorner=false;
//				findRightCorner=false;
//				likelyLeftCorner=false;
//				likelyRightCorner=false;
//			}
//		}
//		if(findLeftCorner&&findRightCorner){
//			lcd->SetRegion(Lcd::Rect(rightCornerX,y,5,5));
//			lcd->FillColor(Lcd::kRed);
//			lcd->SetRegion(Lcd::Rect(leftCornerX,y,5,5));
//			lcd->FillColor(Lcd::kRed);
//			for(int y=(rightCornerY+leftCornerY)/2;y>=0;y--){
//				bool likelyRoundabout=false;
//				if(camPointCheck(y,(rightCornerX+leftCornerX)/2,camBuffer)){
//					for(int x=(rightCornerX+leftCornerX)/2;x>=0;x--){
//						if(!camPointCheck(y,x,camBuffer)){
//							likelyRoundabout=true;
//							lcd->SetRegion(Lcd::Rect(x,y,5,5));
//							lcd->FillColor(Lcd::kGreen);
//							break;
//						}
//					}
//					if(!likelyRoundabout){
//						break;
//					}
//					if(likelyRoundabout){
//						for(int x=(rightCornerX+leftCornerX)/2;x<CamWidth;x++){
//							if(!camPointCheck(y,x,camBuffer)){
//								isRoundabout=true;
//								leftEdge=x-1;
//								lcd->SetRegion(Lcd::Rect(x,y,5,5));
//								lcd->FillColor(Lcd::kGreen);
//								break;
//							}
//						}
//					}
//					break;
//				}
//			}
//			if(isRoundabout){
//				lcd->SetRegion(Lcd::Rect(0,61,50,50));
//				lcd->FillColor(Lcd::kBlue);
//				for(int x=leftEdge+1;x<CamWidth;x++){
//					rightEdge=x;
//					if(camPointCheck(y,x,camBuffer)){
//						break;
//					}
//				}
//				midPt=(rightEdge+leftEdge)/2;
//				rowFinished=1;
////				int slope=(y-(rightCornerY+leftCornerY)/2)/((leftEdge+rightEdge)/2-midPt);
////				rowFinished=0;
////				for(int row=(rightCornerY+leftCornerY)/2+1;row>y;row--){
////					midPt=((row-y)/slope)+(leftEdge+rightEdge)/2;
////					error+=CamWidth/2-midPt;
////					rowFinished++;
////					lcd->SetRegion(Lcd::Rect(midPt,row,1,1));
////					lcd->FillColor(Lcd::kGreen);
////				}
//				roundabout=true;
//			}else{
//				lcd->SetRegion(Lcd::Rect(0,61,50,50));
//				lcd->FillColor(Lcd::kWhite);
//				break;
//			}
//			break;
////		}else if(roundabout&&findRightCorner){
////			roundabout=false;
////			servo->SetDegree(midAngle-200);
////			return;
//		}else if(leftEdge==0&&rightEdge!=CamWidth-1){
//			for(int row=y;y>0;y--){
//				if(camPointCheck(row,0,camBuffer)){
//					error+=(CamWidth/2)*(row-y);
//					rowFinished+=row-y;
//					break;
//				}
//			}
//			break;
//		}else if(rightEdge==CamWidth-1&&leftEdge!=0){
//			for(int row=y;y>0;y--){
//				if(camPointCheck(row,0,camBuffer)){
//					error+=(-CamWidth/2)*(row-y);
//					rowFinished+=row-y;
//					break;
//				}
//			}
//			break;
//		}
//	}
//	int degree=midAngle+error/rowFinished/6*servoAngle;
//	if(degree>1190){
//		degree=1190;
//	}else if(degree<390){
//		degree=390;
//	}
//	servo->SetDegree(degree);
//	return;
//}

void EdgeFinder(const Byte *camBuffer,St7735r *lcd,FutabaS3010 *servo){
int midAngle=800;
#define servoAngle 400;
	int error=0;
	int rowFinished=0;
	int leftEdge=0;
	int prevLeftEdge=0;
	int leftCornerX;
	int leftCornerY;
	bool likelyLeftCorner=false;
	bool findLeftCorner=false;
	int rightEdge=CamWidth-1;
	int prevRightEdge=CamWidth-1;
	int rightCornerX;
	int rightCornerY;
	bool likelyRightCorner=false;
	bool findRightCorner=false;
	int midPt=CamWidth/2;
	int prevMidPt=CamWidth/2;
	bool isRoundabout=false;
	for(int y=CamHeight-1;y>=0;y--){
		if(camPointCheck(y,leftEdge,camBuffer)){
			for(int x=leftEdge;x<CamWidth;x++){
				leftEdge=x;
				if(!camPointCheck(y,leftEdge,camBuffer)){
					leftEdge--;
					break;
				}
			}
		}else{
			for(int x=leftEdge;x>=0;x--){
				leftEdge=x;
				if(camPointCheck(y,leftEdge,camBuffer)){
					break;
				}
			}
		}
		if(leftEdge==CamWidth-1){
			break;
		}
		if(camPointCheck(y,rightEdge,camBuffer)){
			for(int x=rightEdge;x>=0;x--){
				rightEdge=x;
				if(!camPointCheck(y,rightEdge,camBuffer)){
					rightEdge++;
					break;
				}
			}
		}else{
			for(int x=rightEdge;x<CamWidth;x++){
				rightEdge=x;
				if(camPointCheck(y,rightEdge,camBuffer)){
					break;
				}
			}
		}
		if(rightEdge==0){
			break;
		}
		if((leftEdge!=0&&rightEdge!=CamWidth-1)){
			midPt=(rightEdge+leftEdge)/2;
			error+=CamWidth/2-5-midPt;
			rowFinished++;
			lcd->SetRegion(Lcd::Rect(midPt,y,1,1));
			lcd->FillColor(Lcd::kGreen);
		}else if(leftEdge==0&&rightEdge!=CamWidth-1&&y>CamHeight/2){
			midPt=rightEdge-y/2;
			error+=CamWidth/2-5-midPt;
			rowFinished++;
			lcd->SetRegion(Lcd::Rect(midPt,y,1,1));
			lcd->FillColor(Lcd::kGreen);
		}else if(leftEdge!=0&&rightEdge==CamWidth-1&&y>CamHeight/2){
			midPt=leftEdge+y/2;
			error+=CamWidth/2-5-midPt;
			rowFinished++;
			lcd->SetRegion(Lcd::Rect(midPt,y,1,1));
			lcd->FillColor(Lcd::kGreen);
		}else{
			midPt=(rightEdge+leftEdge)/2;
			if(camPointCheck(y,midPt,camBuffer)){
				for(int row=y;row>5;row--){
					if(!camPointCheck(row,midPt,camBuffer)){
						roundabout=true;
						isRoundabout=true;
						break;
					}else{
						isRoundabout=false;
					}
				}
			}
			if(camPointCheck(y,midPt,camBuffer)&&roundabout){
				for(int x=midPt;x>0;x--){
					if(!camPointCheck(y,x,camBuffer)){
						rightEdge=x-1;
						break;
					}
				}
				lcd->SetRegion(Lcd::Rect(rightEdge,y,5,5));
				lcd->FillColor(Lcd::kRed);
				for(int x=rightEdge;x>0;x--){
					leftEdge=x;
					if(camPointCheck(y,leftEdge,camBuffer)){
						break;
					}
				}
				lcd->SetRegion(Lcd::Rect(leftEdge,y,5,5));
				lcd->FillColor(Lcd::kRed);
				midPt=(rightEdge+leftEdge)/2;
				lcd->SetRegion(Lcd::Rect(midPt,y,5,5));
				lcd->FillColor(Lcd::kRed);
				error=CamWidth/2-5-midPt;
				rowFinished=1;
				if(!isRoundabout){
					roundabout=false;
				}
				break;
			}
		}
	}
	int degree=midAngle+error/rowFinished/5*servoAngle;
	if(degree>1180){
		degree=1180;
	}else if(degree<420){
		degree=420;
	}
	servo->SetDegree(degree);
	return;
}

int midPtFinder(const Byte *camBuffer){
	int midPt=CamWidth/2;
	int leftEdge=0;
	int rightEdge=CamWidth-1;
	int trackWidth=CamWidth;
	for(int y=CamHeight-5;y>=30;y--){
		for(int x=midPt;x>0;x--){
			leftEdge=x;
			if(camPointCheck(y,leftEdge,camBuffer)){
				break;
			}
		}
		for(int x=midPt;x<CamWidth;x++){
			rightEdge=x;
			if(camPointCheck(y,rightEdge,camBuffer)){
				break;
			}
		}
		if(trackWidth-rightEdge+leftEdge>10){
			return midPt;
		}
	}
	return 0;
}

bool isRoundabout(const Byte *camBuffer,int row,int midPt){
//	loop the upper bound of the roundabout island
	for(int y=row;y>-1;y--){
//		check if reaching the track (= find the upper bound of the roundabout island)
		if(!camPointCheck(y,midPt,camBuffer)){
//			loop from the bottom to the upper bound of the roundabout island
			for(int i=row;i>row;i--){
//				loop leftward from midPt
				for(int x=midPt;x>-1;x--){
//					if track find then passed the test, if not find the track at camera boundary, it is not a roundabout
					if(!camPointCheck(i,x,camBuffer)){
						break;
					}else if(x==0){
						return false;
					}
				}
//				loop rightward from midPt
				for(int x=midPt;x<CamWidth;x++){
//					if track find then passed the test, if not find the track at camera boundary, it is not a roundabout
					if(!camPointCheck(i,x,camBuffer)){
						break;
					}else if(x==0){
						return false;
					}
				}
			}
//			as it passes all the test, so it is a roundabout
			return true;
		}
	}
//	since cannot find the upper bound of the roundabout island, it is not roundabout
	return false;
}

bool roundaboutTurnLeft=true;

void moveAlgo(const Byte *camBuffer,St7735r *lcd,FutabaS3010 *servo){
//	int midAngle=800;
//	const int servoP=20;
	int midAngle=710;
	const int servoP=20;
	int rightEdge=CamWidth/2;
	int prevRightEdge=CamWidth/2;
	bool rightCornerFind=false;
	int leftEdge=CamWidth/2;
	int prevLeftEdge=CamWidth/2;
	bool leftCornerFind=false;
	int sum=0;
	int rowFinded=0;
//	initialize bottom left edge and right edge
	for(int x=leftEdge;x>=0;x--){
		leftEdge=x;
		if(camPointCheck(CamHeight-1,leftEdge,camBuffer)){
			break;
		}
	}
	for(int x=rightEdge;x<CamWidth;x++){
		rightEdge=x;
		if(camPointCheck(CamHeight-1,rightEdge,camBuffer)){
			break;
		}
	}
//	find left edge and right edge
	for(int y=CamHeight-1;y>=30;y--){
		prevLeftEdge=leftEdge;
		if(camPointCheck(y,leftEdge,camBuffer)){
			for(int x=leftEdge;x<CamWidth;x++){
				leftEdge=x;
				if(!camPointCheck(y,leftEdge,camBuffer)){
					leftEdge--;
					break;
				}
			}
		}else{
			for(int x=leftEdge;x>=0;x--){
				leftEdge=x;
				if(camPointCheck(y,leftEdge,camBuffer)){
					break;
				}
			}
		}
		if(leftEdge==CamWidth-1){
			break;
		}
//		check if facing a left corner, if yes the left edge use prev left edge
		if(abs(leftEdge-prevLeftEdge)>5){
			lcd->SetRegion(Lcd::Rect(leftEdge,y,5,5));
			lcd->FillColor(Lcd::kRed);
			leftEdge=prevLeftEdge;
			leftCornerFind=true;
		}
		prevRightEdge=rightEdge;
		if(camPointCheck(y,rightEdge,camBuffer)){
			for(int x=rightEdge;x>=0;x--){
				rightEdge=x;
				if(!camPointCheck(y,rightEdge,camBuffer)){
					rightEdge++;
					break;
				}
			}
		}else{
			for(int x=rightEdge;x<CamWidth;x++){
				rightEdge=x;
				if(camPointCheck(y,rightEdge,camBuffer)){
					break;
				}
			}
		}
		if(rightEdge==0){
			break;
		}
//		check if facing a right corner, if yes the right edge use prev right edge
		if(abs(rightEdge-prevRightEdge)>5){
			lcd->SetRegion(Lcd::Rect(rightEdge,y,2,2));
			lcd->FillColor(Lcd::kRed);
			rightEdge=prevRightEdge;
			rightCornerFind=true;
		}
//		check if whole row are track, if yes mean car may be on cross road and change edge finding method(search from mid point)
		if(y>CamHeight-7&&rightEdge==CamWidth-1&&leftEdge==0){
			sum=midPtFinder(camBuffer);
			rowFinded=1;
			break;
		}
//		check if left corner and right corner appear, if yes stop searching upper row (as near cross road, upper data is useless)
		if(leftCornerFind&&rightCornerFind){
			break;
		}else{
//			check if the mid point is not track, if yes it likely be roundabout or likely be exit of roundabout
			if(camPointCheck(y,(leftEdge+rightEdge)/2,camBuffer)){
//				check is it really a roundabout
				if(isRoundabout(camBuffer,y,(leftEdge+rightEdge)/2)){
//					as it is in front of the roundabout, use area method to identify which is the shortest path to go through roundabout
					int leftArea=0;
					int rightArea=0;
					for(int row=y;y>10;y--){
						if(camPointCheck(row,CamWidth/2,camBuffer)){
							break;
						}
						for(int col=CamWidth/2-1;col>0;col--){
							leftArea+=!camPointCheck(row,col,camBuffer);
						}
						for(int col=CamWidth/2;col<CamWidth;col++){
							rightArea+=!camPointCheck(row,col,camBuffer);
						}
					}
//					when right track area is larger than left, exit of roundabout is at right side, so it should not turn left
					if(rightArea>leftArea){
						roundaboutTurnLeft=false;
					}
				}
				if(roundaboutTurnLeft){
					for(int x=(leftEdge+rightEdge)/2;x>0;x--){
						if(!camPointCheck(y,x,camBuffer)){
							rightEdge=x;
							break;
						}
					}
					lcd->SetRegion(Lcd::Rect(rightEdge,y,2,2));
					lcd->FillColor(Lcd::kGreen);
					for(int x=rightEdge;x>=0;x--){
						leftEdge=x;
						if(camPointCheck(y,leftEdge,camBuffer)){
							break;
						}
					}
					lcd->SetRegion(Lcd::Rect(leftEdge,y,2,2));
					lcd->FillColor(Lcd::kGreen);
					sum=(rightEdge+leftEdge)/2;
					lcd->SetRegion(Lcd::Rect((leftEdge+rightEdge)/2,y,1,1));
					lcd->FillColor(Lcd::kRed);
					rowFinded=1;
					break;
				}else{
					for(int x=(leftEdge+rightEdge)/2;x<CamWidth;x++){
						if(!camPointCheck(y,x,camBuffer)){
							leftEdge=x;
							break;
						}
					}
					lcd->SetRegion(Lcd::Rect(leftEdge,y,2,2));
					lcd->FillColor(Lcd::kGreen);
					for(int x=leftEdge;x<CamWidth;x++){
						rightEdge=x;
						if(camPointCheck(y,rightEdge,camBuffer)){
							break;
						}
					}
					lcd->SetRegion(Lcd::Rect(rightEdge,y,2,2));
					lcd->FillColor(Lcd::kGreen);
					sum=(rightEdge+leftEdge)/2;
					lcd->SetRegion(Lcd::Rect((leftEdge+rightEdge)/2,y,1,1));
					lcd->FillColor(Lcd::kRed);
					rowFinded=1;
					break;
				}
			}
//			use right edge to find the midPt when only right edge found
			if(leftEdge==0&&rightEdge<CamWidth){
				sum+=rightEdge-y/2;
				lcd->SetRegion(Lcd::Rect(rightEdge-y/2,y,1,1));
				lcd->FillColor(Lcd::kRed);
				rowFinded++;
//			use left edge to find the midPt when only left edge found
			}else if(leftEdge>=0&&rightEdge==CamWidth){
				sum+=leftEdge+y/2;
				lcd->SetRegion(Lcd::Rect(leftEdge-y/2,y,1,1));
				lcd->FillColor(Lcd::kRed);
				rowFinded++;
//			normal method to find the midPt
			}else{
				sum+=(leftEdge+rightEdge)/2;
				lcd->SetRegion(Lcd::Rect((leftEdge+rightEdge)/2,y,1,1));
				lcd->FillColor(Lcd::kRed);
				rowFinded++;
			}
		}
	}
//	find the average midPt found
	int average=sum/rowFinded;
//	determine the servo degree
	int degree=midAngle-(average-40)*servoP;
//	prevent servo degree too large or too small
	if(degree>1050){
		degree=1050;
	}else if(degree<410){
		degree=410;
	}
	servo->SetDegree(degree);
}

void CameraPrint(St7735r *lcd,Ov7725 *Cam){
	lcd->SetRegion(Lcd::Rect(1,1,80,60));
	lcd->FillBits(0x001F,0xFFFF,Cam->LockBuffer(),Cam->GetBufferSize()*8);
}

void Camera2DArrayPrint(St7735r *lcd){
	for(Uint y=0;y<CamHeight;y++){
		for(Uint x=0;x<CamWidth;x++){
			lcd->SetRegion(Lcd::Rect(x,y,1,1));
			if(Cam2DArray[y][x]==0){
				lcd->FillColor(0xFFFF);
			}else if(Cam2DArray[y][x]==1){
				lcd->FillColor(0x001F);
			}
		}
	}
}

void Camera2DArrayPrintTest(St7735r *lcd,Ov7725 *Cam){
	for(Uint y=0;y<CamHeight;y++){
		for(Uint x=0;x<CamWidth;x++){
			lcd->SetRegion(Lcd::Rect(x,y,1,1));
			if(camPointCheck(y,x,Cam->LockBuffer())==0){
				lcd->FillColor(0xFFFF);
			}else if(camPointCheck(y,x,Cam->LockBuffer())==1){
				lcd->FillColor(0x001F);
			}
		}
	}
}
