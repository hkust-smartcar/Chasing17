/*
 * Moving.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: Mr.King
 */
#include "algorithm/king/Moving.h"

//SELF_DEFINED VARIABLES in .cpp

//int left_corner;// return the layer of left corner
//bool right_corner;// return the layer of right corner
//int L_sum_white = 0;
//int R_sum_white = 0;




Status Moving::RoadSituation() {
	int layer = 55;
	bool R_Edge_is_found;
	bool L_Edge_is_found;
	// Find the origin (y=1). (Assume the original layer is accurate every time)-------------------------------------------
	for (int x = W / 2; x > 1; x--) {
		if (ext_camptr[x][layer] != ext_camptr[x - 1][layer]) {
			Left_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	for (int x = W / 2; x < W; x++) {
		if (ext_camptr[x][layer] != ext_camptr[x + 1][layer]) {
			Right_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	// Update the center point
	Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
	//ext_camptr[Center[layer]][layer] = 3;
	// Find the rest (Choose the start point based on last layer midpoint)-------------------------------------------
	for (; --layer > 30;) {
		R_Edge_is_found = false;
		L_Edge_is_found = false;
		//LEFT
		for (int x = Center[layer + 1]; x > 1; x--) {
			if (ext_camptr[x][layer] != ext_camptr[x - 1][layer]) {
				Left_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				L_Edge_is_found = true;
				break;
			}
		}
		if (L_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Left_edge[layer] = Left_edge[layer + 1];
			//ext_camptr[Left_edge[layer]][layer] = 2;
		}
		//Right
		for (int x = Center[layer + 1]; x < W; x++) {
			if (ext_camptr[x][layer] != ext_camptr[x + 1][layer]) {
				Right_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				R_Edge_is_found = true;
				break;
			}
		}
		if (R_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Right_edge[layer] = Right_edge[layer + 1];
			ext_camptr[Right_edge[layer]][layer] = 2;
		}
		// Status judgement---------------------------------------------------------------------------------------------------------------------
		if (!HasCorner(layer, L_Edge_is_found, R_Edge_is_found)) {
			//Normal situation
			if (HasRoad()) {
				Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
				//ext_camptr[Center[layer]][layer] = 3;
				return Status::kNormal;
			}
			//Exit of round road or Sshape
			else {
				return Status::kRoundOut;
			}
		} else {
			//Crossing
			if (HasRoad()) {
				return Status::kCrossing;
			}
			//Entrance of round road
			else {
				return Status::kRoundIn;
			}
		}
	}
	return Status::Fail;
}

void Moving::NormalMovingTestingVersion1(FutabaS3010& servo, St7735r& lcd, CarManager::ServoBounds s) {
	//	// initialize LCD console
	//	LcdConsole::Config console_config;
	//	console_config.lcd = &lcd;
	//	LcdConsole console(console_config);
	int layer = 55;
	int LayerCount = 0;
	bool R_Edge_is_found;
	bool L_Edge_is_found;
	// Find the origin (y=1). (Assume the original layer is accurate every time)-------------------------------------------
	for (int x = W / 2; x > 1; x--) {
		if (ext_camptr[x][layer] != ext_camptr[x - 1][layer]) {
			Left_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	for (int x = W / 2; x < W; x++) {
		if (ext_camptr[x][layer] != ext_camptr[x + 1][layer]) {
			Right_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	// Update the center point
	LayerCount++;
	Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
	//ext_camptr[Center[layer]][layer] = 3;
	// Find the rest (Choose the start point based on last layer midpoint)-------------------------------------------
	for (; --layer > 30;) {
		R_Edge_is_found = false;
		L_Edge_is_found = false;
		//LEFT
		for (int x = Center[layer + 1]; x > 1; x--) {
			//Found when change from white to black && new edge is not far away from last edge
			if ((ext_camptr[x][layer] != ext_camptr[x - 1][layer]) && (abs(x - Left_edge[layer + 1]) <= 5)) {
				Left_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				L_Edge_is_found = true;
				break;
			}
		}
		if (L_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Left_edge[layer] = Left_edge[layer + 1];
			ext_camptr[Left_edge[layer]][layer] = 2;
		}
		//Right
		for (int x = Center[layer + 1]; x < W; x++) {
			if ((ext_camptr[x][layer] != ext_camptr[x + 1][layer]) && (abs(x - Right_edge[layer + 1]) <= 5)) {
				Right_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				R_Edge_is_found = true;
				break;
			}
		}
		if (R_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Right_edge[layer] = Right_edge[layer + 1];
			ext_camptr[Right_edge[layer]][layer] = 2;
		}
		// Update the center point
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
		ext_camptr[Center[layer]][layer] = 3;
		LayerCount++;
	}
	const int ServoP = 20;//20
	int sum = 0; // Initially 50 - 20
	for (int L = 55; L > 55 - LayerCount; L--) {
		sum += Center[L];
	}
	int Average = sum / LayerCount;
	//	string s = "Average: " + to_string(Average) + "\n";
	//	console.WriteString(s.c_str());
	//	if((Average-35)>0){
	//		servo.SetDegree(RightDegree);
	//	}
	//	else if((Average-35)<0){
	//		servo.SetDegree(LeftDegree);
	//	}
	//	else{
	//		servo.SetDegree(StraightDegree);
	//	}

	servo.SetDegree(s.kCenter - (Average - 35) * ServoP);
}

void Moving::NormalMovingTestingVersion2(FutabaS3010& servo, St7735r& lcd, CarManager::ServoBounds s) {
	// initialize LCD console
	LcdConsole::Config console_config;
	console_config.lcd = &lcd;
	LcdConsole console(console_config);
	int layer = 55;
	int LayerCount = 0;
	bool R_Edge_is_found;
	bool L_Edge_is_found;
	bool EncounterCrossing = false;
	int UpdatedDegree;
	const int ServoP = 20;
	// Find the origin (y=1). (Assume the original layer is accurate every time)-------------------------------------------
	for (int x = W / 2; x > 1; x--) {
		if (ext_camptr[x][layer] != ext_camptr[x - 1][layer]) {
			Left_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	for (int x = W / 2; x < W; x++) {
		if (ext_camptr[x][layer] != ext_camptr[x + 1][layer]) {
			Right_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	// Update the center point
	LayerCount++;
	Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
	/*ROUNDABOUT JUDGEMENT*/
	if (ext_camptr[Center[layer] - 1][layer] || ext_camptr[Center[layer]][layer]
																		  || ext_camptr[Center[layer] + 1][layer] == true) {
		/*ROUNDABOUT HANDLING*/
		//		string s = "Status: Roundabout\n";
		//		console.SetCursorRow(1);
		//		console.WriteString(s.c_str());

		for (int x = Center[layer]; x > 0; x--) {
			if (ext_camptr[x][layer] == false) {
				Right_edge[layer] = x;
				break;
			}
		}
		lcd.SetRegion(Lcd::Rect(Right_edge[layer], layer, 5, 5));
		lcd.FillColor(Lcd::kRed);
		for (int x = Right_edge[layer]; x > 0; x--) {
			Left_edge[layer] = x;
			if (ext_camptr[x][layer] == true) {
				break;
			}
		}
		lcd.SetRegion(Lcd::Rect(Left_edge[layer], layer, 5, 5));
		lcd.FillColor(Lcd::kRed);
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2; // Update new center point
		if ((s.kCenter - (Center[layer] - 35) * ServoP) > s.kLeftBound) {
			UpdatedDegree = s.kLeftBound;
		} else if ((s.kCenter - (Center[layer] - 35) * ServoP) < s.kRightBound) {
			UpdatedDegree = s.kRightBound;
		} else {
			UpdatedDegree = s.kCenter - (Center[layer] - 35) * ServoP;
		}
		servo.SetDegree(UpdatedDegree); //Action immediately for this special case
		return;
	}
	//ext_camptr[Center[layer]][layer] = 3;
	// Find the rest (Choose the start point based on last layer midpoint)-------------------------------------------
	for (; --layer > 30;) {
		R_Edge_is_found = false;
		L_Edge_is_found = false;
		//LEFT
		for (int x = Center[layer + 1]; x > 1; x--) {
			//Found when change from white to black && new edge is not far away from last edge
			if ((ext_camptr[x][layer] != ext_camptr[x - 1][layer]) && (abs(x - Left_edge[layer + 1]) <= 5)) {
				Left_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				L_Edge_is_found = true;
				break;
			}
		}
		if (L_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Left_edge[layer] = Left_edge[layer + 1];
			ext_camptr[Left_edge[layer]][layer] = 2;
		}
		//Right
		for (int x = Center[layer + 1]; x < W; x++) {
			if ((ext_camptr[x][layer] != ext_camptr[x + 1][layer]) && (abs(x - Right_edge[layer + 1]) <= 5)) {
				Right_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				R_Edge_is_found = true;
				break;
			}
		}
		if (R_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Right_edge[layer] = Right_edge[layer + 1];
			ext_camptr[Right_edge[layer]][layer] = 2;
		}
		// Update the center point
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
		/*ROUNDABOUT JUDGEMENT*/
		if (ext_camptr[Center[layer] - 1][layer] || ext_camptr[Center[layer]][layer]
																			  || ext_camptr[Center[layer] + 1][layer] == true) {
			/*ROUNDABOUT HANDLING*/
			//			string s = "Status: Roundabout\n";
			//			console.SetCursorRow(1);
			//			console.WriteString(s.c_str());
			for (int x = Center[layer]; x > 0; x--) {
				if (ext_camptr[x][layer] == false) {
					Right_edge[layer] = x;
					break;
				}
			}
			lcd.SetRegion(Lcd::Rect(Right_edge[layer], layer, 5, 5));
			lcd.FillColor(Lcd::kRed);
			for (int x = Right_edge[layer]; x > 0; x--) {
				Left_edge[layer] = x;
				if (ext_camptr[x][layer] == true) {
					break;
				}
			}
			lcd.SetRegion(Lcd::Rect(Left_edge[layer], layer, 5, 5));
			lcd.FillColor(Lcd::kRed);
			Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2; // Update new center point
			if ((s.kCenter - (Center[layer] - 35) * ServoP) > s.kLeftBound) {
				UpdatedDegree = s.kLeftBound;
			} else if ((s.kCenter - (Center[layer] - 35) * ServoP) < s.kRightBound) {
				UpdatedDegree = s.kRightBound;
			} else {
				UpdatedDegree = s.kCenter - (Center[layer] - 35) * ServoP;
			}
			servo.SetDegree(UpdatedDegree); //Action immediately for this special case
			return;
		}
		ext_camptr[Center[layer]][layer] = 3;
		LayerCount++;
		/*CROSSING JUDGEMENT*/
		if ((L_Edge_is_found || R_Edge_is_found) == false) {
			/*HAS ROAD - GO STRAIGHT*/
			//			string s = "Status: Crossing\n";
			//			console.SetCursorRow(1);
			//			console.WriteString(s.c_str());
			EncounterCrossing = true;
			break;
		}
	}
	/*CROSSING HANDLING*/
	if (HasRoad() && EncounterCrossing == true) {
		servo.SetDegree(s.kCenter);
		return;
	}
	int sum = 0; // Initially 50 - 20
	for (int L = 55; L > 55 - LayerCount; L--) {
		sum += Center[L];
	}
	int Average = sum / LayerCount;
	//	string s = "Average: " + to_string(Average) + "\n";
	//	console.WriteString(s.c_str());
	if ((s.kCenter - (Average - 35) * ServoP) > s.kLeftBound) {
		UpdatedDegree = s.kLeftBound;
	} else if ((s.kCenter - (Average - 35) * ServoP) < s.kRightBound) {
		UpdatedDegree = s.kRightBound;
	} else {
		UpdatedDegree = s.kCenter - (Average - 35) * ServoP;
	}
	servo.SetDegree(UpdatedDegree);
}

void Moving::NormalMovingTestingVersion3(FutabaS3010& servo, St7735r& lcd, AlternateMotor & motor_right, AlternateMotor& motor_left, CarManager::ServoBounds s) {
	// initialize LCD console
//	  LcdConsole::Config console_config;
//	  console_config.lcd = &lcd;
//	  LcdConsole console(console_config);

	int layer = 55;
	int LayerCount = 0;
	int NormalRoadWidth = 20; // Set this value for decelerate before roundabout
	bool R_Edge_is_found;
	bool L_Edge_is_found;
	bool EncounterCrossing = false;
	Timer::TimerInt RoundRoad_StartTime = 0;
	bool RoundRoadNow;
	int UpdatedDegree;
	const int ServoP = 20;
	// Find the origin (y=1). (Assume the original layer is accurate every time)-------------------------------------------
	for (int x = W / 2; x > 1; x--) {
		if (ext_camptr[x][layer] != ext_camptr[x - 1][layer]) {
			Left_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	for (int x = W / 2; x < W; x++) {
		if (ext_camptr[x][layer] != ext_camptr[x + 1][layer]) {
			Right_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	// Update the center point
	LayerCount++;
	Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
	/*ROUNDABOUT JUDGEMENT*/
	if (ext_camptr[Center[layer] - 1][layer] || ext_camptr[Center[layer]][layer]
						|| ext_camptr[Center[layer] + 1][layer] == true) {
		/*Double check*/
		//    if (DoubleCheckRound(Center[layer], layer)) {
		//      /*ROUNDABOUT HANDLING*/
		//      string s = "Status: Roundabout\n";
		//      console.SetCursorRow(1);
		//      console.WriteString(s.c_str());

		for (int x = Center[layer]; x > 0; x--) {
			if (ext_camptr[x][layer] == false) {
				Right_edge[layer] = x;
				break;
			}
		}
		for (int x = Right_edge[layer]; x > 0; x--) {
			Left_edge[layer] = x;
			if (ext_camptr[x][layer] == true) {
				break;
			}
		}
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2; // Update new center point
		//      lcd.SetRegion(Lcd::Rect(Center[layer], layer, 5, 5));
		//      lcd.FillColor(Lcd::kRed);
		if ((s.kCenter - (Center[layer] - 37) * ServoP) > s.kLeftBound) {
			UpdatedDegree = s.kLeftBound;
		} else if ((s.kCenter - (Center[layer] - 37) * ServoP) < s.kRightBound) {
			UpdatedDegree = s.kRightBound;
		} else {
			UpdatedDegree = s.kCenter - (Center[layer] - 37) * ServoP;
		}
		servo.SetDegree(UpdatedDegree); //Action immediately for this special case
		return;
		//    }
	}
	//ext_camptr[Center[layer]][layer] = 3;
	// Find the rest (Choose the start point based on last layer midpoint)-------------------------------------------
	for (; --layer > 30;) {
		R_Edge_is_found = false;
		L_Edge_is_found = false;
		//LEFT
		for (int x = Center[layer + 1]; x > 1; x--) {
			//Found when change from white to black && new edge is not far away from last edge
			if ((ext_camptr[x][layer] != ext_camptr[x - 1][layer]) && (abs(x - Left_edge[layer + 1]) <= 5)) {
				Left_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				L_Edge_is_found = true;
				break;
			}
		}
		if (L_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Left_edge[layer] = Left_edge[layer + 1];
			ext_camptr[Left_edge[layer]][layer] = 2;
		}
		//Right
		for (int x = Center[layer + 1]; x < W; x++) {
			if ((ext_camptr[x][layer] != ext_camptr[x + 1][layer]) && (abs(x - Right_edge[layer + 1]) <= 5)) {
				Right_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				R_Edge_is_found = true;
				break;
			}
		}
		if (R_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Right_edge[layer] = Right_edge[layer + 1];
			ext_camptr[Right_edge[layer]][layer] = 2;
		}
		// Update the center point
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
		/*Special Case Judgement----------------------------------------------------------*/
		//		if(R_Edge_is_found && L_Edge_is_found){
		/*ROUNDABOUT JUDGEMENT*/
		if (ext_camptr[Center[layer] - 1][layer] || ext_camptr[Center[layer]][layer]
																			  || ext_camptr[Center[layer] + 1][layer] == true) {
			/*Double check*/
			//      if (DoubleCheckRound(Center[layer], layer)) {
			//        /*ROUNDABOUT HANDLING*/
			//        RoundRoadNow = true;
			//        RoundRoad_StartTime = System::Time();
			//
			//        string s = "Status: Roundabout\n";
			//        console.SetCursorRow(1);
			//        console.WriteString(s.c_str());

			for (int x = Center[layer]; x > 0; x--) {
				if (ext_camptr[x][layer] == false) {
					Right_edge[layer] = x;
					break;
				}
			}
			for (int x = Right_edge[layer]; x > 0; x--) {
				Left_edge[layer] = x;
				if (ext_camptr[x][layer] == true) {
					break;
				}
			}
			Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2; // Update new center point

			//        lcd.SetRegion(Lcd::Rect(Center[layer], layer, 5, 5));
			//        lcd.FillColor(Lcd::kRed);

			if ((s.kCenter - (Center[layer] - 37) * ServoP) > s.kLeftBound) {
				UpdatedDegree = s.kLeftBound;
			} else if ((s.kCenter - (Center[layer] - 37) * ServoP) < s.kRightBound) {
				UpdatedDegree = s.kRightBound;
			} else {
				UpdatedDegree = s.kCenter - (Center[layer] - 37) * ServoP;
			}
			servo.SetDegree(UpdatedDegree); //Action immediately for this special case
			return;
			//      }
		}
		ext_camptr[Center[layer]][layer] = 3;
		LayerCount++;
		//		}
		/*CROSSING JUDGEMENT*/
		if ((L_Edge_is_found || R_Edge_is_found) == false) {
			/*CROSSING DOUBLE CHECK*/
			if (DoubleCheckCrossing(Center[layer], layer)) {
				/*HAS ROAD - GO STRAIGHT*/
				motor_right.SetPower(300);
				motor_left.SetPower(300);
				//        string s = "Status: Crossing\n";
				//        console.SetCursorRow(1);
				//        console.WriteString(s.c_str());
				//			EncounterCrossing = true;
				break;
			}
		}
	}

	/*CROSSING HANDLING*/
	//	if (HasRoad() && EncounterCrossing == true){
	//		servo.SetDegree(s.kCenter);
	//		return;
	//	}

	/*Round road exit handling*/
	// Cancel round road flag after 30s to avoid wrong judgement
	//  if ((System::Time() - RoundRoad_StartTime) > 30000) {
	//    RoundRoadNow = false;
	//  }
	//  if (RoundRoadNow && (Center[layer] == false)) {
	//  //    //Finding new center point
	//  //    servo.SetDegree(s.kLeftBound);
	//  //    return;
	//	  motor_right.SetPower(200);
	//	  motor_left.SetPower(200);
	//    }

	//  // Do special arrangement during round road period to help car exit
	//  if (RoundRoadNow && (Center[layer] == false)) {
	//    //Finding new center point
	//    servo.SetDegree(s.kLeftBound);
	//    return;
	//  }

	int sum = 0; // Initially 50 - 20
	for (int L = 55; L > 55 - LayerCount; L--) {
		sum += Center[L];
	}
	int Average = sum / LayerCount;
//	  	string s = "Average: " + to_string(Average) + "\n";
//	  	console.WriteString(s.c_str());
	if ((s.kCenter - (Average - 37) * ServoP) > s.kLeftBound) {
		UpdatedDegree = s.kLeftBound;
	} else if ((s.kCenter - (Average - 37) * ServoP) < s.kRightBound) {
		UpdatedDegree = s.kRightBound;
	} else {
		UpdatedDegree = s.kCenter - (Average - 37) * ServoP;
	}
	/*Servo Control*/
	servo.SetDegree(UpdatedDegree);
//	  string s = "UpdatedDegree: " + to_string(UpdatedDegree) + "\n";
//	  console.SetCursorRow(1);
//	  console.WriteString(s.c_str());


	/*Motor Control*/
	if(abs(UpdatedDegree-s.kCenter) >100){
		motor_right.SetPower(200);
		motor_left.SetPower(200);
	}
	// Reduce the speed before Roundabout
	else if(abs(Left_edge[55-LayerCount] - Right_edge[55-LayerCount]) >= NormalRoadWidth ){
		motor_right.SetPower(200);
		motor_left.SetPower(200);
	}
	else{
		motor_right.SetPower(300);
		motor_left.SetPower(300);
	}
}

void Moving::NormalMovingTestingVersion4(FutabaS3010& servo, St7735r& lcd, CarManager::Feature& feature, CarManager::ServoBounds s) {
	// initialize LCD console
	//  LcdConsole::Config console_config;
	//  console_config.lcd = &lcd;
	//  LcdConsole console(console_config);
	int layer = 55;
	int LayerCount = 0;
	bool R_Edge_is_found;
	bool L_Edge_is_found;
	bool EncounterCrossing = false;
	//  Timer::TimerInt RoundRoad_StartTime = 0;
	//  bool RoundRoadNow;
	int UpdatedDegree;
	const int ServoP = 20;
	// Find the origin (y=1). (Assume the original layer is accurate every time)-------------------------------------------
	for (int x = W / 2; x > 1; x--) {
		if (ext_camptr[x][layer] != ext_camptr[x - 1][layer]) {
			Left_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	for (int x = W / 2; x < W; x++) {
		if (ext_camptr[x][layer] != ext_camptr[x + 1][layer]) {
			Right_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	// Update the center point
	LayerCount++;
	Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
	/*ROUNDABOUT JUDGEMENT*/
	if (ext_camptr[Center[layer] - 1][layer] || ext_camptr[Center[layer]][layer]
																		  || ext_camptr[Center[layer] + 1][layer] == true) {
		/*Double check*/
		//    if (DoubleCheckRound(Center[layer], layer)) {
		//      /*ROUNDABOUT HANDLING*/
		//      string s = "Status: Roundabout\n";
		//      console.SetCursorRow(1);
		//      console.WriteString(s.c_str());
		feature = CarManager::Feature::kRoundabout;

		for (int x = Center[layer]; x > 0; x--) {
			if (ext_camptr[x][layer] == false) {
				Right_edge[layer] = x;
				break;
			}
		}
		for (int x = Right_edge[layer]; x > 0; x--) {
			Left_edge[layer] = x;
			if (ext_camptr[x][layer] == true) {
				break;
			}
		}
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2; // Update new center point
		//      lcd.SetRegion(Lcd::Rect(Center[layer], layer, 5, 5));
		//      lcd.FillColor(Lcd::kRed);
		if ((s.kCenter - (Center[layer] - 35) * ServoP) > s.kLeftBound) {
			UpdatedDegree = s.kLeftBound;
		} else if ((s.kCenter - (Center[layer] - 35) * ServoP) < s.kRightBound) {
			UpdatedDegree = s.kRightBound;
		} else {
			UpdatedDegree = s.kCenter - (Center[layer] - 35) * ServoP;
		}
		servo.SetDegree(UpdatedDegree); //Action immediately for this special case
		return;
		//    }
	}
	//ext_camptr[Center[layer]][layer] = 3;
	// Find the rest (Choose the start point based on last layer midpoint)-------------------------------------------
	for (; --layer > 30;) {
		R_Edge_is_found = false;
		L_Edge_is_found = false;
		//LEFT
		for (int x = Center[layer + 1]; x > 1; x--) {
			//Found when change from white to black && new edge is not far away from last edge
			if ((ext_camptr[x][layer] != ext_camptr[x - 1][layer]) && (abs(x - Left_edge[layer + 1]) <= 5)) {
				Left_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				L_Edge_is_found = true;
				break;
			}
		}
		if (L_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Left_edge[layer] = Left_edge[layer + 1];
			ext_camptr[Left_edge[layer]][layer] = 2;
		}
		//Right
		for (int x = Center[layer + 1]; x < W; x++) {
			if ((ext_camptr[x][layer] != ext_camptr[x + 1][layer]) && (abs(x - Right_edge[layer + 1]) <= 5)) {
				Right_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				R_Edge_is_found = true;
				break;
			}
		}
		if (R_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Right_edge[layer] = Right_edge[layer + 1];
			ext_camptr[Right_edge[layer]][layer] = 2;
		}
		// Update the center point
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
		/*Special Case Judgement----------------------------------------------------------*/
		//		if(R_Edge_is_found && L_Edge_is_found){
		/*ROUNDABOUT JUDGEMENT*/
		if (ext_camptr[Center[layer] - 1][layer] || ext_camptr[Center[layer]][layer]
																			  || ext_camptr[Center[layer] + 1][layer] == true) {
			/*Double check*/
			//      if (DoubleCheckRound(Center[layer], layer)) {
			//        /*ROUNDABOUT HANDLING*/
			//        RoundRoadNow = true;
			//        RoundRoad_StartTime = System::Time();
			//
			//        string s = "Status: Roundabout\n";
			//        console.SetCursorRow(1);
			//        console.WriteString(s.c_str());
			feature = CarManager::Feature::kRoundabout;

			for (int x = Center[layer]; x > 0; x--) {
				if (ext_camptr[x][layer] == false) {
					Right_edge[layer] = x;
					break;
				}
			}
			for (int x = Right_edge[layer]; x > 0; x--) {
				Left_edge[layer] = x;
				if (ext_camptr[x][layer] == true) {
					break;
				}
			}
			Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2; // Update new center point

			//        lcd.SetRegion(Lcd::Rect(Center[layer], layer, 5, 5));
			//        lcd.FillColor(Lcd::kRed);

			if ((s.kCenter - (Center[layer] - 35) * ServoP) > s.kLeftBound) {
				UpdatedDegree = s.kLeftBound;
			} else if ((s.kCenter - (Center[layer] - 35) * ServoP) < s.kRightBound) {
				UpdatedDegree = s.kRightBound;
			} else {
				UpdatedDegree = s.kCenter - (Center[layer] - 35) * ServoP;
			}
			servo.SetDegree(UpdatedDegree); //Action immediately for this special case
			return;
			//      }
		}
		ext_camptr[Center[layer]][layer] = 3;
		LayerCount++;
		//		}
		/*CROSSING JUDGEMENT*/
		if ((L_Edge_is_found || R_Edge_is_found) == false) {
			/*CROSSING DOUBLE CHECK*/
			if (DoubleCheckCrossing(Center[layer], layer)) {
				/*HAS ROAD - GO STRAIGHT*/
				//        string s = "Status: Crossing\n";
				//        console.SetCursorRow(1);
				//        console.WriteString(s.c_str());
				feature = CarManager::Feature::kCross;
				//			EncounterCrossing = true;
				break;
			}
		}
	}

	/*CROSSING HANDLING*/
	//	if (HasRoad() && EncounterCrossing == true){
	//		servo.SetDegree(s.kCenter);
	//		return;
	//	}

	/*Round road exit handling*/
	// Cancel round road flag after 30s to avoid wrong judgement
	//  if ((System::Time() - RoundRoad_StartTime) > 30000) {
	//    RoundRoadNow = false;
	//  }
	//  // Do special arrangement during round road period to help car exit
	//  if (RoundRoadNow && (Center[layer] == false)) {
	//    //Finding new center point
	//    servo.SetDegree(s.kLeftBound);
	//    return;
	//  }

	int sum = 0; // Initially 50 - 20
	for (int L = 55; L > 55 - LayerCount; L--) {
		sum += Center[L];
	}
	int Average = sum / LayerCount;
	//	string s = "Average: " + to_string(Average) + "\n";
	//	console.WriteString(s.c_str());
	if ((s.kCenter - (Average - 35) * ServoP) > s.kLeftBound) {
		UpdatedDegree = s.kLeftBound;
	} else if ((s.kCenter - (Average - 35) * ServoP) < s.kRightBound) {
		UpdatedDegree = s.kRightBound;
	} else {
		UpdatedDegree = s.kCenter - (Average - 35) * ServoP;
	}
	servo.SetDegree(UpdatedDegree);
}



















void Moving::NormalMovingTestingVersion5(FutabaS3010& servo, St7735r& lcd, AlternateMotor & motor_right, AlternateMotor& motor_left, CarManager::ServoBounds s) {
	// initialize LCD console
//	  LcdConsole::Config console_config;
//	  console_config.lcd = &lcd;
//	  LcdConsole console(console_config);

	int layer = 55;
	int LayerCount = 0;
	int NormalRoadWidth_UpperBound = 66; // Set this value for decelerate before roundabout
	bool R_Edge_is_found;
	bool L_Edge_is_found;
	bool EncounterCrossing = false;
	static Timer::TimerInt RoundRoad_StartTime;
	static bool RoundRoadNow;
	int UpdatedDegree;
	int ServoP = 30;
	const int ServoP_Normal = 20;
	const int ServoP_Roundabout = 30;

	/*Servo P value setting*/
		/*ROUNDABOUT REMINDER*/
//	if(RoundRoadNow){
//		if((System::Time() - RoundRoad_StartTime) > 3000){
//			ServoP = ServoP_Normal;
//			RoundRoadNow = false;
//		}
//	}
//	else if( (abs(abs(Left_edge[55 - LayerCount + 1]) - abs(Right_edge[55 - LayerCount + 1])))
//																				>= NormalRoadWidth_UpperBound ){
//			RoundRoadNow = true;
//			RoundRoad_StartTime = System::Time();
//			ServoP = ServoP_Roundabout;
//		}
//	else{
//		ServoP = ServoP_Normal;
//	}

	// Find the origin (y=1). (Assume the original layer is accurate every time)-------------------------------------------
	for (int x = W / 2; x > 1; x--) {
		if (ext_camptr[x][layer] != ext_camptr[x - 1][layer]) {
			Left_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	for (int x = W / 2; x < W; x++) {
		if (ext_camptr[x][layer] != ext_camptr[x + 1][layer]) {
			Right_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	// Update the center point
	LayerCount++;
	Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
	/*ROUNDABOUT JUDGEMENT*/
	if ((ext_camptr[Center[layer] - 1][layer]
									  || ext_camptr[Center[layer]][layer]
																   || ext_camptr[Center[layer] + 1][layer]) == true) {
		/*Double check*/
		//    if (DoubleCheckRound(Center[layer], layer)) {
		//      /*ROUNDABOUT HANDLING*/
		//      string s = "Status: Roundabout\n";
		//      console.SetCursorRow(1);
		//      console.WriteString(s.c_str());

		for (int x = Center[layer]; x > 0; x--) {
			if (ext_camptr[x][layer] == false) {
				Right_edge[layer] = x;
				break;
			}
		}
		for (int x = Right_edge[layer]; x > 0; x--) {
			Left_edge[layer] = x;
			if (ext_camptr[x][layer] == true) {
				break;
			}
		}
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2; // Update new center point
		//      lcd.SetRegion(Lcd::Rect(Center[layer], layer, 5, 5));
		//      lcd.FillColor(Lcd::kRed);
		if ((s.kCenter - (Center[layer] - 37) * ServoP) > s.kLeftBound) {
			UpdatedDegree = s.kLeftBound;
		} else if ((s.kCenter - (Center[layer] - 37) * ServoP) < s.kRightBound) {
			UpdatedDegree = s.kRightBound;
		} else {
			UpdatedDegree = s.kCenter - (Center[layer] - 37) * ServoP;
		}
		servo.SetDegree(UpdatedDegree); //Action immediately for this special case
		return;
		//    }
	}
	//ext_camptr[Center[layer]][layer] = 3;
	// Find the rest (Choose the start point based on last layer midpoint)-------------------------------------------
	for (; --layer > 30;) {
		R_Edge_is_found = false;
		L_Edge_is_found = false;
		//LEFT
		for (int x = Center[layer + 1]; x > 1; x--) {
			//Found when change from white to black && new edge is not far away from last edge
			if ((ext_camptr[x][layer] != ext_camptr[x - 1][layer]) && (abs(x - Left_edge[layer + 1]) <= 5)) {
				Left_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				L_Edge_is_found = true;
				break;
			}
		}
		if (L_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Left_edge[layer] = Left_edge[layer + 1];
			ext_camptr[Left_edge[layer]][layer] = 2;
		}
		//Right
		for (int x = Center[layer + 1]; x < W; x++) {
			if ((ext_camptr[x][layer] != ext_camptr[x + 1][layer]) && (abs(x - Right_edge[layer + 1]) <= 5)) {
				Right_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				R_Edge_is_found = true;
				break;
			}
		}
		if (R_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Right_edge[layer] = Right_edge[layer + 1];
			ext_camptr[Right_edge[layer]][layer] = 2;
		}
		// Update the center point
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
		/*Special Case Judgement----------------------------------------------------------*/
		//		if(R_Edge_is_found && L_Edge_is_found){
		/*ROUNDABOUT JUDGEMENT*/
		if ((ext_camptr[Center[layer] - 1][layer]
										   || ext_camptr[Center[layer]][layer]
																			  || ext_camptr[Center[layer] + 1][layer]) == true) {
			/*Double check*/
//	        string s = "Status: Roundabout\n";
//	        console.SetCursorRow(1);
//	        console.WriteString(s.c_str());

			for (int x = Center[layer]; x > 0; x--) {
				if (ext_camptr[x][layer] == false) {
					Right_edge[layer] = x;
					break;
				}
			}
			for (int x = Right_edge[layer]; x > 0; x--) {
				Left_edge[layer] = x;
				if (ext_camptr[x][layer] == true) {
					break;
				}
			}
			Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2; // Update new center point

			//        lcd.SetRegion(Lcd::Rect(Center[layer], layer, 5, 5));
			//        lcd.FillColor(Lcd::kRed);

			if ((s.kCenter - (Center[layer] - 37) * ServoP) > s.kLeftBound) {
				UpdatedDegree = s.kLeftBound;
			} else if ((s.kCenter - (Center[layer] - 37) * ServoP) < s.kRightBound) {
				UpdatedDegree = s.kRightBound;
			} else {
				UpdatedDegree = s.kCenter - (Center[layer] - 37) * ServoP;
			}
			servo.SetDegree(UpdatedDegree); //Action immediately for this special case
			return;
			//      }
		}
		ext_camptr[Center[layer]][layer] = 3;
		LayerCount++;
		//		}
		/*CROSSING JUDGEMENT*/
		if ((L_Edge_is_found || R_Edge_is_found) == false) {
			/*CROSSING DOUBLE CHECK*/
			if (DoubleCheckCrossing(Center[layer], layer)) {
				/*HAS ROAD - GO STRAIGHT*/
				motor_right.SetPower(350);
				motor_left.SetPower(350);
				//        string s = "Status: Crossing\n";
				//        console.SetCursorRow(1);
				//        console.WriteString(s.c_str());
				//			EncounterCrossing = true;
				break;
			}
		}
	}

	/*CROSSING HANDLING*/
	//	if (HasRoad() && EncounterCrossing == true){
	//		servo.SetDegree(s.kCenter);
	//		return;
	//	}


	int sum = 0; // Initially 50 - 20
	for (int L = 55; L > 55 - LayerCount; L--) {
		sum += Center[L];
	}
	int Average = sum / LayerCount;
//	  	string s = "Average: " + to_string(Average) + "\n";
//	  	console.WriteString(s.c_str());
	if ((s.kCenter - (Average - 37) * ServoP) > s.kLeftBound) {
		UpdatedDegree = s.kLeftBound;
	} else if ((s.kCenter - (Average - 37) * ServoP) < s.kRightBound) {
		UpdatedDegree = s.kRightBound;
	} else {
		UpdatedDegree = s.kCenter - (Average - 37) * ServoP;
	}
	/*Servo Control*/
	servo.SetDegree(UpdatedDegree);
//	  string s = "UpdatedDegree: " + to_string(UpdatedDegree) + "\n";
//	  console.SetCursorRow(1);
//	  console.WriteString(s.c_str());

//	string s = "  Width: " + to_string(abs(abs(Left_edge[55 - LayerCount + 1]) - abs(Right_edge[55 - LayerCount + 1])))+ "\n";
//	console.SetCursorRow(1);
//	console.WriteString(s.c_str());

	//      if (DoubleCheckRound(Center[layer], layer)) {
	//        /*ROUNDABOUT HANDLING*/
	//        RoundRoadNow = true;
	//        RoundRoad_StartTime = System::Time();
	//

	/*Round road exit handling*/
	// Cancel round road flag after 30s to avoid wrong judgement
	//  if ((System::Time() - RoundRoad_StartTime) > 30000) {
	//    RoundRoadNow = false;
	//  }
	//  // Do special arrangement during round road period to help car exit
	//  if (RoundRoadNow && (Center[layer] == false)) {
	//    //Finding new center point
	//    servo.SetDegree(s.kLeftBound);
	//    return;
	//  }

	/*Motor Control*/
	if(!RoundRoadNow){
		if(abs(UpdatedDegree-s.kCenter) >100){
			motor_right.SetPower(250);
			motor_left.SetPower(250);
		}
		// Reduce the speed before Roundabout
		else if((abs(abs(Left_edge[55 - LayerCount + 1]) - abs(Right_edge[55 - LayerCount + 1]))) >= NormalRoadWidth_UpperBound ){
			motor_right.SetPower(200);
			motor_left.SetPower(200);
			RoundRoadNow = true;
			RoundRoad_StartTime = System::Time();
		}
		else{
			motor_right.SetPower(350);
			motor_left.SetPower(350);
		}
	}
	// During Roundabout
	else{
		motor_right.SetPower(200);
		motor_left.SetPower(200);
		if((System::Time() - RoundRoad_StartTime) > 3000){
			ServoP = ServoP_Normal;
			RoundRoadNow = false;
		}
	}
}
















bool Moving::HasCornerTesting() {
	int layer = 55;
	bool R_Edge_is_found;
	bool L_Edge_is_found;
	// Find the origin (y=1). (Assume the original layer is accurate every time)-------------------------------------------
	for (int x = W / 2; x > 1; x--) {
		if (ext_camptr[x][layer] != ext_camptr[x - 1][layer]) {
			Left_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	for (int x = W / 2; x < W; x++) {
		if (ext_camptr[x][layer] != ext_camptr[x + 1][layer]) {
			Right_edge[layer] = x;
			ext_camptr[x][layer] = 2;
			break;
		}
	}
	// Update the center point
	Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
	ext_camptr[Center[layer]][layer] = 3;
	// Find the rest (Choose the start point based on last layer midpoint)-------------------------------------------
	for (; --layer > 30;) {
		R_Edge_is_found = false;
		L_Edge_is_found = false;
		//LEFT
		for (int x = Center[layer + 1]; x > 1; x--) {
			//Found when change from white to black && new edge is not far away from last edge
			if ((ext_camptr[x][layer] != ext_camptr[x - 1][layer]) && (abs(x - Left_edge[layer + 1]) <= 5)) {
				Left_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				L_Edge_is_found = true;
				break;
			}
		}
		if (L_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Left_edge[layer] = Left_edge[layer + 1];
			ext_camptr[Left_edge[layer]][layer] = 2;
		}
		//Right
		for (int x = Center[layer + 1]; x < W; x++) {
			if ((ext_camptr[x][layer] != ext_camptr[x + 1][layer]) && (abs(x - Right_edge[layer + 1]) <= 5)) {
				Right_edge[layer] = x;
				ext_camptr[x][layer] = 2;
				R_Edge_is_found = true;
				break;
			}
		}
		if (R_Edge_is_found == false) {
			//DO CANT FIND THING HERE
			Right_edge[layer] = Right_edge[layer + 1];
			ext_camptr[Right_edge[layer]][layer] = 2;
		}
		// Update the center point
		Center[layer] = (Left_edge[layer] + Right_edge[layer]) / 2;
		ext_camptr[Center[layer]][layer] = 3;
		//Judgement
		if ((L_Edge_is_found == false ||
				(Left_edge[layer + 1] - Left_edge[layer + 3]) * (Left_edge[layer] - Left_edge[layer + 3]) <= 0) &&
				(R_Edge_is_found == false ||
						(Right_edge[layer + 1] - Right_edge[layer + 3]) * (Right_edge[layer] - Right_edge[layer + 3]) <= 0)) {
			return true;
		} else
			return false;
	}

}




//---------
/*Moving function*/
bool Moving::NormalMoving(FutabaS3010& servo, CarManager::ServoBounds s) {
	const int ServoPara = 40;
	int sum = 0;
	for (int L = 60; L > 30; L--) {
		sum += Center[L];
	}
	int Average = sum / 30;
	servo.SetDegree(s.kCenter - (Average - 40) * ServoPara);
	return true;
}

bool Moving::RoundMoving(FutabaS3010& servo, CarManager::ServoBounds s) {
	//TURN LEFT ANYWAY
	servo.SetDegree(s.kLeftBound);
	return true;
}

bool Moving::CrossingMoving(FutabaS3010& servo, CarManager::ServoBounds s) {
	//GO STRAIGHT ANYWAY
	servo.SetDegree(s.kCenter);
	return true;
}

//bool Moving::SshapeMoving(FutabaS3010 & servo);

/*Indicator function*/

/*
 * This Eye function is served as car eye, it can detect a 3x3 rectangle in front of car
 * @return: if whole rect is black, return false. Otherwise, return true
 * Black - 1.
 * White - 0.
 */
bool Moving::HasRoad() {
	bool road = true;
	const int width = 3;
	const int height = 3;
	int eyesight = 80; // eyesight is how many pixels from car to the rect
	int sum = 0;
	//ext_camptr[40][eyesight]=3;
	for (int i = eyesight; i > eyesight - height; i--) {
		for (int j = W / 2 - static_cast<int>(width / 2); j < (W / 2 - static_cast<int>(width / 2)) + width + 1; j++) {
			sum = (ext_camptr[j][i]) ? sum + 1 : sum;
		}
	}
	if (sum == width * height) {
		road = false;
	}
	return road;
}
/*
 * This function is served as a corner detector
 * @return: True if there are corners, False if there are no corners
 */
bool Moving::HasCorner(const int layer, const bool find_L, const bool find_R) {
	if (
			(find_L == false ||
					(Left_edge[layer + 1] - Left_edge[layer + 3]) * (Left_edge[layer] - Left_edge[layer + 3]) <= 0) &&
					(find_R == false ||
							(Right_edge[layer + 1] - Right_edge[layer + 3]) * (Right_edge[layer] - Right_edge[layer + 3]) <= 0)
	) {
		/*
		 * CAN ADD Double-check here
		 */
		return true;
	} else
		return false;
}

/*Printing function*/
/*
 * A function printing out original photo
 * 60*80 = GetW * GetH , which represents bits, not byte
 * GetBufferSize uses Byte as unit.
 * GetBufferSize()*8 = GetW*GetH
 */
bool Moving::printCameraImage(const Byte* image, St7735r& lcd) {
	if (image == NULL) {
		return false;
	} else {
		lcd.SetRegion(Lcd::Rect(0, 0, W, H));
		lcd.FillBits(0x001F, 0xFFFF, image, W * H);
		return true;
	}
}

//A function display camera image on LCD
bool Moving::Print2Darray(Led& led, St7735r& lcd) {
	//led.SetEnable(true);
	// rewrite lcd with new data
	for (int y = 0; y < H; y++) {
		for (int x = 0; x < W; x++) {
			// To scale the image to fill 120x160 LCD
			Lcd::Rect pixel(x, y + H + 2, 1, 1);
			lcd.SetRegion(pixel);
			switch (ext_camptr[x][y]) {
			case 0:
				lcd.FillColor(Lcd::kWhite);
				break;
			case 1:
				lcd.FillColor(Lcd::kBlack);
				break;
			case 2:
				lcd.FillColor(Lcd::kGreen);
				break;
			case 3:
				lcd.FillColor(Lcd::kRed);
				break;
			default:
				return false;
			}
		}
	}
	return true;
}

/*Helping function*/

/*
 * A function convert the byte array into 2D array
 * @ camBuffer: initial camera array in Byte
 * @ return type: int array in 0/1 - only one bit
 * NOTE: Inside the camera, one pixel = one bit
 */
void Moving::extract_cam(const Byte* camBuffer) {
	Byte CamByte;
	Uint pos = 0;
	int bit_pos = 8;
	// Get 8 bits info. from byte and pass it into a new array called extract_cam
	for (Uint i = 0; i < H; i++) {
		for (Uint j = 0; j < W; j++) {
			if (--bit_pos < 0) // Update after 8 bits are read
			{
				bit_pos = 7;// to track which position in a branch of Byte(Totally 8) is being read now.
				++pos;// to track which position in Byte array is being read now.
			}
			ext_camptr[j][i] = GET_BIT(camBuffer[pos], bit_pos);
		}
	}
}

//Median Filter
void Moving::Med_Filter() {
	for (int j = 1; j < H - 1; j++) {
		for (int i = 1; i < W - 1; i++) {
			int count = 0;
			for (int y = j - 1; y < j + 2; y++) {
				for (int x = i - 1; x < i + 2; x++) {
					count += (int) ext_camptr[x][y];
				}
			}
			if (count >= 5) {
				ext_camptr[i][j] = 1;
			} else {
				ext_camptr[i][j] = 0;
			}
		}
	}
}

bool Moving::DoubleCheckRound(int center_xcor, int center_ycor) {
	int radius_L = 20;
	int radius_R = 10;
	int radius_F = 20; /*front*/
	bool IsRound_Front = false;
	bool IsRound_Right = false;
	bool IsRound_Left = false;
	/*front*/
	for (int y = center_ycor; y > center_ycor - radius_F; y--) {
		if (!ext_camptr[center_xcor][y]) {
			IsRound_Front = true;
			break;
		}
	}
	/*right*/
	for (int x = center_xcor; x < center_xcor + radius_R; x++) {
		if (!ext_camptr[x][center_ycor]) {
			IsRound_Right = true;
			break;
		}
	}
	/*left*/
	for (int x = center_xcor; x > center_xcor - radius_L; x--) {
		if (!ext_camptr[x][center_ycor]) {
			IsRound_Left = true;
			break;
		}
	}
	if (IsRound_Front && IsRound_Right && IsRound_Left) {
		return true;
	} else
		return false;
}

bool Moving::DoubleCheckCrossing(int center_xcor, int center_ycor) {
	int eyesight = 7;
	if (ext_camptr[center_xcor][center_ycor - eyesight] == false) {
		return true;
	}
	return false;
}

/*Testing function*/
/*
 * Print Frame
 * @lcd: the lcd where you want to print
 * @x,y the coordinate for the left top point
 * @w: width
 * @h: height
 */
void Moving::PrintingFrame(St7735r& lcd, int x, int y, int w, int h) {
	//TOP and BOTTOM
	for (int i = x; i < x + w; i++) {
		Lcd::Rect pixel1(i, y, 1, 1);
		lcd.SetRegion(pixel1);
		lcd.FillColor(Lcd::kRed);
	}
	for (int i = x; i < x + w; i++) {
		Lcd::Rect pixel2(i, y + h, 1, 1);
		lcd.SetRegion(pixel2);
		lcd.FillColor(Lcd::kRed);
	}
	//LEFT and RIGHT
	for (int j = y; j < y + h; j++) {
		Lcd::Rect pixel3(x, j, 1, 1);
		lcd.SetRegion(pixel3);
		lcd.FillColor(Lcd::kRed);
	}
	for (int j = y; j < y + h; j++) {
		Lcd::Rect pixel4(x + w, j, 1, 1);
		lcd.SetRegion(pixel4);
		lcd.FillColor(Lcd::kRed);
	}
}
/*
 * Print 4 Frames with size of 20x20 on each corner
 * LCD size:120*128 120 is x, 128 is y
 * (0,0) can not be displayed (1,2) is the original point
 */
void Moving::Printing4Frames(St7735r& lcd) {
	//Print Frame1 left top
	PrintingFrame(lcd, 1, 1, 20, 20);
	//Print Frame2 right top
	PrintingFrame(lcd, 99, 1, 20, 20);
	//Print Frame3 left bottom
	PrintingFrame(lcd, 1, 107, 20, 20);
	//Print Frame4 right bottom
	PrintingFrame(lcd, 99, 107, 20, 20);
}

/*
 * Print 6 Frames with size of 20x20 on each corner
 */
void Moving::Printing6Frames(St7735r& lcd) {
	PrintingFrame(lcd, 0, 0, 20, 20);
	PrintingFrame(lcd, 140, 0, 20, 20);
	PrintingFrame(lcd, 0, 50, 20, 20);
	PrintingFrame(lcd, 140, 50, 20, 20);
	PrintingFrame(lcd, 0, 100, 20, 20);
	PrintingFrame(lcd, 140, 100, 20, 20);
}
