import processing.serial.*;
import java.util.*;

Serial myPort;

int kHandshake = 0xF0;
int kEND = 0xFF;
int kACK = 0xF1;

int kSpeed = 0x10;
int kSlopeDeg = 0x11;
int kDist = 0x12;
int kFeature = 0x13;
int kSide = 0x14;
int kReq = 0x15;

int index = 0;

int speed = 0;
int slopedeg = 0;
int dist = 0;
int feat = 0;
int side = 0;

int[] DataArray = new int[6];

boolean flag = false;

void setup() {
  size(300, 300);
  textSize(16);
  fill(0);
  background(255);
  text("Speed = " + speed, 10, 30);
  text("Servo Deg = " + slopedeg, 10, 70);
  text("Distance = " + dist, 10, 110);
  text("Feature = " + feat, 10, 150);
  text("Side = " + side, 10, 190);
  text("Package ID = N/A", 10, 230);
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.buffer(1);
}


void test(){
  background(255);
  if (!flag){
    ellipse(280, 280, 10, 10);
    flag = true;
  } else flag = false;
  text("Speed = " + speed, 10, 30);
  text("Servo Deg = " + slopedeg, 10, 70);
  text("Distance = " + dist, 10, 110);
  text("Feature = " + feat, 10, 150);
  text("Side = " + side, 10, 190);
  text("Package ID = N/A", 10, 230);
  
  delay(100);
}

void draw() {
  if (myPort.available() > 0) {
    
    /* Fetch Info */
    int temp = myPort.read();
    print(temp, ' ');
    if (temp == kHandshake && index == 0){
      DataArray[0] = temp;
      index++;
      return;
    } else if (temp == kEND && index == 5){
      DataArray[5] = temp;
      index = 0;
      print("\n\n");
    } else if (index != 0 && index != 5){
      DataArray[index] = temp;
      index++;
      return;
    } else {
      for (int i = 0; i < 6; i++){
        DataArray[i] = 0;
      }
      return;
    }
    
    /* Assign Temporary Variables */
    int ID = DataArray[1];
    int type = DataArray[2];
    int value = (DataArray[3] << 8) + DataArray[4];
    
    /* Assign Variables to Buffer */
    if (type == kSpeed){
      speed = value;
    } else if (type == kSlopeDeg){
      slopedeg = value;
    } else if (type == kDist){
      dist = value;
    } else if (type == kFeature){
      feat = value;
    } else if (type == kSide){
      side = value;
    } else return; 
        
    /* Send ACK Package */
    //myPort.write(kACK);
    ////delay(1);
    //myPort.write(ID);
    ////delay(1);
    //myPort.write(0);
    ////delay(1);
    //myPort.write(0);
    ////delay(1);
    //myPort.write(0);
    ////delay(1);
    //myPort.write(kEND);   
    
    print(ID + " ACK.\n");
    
    if (!flag){
      ellipse(280, 280, 10, 10);
      flag = true;
    } else flag = false;
    
    background(255);
    text("Speed = " + speed, 10, 30);
    text("Servo Deg = " + ((byte)slopedeg), 10, 70);
    text("Distance = " + dist, 10, 110);
    text("Feature = " + feat, 10, 150);
    text("Side = " + side, 10, 190);
    text("Package ID = " + ID, 10, 230);
   
  } 
}