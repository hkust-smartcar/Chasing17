import processing.serial.*; //<>//
import java.util.*;
import controlP5.*;
import g4p_controls.*;

//Frame rate
int frate = 240; // Increase frame rate for real time graph plotting, depending on the rate of data
//BT port
Boolean btEnable = true;
int portNum = 0;  //Choose the correct port here
int baudRate = 115200; //Baud rate of the bluetooth
//Change constant
int consSize = 16;  //Number of constant to change
//Plot graph
int range1 = 20000, range2 = 90;  //Upper limit of the graph
color[] colorArray = {#03FF04, #03FF04, #FF0303, #031DFF, #03FF04, #FF0303};  //Add color here for more variable

//Camera algorithm
void trytry() {

  if (selectedIndex >= 0) {

    background(#D3D3D3);

    Boolean imageTemp[][] = imageData.get(selectedIndex);
    int zeroX = 280, zeroY = 200;

    noStroke();
    for (int y=0; y<camHeight; y++) {
      for (int x=0; x<camWidth; x++) {
        if (imageTemp[y][x]) {
          fill(0);
        } else {
          fill(255);
        }
        rect(zeroX + pixelSide*x, zeroY + pixelSide*y, pixelSide, pixelSide);
      }
    }

    float xInc = 0.03, yInc = 0.01;
    pixelSide = 2;
    for (int y=0; y<camHeight; y++) {
      for (int x=0; x<camWidth; x++) {
        if (imageTemp[y][x]) {
          fill(0);
        } else {
          fill(255);
        }
        rect(900 + pixelSide*(x-camWidth/2)*(1+((camHeight-y)-1)*xInc), 600 - pixelSide*(camHeight-y)*(1+((camHeight-y)-1)*yInc), pixelSide, pixelSide);
      }
    }
    pixelSide = 4;

    if (mouseX>zeroX && mouseX<zeroX+pixelSide*camWidth
      && mouseY>zeroY && mouseY<zeroY+pixelSide*camHeight) {
      int x = 0, y = 0;
      x = (mouseX-zeroX)/pixelSide;
      y = (mouseY-zeroY)/pixelSide;
      fill(100);
      rect(zeroX + pixelSide*x, zeroY + pixelSide*y, pixelSide, pixelSide);

      int centreX = 920, centreY = 300;
      int sum = 0;

      strokeWeight(3);
      sum = getSum(imageTemp, x, y, -1, 0);
      stroke(#F20000);
      line(centreX, centreY, centreX-sum, centreY);
      sum = getSum(imageTemp, x, y, 1, 0);
      stroke(#E800ED);
      line(centreX, centreY, centreX+sum, centreY);
      sum = getSum(imageTemp, x, y, 0, -1);
      stroke(#3403FF);
      line(centreX, centreY, centreX, centreY-sum);
      sum = getSum(imageTemp, x, y, 0, 1);
      stroke(#038901);
      line(centreX, centreY, centreX, centreY+sum);

      stroke(100);
      sum = getSum(imageTemp, x, y, -1, -1);
      line(centreX, centreY, centreX-sum*cos(45), centreY-sum*cos(45));
      sum = getSum(imageTemp, x, y, -1, 1);
      line(centreX, centreY, centreX-sum*cos(45), centreY+sum*cos(45));
      sum = getSum(imageTemp, x, y, 1, -1);
      line(centreX, centreY, centreX+sum*cos(45), centreY-sum*cos(45));
      sum = getSum(imageTemp, x, y, 1, 1);
      line(centreX, centreY, centreX+sum*cos(45), centreY+sum*cos(45));
    }
  }
}

/*
//Example of receiving constants in smartcar.
 string str;  //global
 bool tuning = false;  //global
 vector<double> constVector;  //global
 
 bool bluetoothListener(const Byte *data, const size_t size) {
 
 if (data[0] == 't') {
 tune = 1;
 inputStr = "";
 }
 if (tune) {
 int i = 0;
 while (i<size) {
 if (data[i] != 't' && data[i] != '\n') {
 inputStr += (char)data[i];
 } else if (data[i] == '\n') {
 tune = 0;
 }
 i++;
 }
 if (!tune) {
 constVector.clear();
 
 char * pch;
 pch = strtok(&inputStr[0], " ,");
 while (pch != NULL){
 double constant;
 stringstream(pch) >> constant;
 constVector.push_back(constant);
 pch = strtok (NULL, " ,");
 }
 
 //global variable
 powAngP = constVector[0];
 powAngI = constVector[1];
 powAngD = constVector[2];
 }
 }
 
 }
 
 */

/*
//Example of sending 2 variables from smartcar.
 char speedChar[15] = {};
 sprintf(speedChar, "%.1f,%.2f,%.2f\n", 1.0, leftSpeed, rightSpeed); //first one must be positive, seperated by commas, end by '\n'
 string speedStr = speedChar;
 const Byte speedByte = 85;
 bluetooth1.SendBuffer(&speedByte, 1);
 bluetooth1.SendStr(speedStr);
 */

ControlP5 cp5;
Serial myPort;

int startTime = 0, currentTime = 0;
int viewMode = 0;
String inputString = "";

BufferedReader reader;
PrintWriter writer;

float getDistance(float x1, float y1, float x2, float y2) {
  return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}

int getSum(Boolean[][] imageTemp, int x, int y, int xInc, int yInc) {
  float sum = 0;
  int xTemp = x+xInc;
  int yTemp = y+yInc;
  while (xTemp>=0 && xTemp<camWidth
    && yTemp>=0 && yTemp<camHeight) {
    if (imageTemp[yTemp][xTemp]) {
      sum += (float)50/getDistance(x, y, xTemp, yTemp);
      //sum+=2;
    }
    xTemp += xInc;
    yTemp += yInc;
  }
  return (int)sum;
}

void displayText(String text, int x, int y, int w, int size) {

  fill(#D3D3D3);
  noStroke();
  rect(x-size, y-size, w, size*1.5);
  fill(0);
  textSize(size);
  text(text, x, y);
}

void keyPressed() {

  if (key == UP) {
    myPort.write('w');
  } else if (key == DOWN) {
    myPort.write('s');
  } else if (key == LEFT) {
    myPort.write('a');
  } else if (key == RIGHT) {
    myPort.write('d');
  }
}

void keyReleased() {

  if (key == UP) {
    myPort.write('W');
  } else if (key == DOWN) {
    myPort.write('S');
  } else if (key == LEFT) {
    myPort.write('A');
  } else if (key == RIGHT) {
    myPort.write('D');
  }
}

void setup() {

  printArray(Serial.list());
  if (btEnable) {
    myPort = new Serial(this, Serial.list()[portNum], baudRate);
    myPort.clear();
  }
  startTime = millis();
  frameRate(frate);
  size(1200, 660);
  background(#D3D3D3);
  cp5 = new ControlP5(this);

  btName = new String[btSize];
  tfName = new String[textFieldSize];
  constantArr = new String[textFieldSize];
  imageData = new ArrayList();
  imageName = new ArrayList();

  buttonSetUp();
  tfSetUp();
  readConstant();
  listSetUp();

  for (int y=0; y<camHeight; y++) {
    for (int x=0; x<camWidth; x++) {
      pixelArray[y][x] = false;
    }
  }

  fill(#898989);
  noStroke();
  rect(imageX, imageY, camWidth*pixelSide, camHeight*pixelSide);
  rect(boundaryX, boundaryY, camWidth*pixelSide, camHeight*pixelSide);
  rect(regionX, regionY, camWidth*pixelSide, camHeight*pixelSide);
  fill(#404040);
  stroke(0);
  rect(graphOneX, graphOneY, graphWidth, graphHeight);
  rect(graphTwoX, graphTwoY, graphWidth, graphHeight);
  stroke(255);
  strokeWeight(0);
  line(graphOneX+1, graphOneY+graphHeight/2, graphOneX+graphWidth-1, graphOneY+graphHeight/2);
  line(graphTwoX+1, graphTwoY+graphHeight/2, graphTwoX+graphWidth-1, graphTwoY+graphHeight/2);
}

void draw() {

  if (viewMode == 2) {
    trytry();
  }

  if (btEnable && myPort.available() > 0) {

    int inputInt = myPort.read();

    //encoder
    if (inputInt == 85 && viewMode <= 1) {

      while (true) {
        print(',');
        if (myPort.available() > 0) {
          inputString = myPort.readStringUntil('\n');
          break;
        }
      }
      if (inputString != null) {
        inputString = inputString.trim();
        displayText(inputString, 150, 620, 600, 32);
        stringToDouble(inputString);
      }
      getData();
      plotGraph();
    }

    //image
    if (inputInt == 170) {

      int i = 0;
      arrayPosX = 0;
      arrayPosY = 0;

      while (i < camWidth*camHeight/8) {
        print('.');
        if (myPort.available() > 0) {
          inputInt = myPort.read();
          getImage(inputInt);
          i++;
        }
      }
      if (viewMode == 0) {
        outputImage();
        getBoundary();
        outputBoundary();
        getRegion();
        outputRegion();
      }
    }

    //feedback
    if (inputInt == 171) {

      String inputString = "";

      while (true) {
        print(',');
        if (myPort.available() > 0) {
          inputString = myPort.readStringUntil('\n');
          break;
        }
      }
      if (inputString != null) {
        println(inputString);
      }
      delay(1);
    }
  }
}