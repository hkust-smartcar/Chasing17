double[][] value;
double[] newValue;
int valSize = 0;
int middle = 2;
Boolean started = false;

int graphWidth = 400, graphHeight = 250;
int graphOneX = 100, graphOneY = 320;
int graphTwoX = 600, graphTwoY = 320;

void stringToDouble(String inputString) {
  
  String[] valStr1 = inputString.split("=")[0].split(",");
  String[] valStr2 = inputString.split("=")[1].split(",");
  int counter = 0;
  
  if (!started) {
    started = true;
    valSize = valStr1.length+valStr2.length;
    value = new double[valSize][graphWidth];
    newValue = new double[valSize];
    for (int i = 0; i < valSize; i++) {
      newValue[i] = 0;
      for (int j = 0; j < graphWidth; j++) {
        value[i][j] = 0;
      }
    }
  }
  
  for (int i = 1; i < valStr1.length; i++) {
    valStr1[i] = valStr1[i].replaceAll("[^0-9.-]+", "");
    newValue[counter] = Double.parseDouble(valStr1[i]);
    counter++;
  }
  middle = counter;
  
  for (int i = 0; i < valStr2.length; i++) {
    valStr2[i] = valStr2[i].replaceAll("[^0-9.-]+", "");
    newValue[counter] = Double.parseDouble(valStr2[i]);
    counter++;
  }
  
}

void getData() {
  
  for (int i = 0; i < valSize; i++) {
    for (int j = 1; j < graphWidth; j++) {
      value[i][j-1] = value[i][j];
    }
  }
  
  for (int i = 0; i < middle; i++) {
    if (newValue[i] > range1) {
      newValue[i] = range1;
    } else if (newValue[i] < 0-range1) {
      newValue[i] = 0-range1;
    }
    value[i][graphWidth-1] = map((float)newValue[i], 0-range1, range1, -10000, 10000);
  }
  
  for (int i = middle; i < valSize; i++) {
    if (newValue[i] > range2) {
      newValue[i] = range2;
    } else if (newValue[i] < 0-range2) {
      newValue[i] = 0-range2;
    }
    value[i][graphWidth-1] = map((float)newValue[i], 0-range2, range2, -10000, 10000);
  }
  
}

void plotGraph() {

  if (viewMode == 0 || viewMode == 1) {
    fill(#404040);
    stroke(0);
    rect(graphOneX, graphOneY, graphWidth, graphHeight);
    rect(graphTwoX, graphTwoY, graphWidth, graphHeight);
    stroke(255);
    strokeWeight(0);
    line(graphOneX+1, graphOneY+graphHeight/2, graphOneX+graphWidth-1, graphOneY+graphHeight/2);
    line(graphTwoX+1, graphTwoY+graphHeight/2, graphTwoX+graphWidth-1, graphTwoY+graphHeight/2);
    //line(graphOneX+1, graphOneY+(graphHeight/2)*(float)(range-Double.parseDouble(constantArr[0]))/range, graphOneX+graphWidth-1, graphOneY+(graphHeight/2)*(float)(range-Double.parseDouble(constantArr[0]))/range);
    
  }
  
  strokeWeight(2);

  for (int i = 0; i < middle; i++) {
    stroke(colorArray[i]);
    for(int j=1; j<graphWidth; j++) {
      point(graphOneX+j, graphOneY+graphHeight/2-(int)(value[i][j]/10000*graphHeight/2)); //<>//
    }
  }
  
  for (int i = middle; i < valSize; i++) {
    stroke(colorArray[i]);
    for(int j=1; j<graphWidth; j++) {
      point(graphTwoX+j, graphTwoY+graphHeight/2-(int)(value[i][j]/10000*graphHeight/2));
    }
  }
}