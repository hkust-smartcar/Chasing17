int camWidth = 80, camHeight = 60;
int arrayPosX, arrayPosY;

int pixelSide = 4;
Boolean[][] pixelArray = new Boolean[camHeight][camWidth];
int imageX = 50, imageY = 40;

Boolean[][] boundaryArray = new Boolean[camHeight][camWidth];
int boundaryX = 750, boundaryY = 40;

int regionSide = 5;
int regionSize = (camHeight*camWidth) / (regionSide*regionSide);
int[] regionArray = new int[regionSize];
int regionX = 400, regionY = 40;

void getImage(int data) {

  String binData = "";
  int strLen = 0;

  binData = Integer.toBinaryString(data);
  strLen = binData.length();
  //println(binData);

  for (int i=0; i<8; i++) {

    if (i < strLen) {

      if (binData.charAt(strLen-1-i) == '1') {
        pixelArray[arrayPosY][arrayPosX+7-i] = true;
      } else {
        pixelArray[arrayPosY][arrayPosX+7-i] = false;
      }
    } else {
      pixelArray[arrayPosY][arrayPosX+7-i] = false;
    }
  }

  arrayPosX += 8;

  if (arrayPosX >= camWidth) {
    arrayPosX = 0;
    arrayPosY++;
  }
  if (arrayPosY >= camHeight) {
    arrayPosY = 0;
  }
}

void outputImage() {

  for (int y=0; y<camHeight; y++) {
    for (int x=0; x<camWidth; x++) {
      if (pixelArray[y][x]) {
        fill(0);
      } else {
        fill(255);
      }
      noStroke();
      rect(imageX + pixelSide*x, imageY + pixelSide*y, pixelSide, pixelSide);
    }
  }
}

void getBoundary() {

  Boolean previous;

  for (int y = 0; y < camHeight; y++) {

    previous = pixelArray[y][0];

    for (int x = 0; x < camWidth; x++) {

      if (pixelArray[y][x] != previous) {

        if (pixelArray[y][x] == false) {
          boundaryArray[y][x] = true;
        } else {
          boundaryArray[y][x-1] = true;
          boundaryArray[y][x] = false;
        }
        previous = pixelArray[y][x];
      } else {
        boundaryArray[y][x] = false;
      }
    }
  }

  for (int x = 0; x < camWidth; x++) {

    previous = pixelArray[0][x];

    for (int y = 0; y < camHeight; y++) {

      if (pixelArray[y][x] != previous) {

        if (pixelArray[y][x] == false) {
          boundaryArray[y][x] = true;
        } else {
          boundaryArray[y-1][x] = true;
          boundaryArray[y][x] = false;
        }
        previous = pixelArray[y][x];
      }
    }
  }
}

void outputBoundary() {

  for (int y=0; y<camHeight; y++) {
    for (int x=0; x<camWidth; x++) {
      if (boundaryArray[y][x]) {
        fill(0);
      } else {
        fill(255);
      }
      noStroke();
      rect(boundaryX + pixelSide*x, boundaryY + pixelSide*y, pixelSide, pixelSide);
    }
  }
}

void getRegion() {

  int counter = 0;

  for (int y=0; y<camHeight; y+=regionSide) {
    for (int x=0; x<camWidth; x+=regionSide) {

      int sum = 0;

      for (int subY=0; subY<regionSide; subY++) {
        for (int subX=0; subX<regionSide; subX++) {

          if (pixelArray[y+subY][x+subX]) {
            sum++;
          }
        }
      }

      regionArray[counter] = sum*10/(regionSide*regionSide);
      counter++;
    }
  }
}

void outputRegion() {

  int x, y;

  for (int i=0; i<regionSize; i++) {

    x = i % (camWidth/regionSide);
    y = i / (camWidth/regionSide);

    fill((int)((1 - ((double)regionArray[i]/10)) * 255));
    noStroke();
    rect(regionX + regionSide*pixelSide*x, regionY + regionSide*pixelSide*y, 
      regionSide*pixelSide, regionSide*pixelSide);
  }
}