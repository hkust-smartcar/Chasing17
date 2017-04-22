String[] btName;
int btSize = 8;

int buttonDiameter = 50;
int leftX = 100, leftY = 600;
int middleX = 200, middleY = 600;
int rightX = 300, rightY = 600;
int modeButtonX = 1100, modeButtonY = 30;
boolean leftOver = false, middleOver = false, rightOver = false;
boolean modeButtonOver = false;
boolean tuneButtonOver = false;

void buttonSetUp() {

  btName[0] = "mode";
  btName[1] = "moveUp";
  btName[2] = "moveDown";
  btName[3] = "delete";
  btName[4] = "saveImage";
  btName[5] = "refresh";
  btName[6] = "tune";
  btName[7] = "rename";

  cp5.addButton(btName[0])
    .setPosition(1100, 30)
    .setSize(50, 20)
    ;
    
  cp5.addButton(btName[1])
    .setPosition(80, 480)
    .setSize(50, 20)
    ;
  
  cp5.addButton(btName[2])
    .setPosition(80, 520)
    .setSize(50, 20)
    ;

  cp5.addButton(btName[3])
    .setPosition(80, 560)
    .setSize(50, 20)
    ;

  cp5.addButton(btName[4])
    .setPosition(80, 600)
    .setSize(50, 20)
    ;

  cp5.addButton(btName[5])
    .setPosition(1100, 50)
    .setSize(50, 20)
    ;
    
  cp5.addButton(btName[6])
    .setPosition(1100, 600)
    .setSize(50, 20)
    ;
    
  cp5.addButton(btName[7])
    .setPosition(80, 440)
    .setSize(50, 20)
    ;
    
  cp5.get(Button.class, btName[1]).hide();
  cp5.get(Button.class, btName[2]).hide();
  cp5.get(Button.class, btName[3]).hide();
  cp5.get(Button.class, btName[6]).hide();
  cp5.get(Button.class, btName[7]).hide();
}

void mode() {
  
  if (millis() > 1000) {
  
    background(#D3D3D3);
    viewMode = ++viewMode % 3;
  
    if (viewMode == 0) {
  
      graphWidth = 400;
      graphHeight = 250;
      graphOneX = 100;
      graphOneY = 320;
      graphTwoX = 600;
      graphTwoY = 320;
  
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
  
      cp5.get(Button.class, btName[4]).show();
      cp5.get(Button.class, btName[1]).hide();
      cp5.get(Button.class, btName[2]).hide();
      cp5.get(Button.class, btName[3]).hide();
      cp5.get(Button.class, btName[7]).hide();
      cp5.get(Textfield.class, "newName").hide();
      cp5.get(ScrollableList.class, "ImageList").hide();
      
      selectedIndex = -1;
      
    } else if (viewMode == 1) {
      
      graphOneY = 50;
      graphTwoY = 50;
      fill(#404040);
      stroke(0);
      rect(graphOneX, graphOneY, graphWidth, graphHeight);
      rect(graphTwoX, graphTwoY, graphWidth, graphHeight);
      stroke(255);
      strokeWeight(0);
      line(graphOneX+1, graphOneY+graphHeight/2, graphOneX+graphWidth-1, graphOneY+graphHeight/2);
      line(graphTwoX+1, graphTwoY+graphHeight/2, graphTwoX+graphWidth-1, graphTwoY+graphHeight/2);
  
      readConstant();
      displayConstant();
      tfShow();
  
      cp5.get(Button.class, btName[6]).show();
      cp5.get(Button.class, btName[4]).hide();
    } else if (viewMode == 2) {
      
      readImage();
      list = new String[imageName.size()];
      for (int i=0; i<imageName.size(); i++) {
        list[i] = imageName.get(i);
      }
      List l = Arrays.asList(list);
      cp5.get(ScrollableList.class, "ImageList").setItems(l);
      
      cp5.get(Button.class, btName[1]).show();
      cp5.get(Button.class, btName[2]).show();
      cp5.get(Button.class, btName[3]).show();
      cp5.get(Button.class, btName[7]).show();
      cp5.get(Textfield.class, "newName").show();
      cp5.get(ScrollableList.class, "ImageList").show();
      cp5.get(Button.class, btName[6]).hide();
      tfHide();
      
    }
  
  }
}

void refresh() {
  if (millis() > 1000) {
    myPort.clear();
  }
}

void saveImage() {
  if (millis() > 1000) {
    saveTo("imageData.txt");
  }
}

void tune() {
  if (millis() > 1000 && viewMode == 1) {
    editConstant();
    displayConstant();
    String sendStr = "";
    sendStr += constantArr[0];
    for (int i=1; i<consSize; i++) {
      sendStr += ",";
      sendStr += constantArr[i];
    }
    myPort.write('t' + sendStr + '\n');
  }
}

void rename() {
  if (millis() > 1000 && selectedIndex >= 0) {
    imageName.set(selectedIndex, "#" + cp5.get(Textfield.class,"newName").getText());
    saveData();
    
    list[selectedIndex] = imageName.get(selectedIndex);
    List l = Arrays.asList(list);
    cp5.get(ScrollableList.class, "ImageList").setItems(l);
    CColor c = new CColor();
    c.setBackground(color(255,0,0));
    cp5.get(ScrollableList.class, "ImageList").getItem(selectedIndex).put("color", c);
  }
}

void moveUp() {
  if (millis() > 1000 && selectedIndex >= 1) {
    String temp = imageName.get(selectedIndex-1);
    imageName.set(selectedIndex-1, imageName.get(selectedIndex));
    imageName.set(selectedIndex, temp);
    Boolean[][] imageTemp = new Boolean[camHeight][camWidth];
    imageTemp = imageData.get(selectedIndex-1);
    imageData.set(selectedIndex-1, imageData.get(selectedIndex));
    imageData.set(selectedIndex, imageTemp);
    saveData();
    
    list[selectedIndex] = imageName.get(selectedIndex);
    list[selectedIndex-1] = imageName.get(selectedIndex-1);
    List l = Arrays.asList(list);
    cp5.get(ScrollableList.class, "ImageList").setItems(l);
    
    selectedIndex--;
    CColor c = new CColor();
    c.setBackground(color(255,0,0));
    cp5.get(ScrollableList.class, "ImageList").getItem(selectedIndex).put("color", c);
  }
}

void moveDown() {
  if (millis() > 1000 && selectedIndex >= 0 && selectedIndex <= imageName.size()-2 ) {
    String temp = imageName.get(selectedIndex+1);
    imageName.set(selectedIndex+1, imageName.get(selectedIndex));
    imageName.set(selectedIndex, temp);
    Boolean[][] imageTemp = new Boolean[camHeight][camWidth];
    imageTemp = imageData.get(selectedIndex+1);
    imageData.set(selectedIndex+1, imageData.get(selectedIndex));
    imageData.set(selectedIndex, imageTemp);
    saveData();
    
    list[selectedIndex] = imageName.get(selectedIndex);
    list[selectedIndex+1] = imageName.get(selectedIndex+1);
    List l = Arrays.asList(list);
    cp5.get(ScrollableList.class, "ImageList").setItems(l);
    
    selectedIndex++;
    CColor c = new CColor();
    c.setBackground(color(255,0,0));
    cp5.get(ScrollableList.class, "ImageList").getItem(selectedIndex).put("color", c);
  }
}

void delete() {
  if (millis() > 1000 && selectedIndex >= 0) {
    imageName.remove(selectedIndex);
    imageData.remove(selectedIndex);
    saveData();
    
    list = new String[imageName.size()];
    for (int i=0; i<imageName.size(); i++) {
      list[i] = imageName.get(i);
    }
    List l = Arrays.asList(list);
    cp5.get(ScrollableList.class, "ImageList").setItems(l);
    selectedIndex = -1;
    background(#D3D3D3);
  }
}