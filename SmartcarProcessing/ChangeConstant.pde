String[] tfName;
String[] constantArr;
int textFieldSize = 16;
int tuneButtonX = 900, tuneButtonY = 500;
int textFieldX = 100, textFieldY = 400;

void tfSetUp() {

  for (int i=0; i<textFieldSize; i++) {
    tfName[i] = "Constant" + Integer.toString(i+1);
  }
  tfName[0] = "angle";
  tfName[1] = "powAngP";
  tfName[2] = "powAngI";
  tfName[3] = "powAngD";
  tfName[4] = "normalDiff";
  tfName[5] = "targetSpeed";
  tfName[6] = "speedAngFactor";
  tfName[7] = "preSpeedProp";
  tfName[8] = "errorRange";
  
  

  for (int i=0; i<textFieldSize; i++) {
    cp5.addTextfield(tfName[i])
      .setPosition(textFieldX + (i%4)*250, textFieldY + (i/4)*50)
      .setWidth(150)
      .setColorLabel(0)
      .setAutoClear(false)
      ;
  }

  tfHide();
}

void readConstant() {

  int counter = 0;
  String line;

  reader = createReader("constant.txt");

  do {

    try {
      line = reader.readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line = null;
    }

    if (line != null) {
      constantArr[counter] = line;
      counter++;
    }
  } while (line != null);
}

void editConstant() {

  for (int i=0; i<textFieldSize; i++) {
    if (cp5.get(Textfield.class, tfName[i]).getText().length() != 0) {
      constantArr[i] = cp5.get(Textfield.class, tfName[i]).getText();
    }
  }

  writer = createWriter("constant.txt");

  for (int i=0; i<textFieldSize; i++) {
    writer.println(constantArr[i]);
  }

  writer.flush();
  writer.close();
}

void displayConstant() {

  for (int i=0; i<consSize; i++) {
    displayText(constantArr[i], textFieldX+(i%4)*250+160, textFieldY+50*(i/4)+10, 100, 24);
  }
}

void tfShow() {

  for (int i=0; i<textFieldSize; i++) {
    cp5.get(Textfield.class, tfName[i]).show();
  }
}

void tfHide() {

  for (int i=0; i<textFieldSize; i++) {
    cp5.get(Textfield.class, tfName[i]).hide();
  }
}