List<Boolean[][]> imageData;
List<String> imageName;
String[] list;
ScrollableList scrollList;
int selectedIndex = -1;

void listSetUp() {

  list = new String[imageName.size()];
  for (int i=0; i<imageName.size(); i++) {
    list[i] = imageName.get(i);
  }
  List l = Arrays.asList(list);

  scrollList = cp5.addScrollableList("ImageList")
    .setType(ScrollableList.LIST)
    .setPosition(60, 100)
    .setSize(150, 240)
    .setBarHeight(20)
    .setItemHeight(20)
    .addItems(l)
    ;

  cp5.addTextfield("newName")
    .setPosition(80, 400)
    .setWidth(100)
    .setColorLabel(0)
    .setAutoClear(false)
    ;
    
  
  cp5.get(Textfield.class, "newName").hide();
  cp5.get(ScrollableList.class, "ImageList").hide();
}

void ImageList(int n) {

  selectedIndex = n;

  for (int y=0; y<camHeight; y++) {
    for (int x=0; x<camWidth; x++) {
      if (imageData.get(n)[y][x]) {
        fill(0);
      } else {
        fill(255);
      }
      noStroke();
      rect(280 + pixelSide*x, 200 + pixelSide*y, pixelSide, pixelSide);
    }
  }

  CColor c = new CColor();
  c.setBackground(color(0, 45, 90));
  for (int i=0; i<imageName.size(); i++) {
    cp5.get(ScrollableList.class, "ImageList").getItem(i).put("color", c);
  }
  
  trytry();
}

void readImage() {

  String line;

  imageData.clear();
  imageName.clear();
  Boolean[][] imageTemp;

  reader = createReader("imageData.txt");

  do {

    try {
      line = reader.readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line = null;
    }

    if (line != null) {
      if (line.charAt(0) == '#') {
        imageName.add(line);
        continue;
      }
      if (line.length() <= 1) {
        continue;
      }
      imageTemp = new Boolean[camHeight][camWidth];
      for (int i=0; i<camHeight*camWidth; i++) {
        if (line.charAt(i) == '1') {
          imageTemp[i/camWidth][i%camWidth] = true;
        } else {
          imageTemp[i/camWidth][i%camWidth] = false;
        }
      }
      imageData.add(imageTemp);
    }
  } while (line != null);
}

void saveTo(String filename) {

  List<String> stringList = new ArrayList();
  String line;

  reader = createReader(filename);

  do {

    try {
      line = reader.readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line = null;
    }

    if (line != null) {
      stringList.add(line);
    }
  } while (line != null);

  writer = createWriter(filename);

  for (int i=0; i<stringList.size(); i++) {
    writer.println(stringList.get(i));
  }
  stringList.clear();

  writer.println("#Image");
  for (int y = 0; y < camHeight; y++) {
    for (int x = 0; x < camWidth; x++) {
      writer.print(pixelArray[y][x]?1:0);
    }
  }
  writer.println();

  writer.flush();
  writer.close();
}

void saveData() {
  writer = createWriter("imageData.txt");

  for (int i=0; i<imageName.size(); i++) {
    writer.println(imageName.get(i));
    for (int y = 0; y < camHeight; y++) {
      for (int x = 0; x < camWidth; x++) {
        writer.print(imageData.get(i)[y][x]?1:0);
      }
    }
    writer.println();
  }

  writer.flush();
  writer.close();
}