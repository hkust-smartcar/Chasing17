import processing.serial.*;
import java.util.*;

Serial myPort;
PrintWriter output;

int cnt = 0;

void setup() {
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.buffer(1);
  size(1000, 650);
  background(211,211,211);

  textSize(32);
  fill(0);
  text("Start", 150, 150);
  


}

void draw() {
  if (myPort.available() > 0) {
    int input = myPort.read();
    int filecnt = 0;
    String[] d = new String[128*480];
    while (input == 170 && cnt <= 128*480 - 1){
       while (myPort.available() > 0){
         int data = myPort.read();
         for (int i = 0; i < 8; i++){
           Integer dd = (data >> (7-i) & 1);
           d[cnt] = dd.toString();
           //saveStrings("info.txt", d);
           cnt++;  
         }
      }
      println(cnt);
      
    }
    
    saveStrings("info_0.txt", d);
    //filecnt++;
    print(filecnt); println(" done.");
    while (true);
  } 
}