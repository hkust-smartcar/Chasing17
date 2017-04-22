import processing.serial.*;
import java.util.*;

Serial myPort;

byte cnt = 0;

void setup() {
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.buffer(1);
}

void draw() {
  myPort.write(cnt++);
  if (myPort.available() > 0){
    println(myPort.read());
  }
}