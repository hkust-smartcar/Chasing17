import processing.serial.*;
import java.util.*;

Serial myPort;

void setup() {
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
  println("Start: ");
  
  myPort.write("AT");   delay(100);
  myPort.write("AT+RESET");
  delay(100);
  myPort.write("AT+ROLES");   delay(100);
  myPort.write("AT+BAUD4");   delay(100);
  myPort.write("AT+NAMEPTSlave4.0");   delay(100);
  myPort.write("AT+PIN123456");   delay(100);
}

void draw() {
  if (myPort.available() > 0) {
    delay(100);
    println(myPort.readString());
  } 
}