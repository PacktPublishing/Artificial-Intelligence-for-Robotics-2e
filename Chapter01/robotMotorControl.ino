#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Timer.h>
#include <Event.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);
char inData[20]; // buffer for input commands
String inChar;  // input one character at a time
Timer timer;
boolean hbb;  // heartbeat fail indication
int timerEvent;
int hti; //heartbeat timeout interval

boolean between(int in, int lower, int upper) {
  boolean result = false;
  if ((in <= upper) && (in >= lower)) {
    result = true;
  }
   return result;
}

void clearHBB(){
  hbb = false;
  //Serial.write("TO\n");
  timerEvent = timer.after(hti,clearHBB);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
  

void ALLSTOP(){
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  Serial.write("All Stop\n");
}

void setup() {
  AFMS.begin();
  Serial.begin(115200);
  Serial.write("Power On Robot Motor Control\n");
  hti = 1000;
  timerEvent = timer.after(hti,clearHBB);
  
}


void loop(){
  



  timer.update();
  
  while (Serial.available()>0) {
    inChar = Serial.readString();
    Serial.print(inChar);
   ;
      if (inChar.charAt(0) =='S'){
        // stop!!
        Serial.print("STOP CMD\n");
        ALLSTOP();
      }
      // heartbeat - check this first 
      if (inChar.charAt(0)=='H'){
        // just put it right back out the serial port
        Serial.println("HBB\n");
        hbb = true;
        timer.stop(timerEvent); //cancel the last timer
        // set a timer that goes of in 2 seconds
        timerEvent = timer.after(hti,clearHBB);
      }
//     if (hbb==false) {
//         Serial.write("Heartbeat Timeout\n");
//          //heartbeat false means we have lost comms with the controller
//         
//         // don't process anything else
//         continue;
//        }
      if (inChar.charAt(0)=='D'){   // drive command
        hbb = true;
        timer.stop(timerEvent);
        timerEvent = timer.after(hti,clearHBB);
        uint8_t leftMotorCmd =(inChar.charAt(1));
        uint8_t rightMotorCmd = (inChar.charAt(2));
        Serial.print("MOTOR:");
        Serial.print(leftMotorCmd);
        Serial.print(rightMotorCmd);
        Serial.print("\n");
        if (between(leftMotorCmd,0,200)){
          // speed comes in as 0-200. 
          // convert to -100 to +100
          if (leftMotorCmd < 100){
            leftMotorCmd *=2.55;
            leftMotor->setSpeed(leftMotorCmd);
            leftMotor->run(FORWARD);
          }
          else if (leftMotorCmd >= 100) {
            leftMotorCmd -=100;
            leftMotorCmd *=2.55;
            leftMotor->setSpeed(leftMotorCmd);
            leftMotor->run(BACKWARD);
          }
          if (between(rightMotorCmd,0,200)){
          // speed comes in as 0-200. 
          // convert to -100 to +100
          if (rightMotorCmd < 100){
            rightMotorCmd *=2.55;
            rightMotor->setSpeed(rightMotorCmd);
            rightMotor->run(BACKWARD);
          }
          else if (rightMotorCmd >= 100) {
            rightMotorCmd -=100;
            rightMotorCmd *=2.55;
            rightMotor->setSpeed(rightMotorCmd);
            rightMotor->run(FORWARD);
          }
          Serial.print("Motor->");
          Serial.print(leftMotorCmd);
          Serial.print(rightMotorCmd);
          Serial.print("\n");
      }
    }
  } // end while 
}
}
