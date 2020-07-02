// Blueberry Hardware code for arduino nano, ATmega328P (old bootloader)
// Reads commands from serial 

//Imports
#include <Servo.h>  //arduino library
#include <math.h>   //standard c library

//globals
struct servoGoal {
  int base;
  int shoulder;
  int elbow;
  int wRot;
  int wFlex;
  int grip;
};

Servo baseServo;  
Servo shoulderServo;  
Servo elbowServo; 
Servo wristRotServo;
Servo wristFlexServo;
Servo gripperServo;

struct servoGoal desired; //desired angles of the servos


//Const params
const int DELAY = 15;
const int DECAY_FACTOR = 6;
const int MIN_STEP = 2;


//+++++++++++++++FUNCTION DECLARATIONS+++++++++++++++++++++++++++
int stepServos();
int stepOneServo(Servo s, int goalPos);
int snapOneServo(Servo s, int goalPos);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void setup()
{ 
  Serial.begin(9600);
  //Attach servos to correct pin
  baseServo.attach(2);       
  shoulderServo.attach(3);
  elbowServo.attach(4);
  wristRotServo.attach(5);
  wristFlexServo.attach(6);
  gripperServo.attach(8);
  
  Serial.setTimeout(50);      //ensures the the arduino does not read serial for too long
  Serial.Print("s");
} 

//primary arduino loop
void loop() 
{ 
  while (!Serial.available()) {
    delay(DELAY);
  }
  
  desired.base = Serial.parseInt();
  desired.shoulder = Serial.parseInt();
  desired.elbow = Serial.parseInt();
  desired.wRot = Serial.parseInt();
  desired.wFlex = Serial.parseInt();
  desired.grip = Serial.parseInt();

  if(Serial.read() == '\n'){              // if the last byte is '\n' then stop reading and execute command 'd' stands for 'done'
      Serial.flush();                     //clear all other commands piled in the buffer                //send completion of the command
  }

  int servoCheck = 0;

  while (servoCheck < 6){
    servoCheck = stepServos();
    delay(DELAY);
  }
  Serial.print('d');  
}

//++++++++++++++++++++++++++++++FUNCTION DEFINITIONS++++++++++++++++++++++++++++++++++++++++++

int stepServos() {
  int completedServos = 0;
  completedServos += stepOneServo(baseServo, desired.base);
  completedServos += stepOneServo(shoulderServo, desired.shoulder);
  completedServos += stepOneServo(elbowServo, desired.elbow);
  completedServos += stepOneServo(wristRotServo, desired.wRot);
  completedServos += stepOneServo(wristFlexServo, desired.wFlex);
  completedServos += stepOneServo(gripperServo, desired.grip);
  return completedServos;
}

int stepOneServo(Servo s, int goalPos) {
  
    int startPos = s.read();        //read the current pos

    if (goalPos == startPos) {
      return 1;
    }

    if (goalPos > startPos) {
      int newStep = startPos + 1;
      s.write(newStep);
    } else {
      int newStep = startPos - 1;
      s.write(newStep);
    }
    
    return 0;
}

int snapOneServo(Servo s, int goalPos) {
  
    int startPos = s.read();        //read the current pos

    if (abs(goalPos - startPos) <= MIN_STEP) {
      return 1;
    }

    int potentialStep = (goalPos - startPos)/DECAY_FACTOR;

    if (abs(potentialStep) >= MIN_STEP) {
      s.write(startPos + potentialStep);
    } else {
      if (goalPos > startPos) {
        int newStep = startPos + MIN_STEP;
        s.write(min(goalPos, newStep));
      } else {
        int newStep = startPos - MIN_STEP;
        s.write(max(goalPos, newStep));
      }
    }
    
    return 0;
}
