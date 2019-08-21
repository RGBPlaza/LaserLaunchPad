#include <AccelStepper.h>

#define X_STEP 2
#define X_DIR 5

#define YA_STEP 3
#define YA_DIR 6

#define YB_STEP 4
#define YB_DIR 7

#define PEN 11
#define ENABLE 8
#define MAX_SPEED (200 * 4 * 4)

AccelStepper X(1, X_STEP, X_DIR);
AccelStepper YA(1, YA_STEP, YA_DIR);
AccelStepper YB(1, YB_STEP, YB_DIR);

struct Instruction {
  double vX;
  double vY;
  double t;
};

void setup()
{

  pinMode(ENABLE, OUTPUT);
  analogWrite(PEN, 0);
  
  X.setMaxSpeed(MAX_SPEED);
  YA.setMaxSpeed(MAX_SPEED);
  YB.setMaxSpeed(MAX_SPEED);

  Serial.begin(2000000);

}

double dX = 0;
double dY = 0;
bool stopReceived = false;
void loop() {
  if (((X.currentPosition() >= dX && X.speed() > 0) || (X.currentPosition() <= dX && X.speed() < 0) || X.speed() == 0) && ((YA.currentPosition() >= dY && YA.speed() > 0) || (YA.currentPosition() <= dY && YA.speed() < 0) || YA.speed() == 0)){
    digitalWrite(ENABLE,HIGH);
    Instruction nextInstruction = FetchInstruction();
    if(!stopReceived)
      RequestInstruction();
    else
      stopReceived = false;
    ExecuteInstruction(nextInstruction);
  }
  else {
    X.runSpeed();
    YA.runSpeed();
    YB.runSpeed();
  }
}

void RequestInstruction() {
  if (Serial.availableForWrite() && !stopReceived) {
    Serial.write(":)");   
  }
}

Instruction FetchInstruction() {
    String instruction = "";
    do {
    instruction = Serial.readStringUntil(';');
    } while (instruction.length() == 0);
    if (instruction.substring(0, 8) == "SetPower") {
      double laserPower = instruction.substring(9, instruction.length()).toDouble();
      return (Instruction){laserPower, 0, -1};
    }
    else if (instruction == "STOP"){
        stopReceived = true;
        return (Instruction){0, 0, 0};
    }
    else {
      // All remaining possible inputs are velocity vectors
      int commaIndex0 = instruction.indexOf(',');
      int commaIndex1 = instruction.lastIndexOf(',');
      double vX = instruction.substring(1, commaIndex0).toDouble();
      double vY = instruction.substring(commaIndex0 + 1, commaIndex1).toDouble() * -1;
      double t = instruction.substring(commaIndex1 + 1, instruction.length()).toDouble();
      return (Instruction){vX, vY, t};
    }
}

void ExecuteInstruction(Instruction instruction) {
  if (instruction.t <= 0) {
    int laserPower = (int)instruction.vX;
    if (laserPower == 255)
      digitalWrite(PEN, HIGH);
    else
      analogWrite(PEN, laserPower);
  }
  else {
    digitalWrite(ENABLE, LOW);
    dX = instruction.vX * instruction.t * MAX_SPEED / 1000;
    dY = instruction.vY * instruction.t * MAX_SPEED / 1000;
    X.setCurrentPosition(0);
    YA.setCurrentPosition(0);
    YB.setCurrentPosition(0);
    X.setSpeed(MAX_SPEED * instruction.vX);
    YA.setSpeed(MAX_SPEED * instruction.vY);
    YB.setSpeed(MAX_SPEED * instruction.vY);
  }
}
