#include <AccelStepper.h>

#define X_STEP 2
#define X_DIR 5

#define Y_STEP 4
#define Y_DIR 7

#define PEN 6
#define ENABLE 8
#define MAX_SPEED (200 * 1 * 2)

AccelStepper X(1, X_STEP, X_DIR);
AccelStepper Y(1, Y_STEP, Y_DIR);

enum InstructionType {
  penPower,
  velocityTime,
  printLine,
  STOP
};

struct Instruction {
  InstructionType type;
  String op;
};

void setup()
{

  pinMode(ENABLE, OUTPUT);
  analogWrite(PEN, 0);
  
  X.setMaxSpeed(MAX_SPEED * 5);
  Y.setMaxSpeed(MAX_SPEED * 2);

  Serial.begin(2000000);

}

bool printingLine = false;
int segmentLevels[512];
double segmentLength;

double dX = 0;
double dY = 0;
int currentPenPower = 0;
bool stopReceived = false;
void loop() {
  if (((X.currentPosition() >= dX && X.speed() > 0) || (X.currentPosition() <= dX && X.speed() < 0) || X.speed() == 0) && ((Y.currentPosition() >= dY && Y.speed() > 0) || (Y.currentPosition() <= dY && Y.speed() < 0) || Y.speed() == 0)){
    digitalWrite(ENABLE, HIGH);
    printingLine = false;
    Instruction nextInstruction = FetchInstruction();
    ExecuteInstruction(nextInstruction);
    RequestInstruction();
  }
  else {
    
    if (printingLine) {
      int segmentIndex = (int)floor(abs(X.currentPosition() / (segmentLength)));
      int newPower = segmentLevels[segmentIndex];
      if (newPower != currentPenPower){
        if(newPower == 255){
          digitalWrite(PEN, HIGH);
        }else{
          analogWrite(PEN, newPower);
        }
        currentPenPower = newPower;
      }
    }
    
    X.runSpeed();
    Y.runSpeed();
  }
}

void RequestInstruction() {
  if (Serial.availableForWrite() && !stopReceived) {
    Serial.write("+");   
  }
}

Instruction FetchInstruction() {
    String instruction = "";
    do {
    instruction = Serial.readStringUntil(';');
    } while (instruction.length() == 0);
    stopReceived = false;
    
    if (instruction.substring(0, 8) == "SetPower") {
      return (Instruction){penPower, instruction};
    }
    else if (instruction == "STOP"){
        stopReceived = true;
        return (Instruction){STOP, instruction};
    }
    else if (instruction.substring(0,9) == "PrintLine"){
      return (Instruction){printLine, instruction};
    }
    else {
      // All remaining possible inputs are velocity vectors
      return (Instruction){velocityTime, instruction};
    }
}

void ExecuteInstruction(Instruction instruction) {
  if (instruction.type == penPower) {
    int laserPower = instruction.op.substring(9, instruction.op.length()).toInt();
    if(laserPower == 255) {
      digitalWrite(PEN, HIGH);
    }
    else {
      analogWrite(PEN, laserPower);
    }
    currentPenPower = laserPower;
  }
  else if (instruction.type == STOP){
    X.setSpeed(0);
    Y.setSpeed(0);
  }
  else if (instruction.type == printLine) {
    int commaIndex = instruction.op.indexOf(',');
    double signedTime = instruction.op.substring(10, commaIndex).toDouble();
    int nominalWidth = instruction.op.substring(commaIndex + 1, instruction.op.length()).toInt();  

    int i = 0;
    while(i < nominalWidth){
      RequestInstruction();
      String data = "";
      do
        data = Serial.readStringUntil('!');
      while (data == "");
      int colonIndex = data.indexOf(':');
      int lvl = data.substring(0,colonIndex).toInt();
      int count = data.substring(colonIndex + 1, data.length() + 1).toInt();
      while(count > 0){
        segmentLevels[i] = lvl;
        i++;
        count--;
      }
    }
    
    dX = signedTime *  MAX_SPEED / 1000;
    segmentLength = dX / nominalWidth;

    printingLine = true;
    
    digitalWrite(ENABLE, LOW);
    X.setCurrentPosition(0);
    X.setSpeed(signedTime > 0 ? MAX_SPEED * 3 : -MAX_SPEED * 3);
    Y.setSpeed(0);
  }
  else if (instruction.type == velocityTime) {
    int commaIndex0 = instruction.op.indexOf(',');
    int commaIndex1 = instruction.op.lastIndexOf(',');
    double vX = instruction.op.substring(1, commaIndex0).toDouble();
    double vY = instruction.op.substring(commaIndex0 + 1, commaIndex1).toDouble() * -1;
    double t = instruction.op.substring(commaIndex1 + 1, instruction.op.length()).toDouble();
    
    dX = vX * t * MAX_SPEED / 1000;
    dY = vY * t * MAX_SPEED / 1000;
    
    digitalWrite(ENABLE, LOW);
    X.setCurrentPosition(0);
    Y.setCurrentPosition(0);
    X.setSpeed(MAX_SPEED * vX);
    Y.setSpeed(MAX_SPEED * vY);
  }
}
