#include <AccelStepper.h>
#include <Queue.h>

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
DataQueue<Instruction> instructions(400);

void setup()
{

  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  analogWrite(PEN, 0);
  
  X.setMaxSpeed(MAX_SPEED);
  YA.setMaxSpeed(MAX_SPEED);
  YB.setMaxSpeed(MAX_SPEED);

  Serial.begin(115200);

}

double dX = 0;
double dY = 0;
bool stopReceived = false;
bool silenceRequested = false;
void loop() {
  if (((X.currentPosition() >= dX && X.speed() > 0) || (X.currentPosition() <= dX && X.speed() < 0) || X.speed() == 0) && ((YA.currentPosition() >= dY && YA.speed() > 0) || (YA.currentPosition() <= dY && YA.speed() < 0) || YA.speed() == 0)){
    X.setSpeed(0);
    YA.setSpeed(0);
    YB.setSpeed(0);
  }
  if(X.speed() == 0 && YA.speed() == 0 && !instructions.isEmpty()){
      ExecuteInstruction(instructions.dequeue());
  }
  else {
    X.runSpeed();
    YA.runSpeed();
    YB.runSpeed();
  }
}

void RequestInstructions() {
  if (Serial.availableForWrite() && !instructions.isFull() && !stopReceived) {
    Serial.write(":)");   
  }
}

void RequestSilence() {
  if (Serial.availableForWrite()) {
    Serial.write(":(");   
  }
}

void ConfirmFinished() {
  if (Serial.availableForWrite()) {
    Serial.write("(:");   
  }
}

void serialEvent() {
  if (Serial.available() > 0) {
    // All strings sent to arduino end with ';'
    String instruction = Serial.readStringUntil(';');
    stopReceived = false;
    if (instruction.substring(0, 8) == "SetPower") {
      double laserPower = instruction.substring(9, instruction.length()).toDouble();
      instructions.enqueue({laserPower, 0, -1});
    }
    else if (instruction == "STOP"){
        instructions.enqueue({0, 0, 0});
        stopReceived = true;
    }
    else {
      // All remaining possible inputs are velocity vectors
      int commaIndex0 = instruction.indexOf(',');
      int commaIndex1 = instruction.lastIndexOf(',');
      double vX = instruction.substring(1, commaIndex0).toDouble();
      double vY = instruction.substring(commaIndex0 + 1, commaIndex1).toDouble() * -1;
      double t = instruction.substring(commaIndex1 + 1, instruction.length()).toDouble() * 4;
      instructions.enqueue({vX, vY, t});
    }
    if (instructions.item_count() >= 40 && !silenceRequested){
      RequestSilence();
      silenceRequested = true;
    }
  }
}

void ExecuteInstruction(Instruction instruction) {
  if (instruction.t <= 0) {
    int laserPower = (int)instruction.vX;
    analogWrite(PEN, laserPower);
  }
  else {
    dX = instruction.vX * instruction.t;
    dY = instruction.vY * instruction.t;
    X.setCurrentPosition(0);
    YA.setCurrentPosition(0);
    YB.setCurrentPosition(0);
    X.setSpeed(MAX_SPEED * instruction.vX);
    YA.setSpeed(MAX_SPEED * instruction.vY);
    YB.setSpeed(MAX_SPEED * instruction.vY);
  }
  if(instructions.item_count() < 40 && !stopReceived && silenceRequested){
    RequestInstructions();
    silenceRequested = false;
  }
  if(instructions.isEmpty() && stopReceived){
    ConfirmFinished();
  }
}
