#include <AccelStepper.h>
#include <MultiStepper.h>

#define X_STEP 2
#define X_DIR 5

#define YA_STEP 3
#define YA_DIR 6

#define YB_STEP 4
#define YB_DIR 7

#define PEN 11
#define ENABLE 8
#define MAX_SPEED (200 * 6)

AccelStepper X(1, X_STEP, X_DIR);
AccelStepper YA(1, YA_STEP, YA_DIR);
AccelStepper YB(1, YB_STEP, YB_DIR);

MultiStepper steppers;

String instruction;
bool awaitingInstruction = false;
bool penDown = false;
long positions[3];

void setup()
{

  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  analogWrite(PEN, 0);

  steppers.addStepper(X);
  steppers.addStepper(YA);
  steppers.addStepper(YB);

  Serial.begin(9600);

}

unsigned int timeCounter = 0;
void loop() {
  if (!awaitingInstruction) {
    steppers.runSpeedToPosition();
    RequestInstruction();
  }
}

void RequestInstruction() {
  if (Serial.availableForWrite()) {
    Serial.write(":)");
    awaitingInstruction = true;
  }
}

void serialEvent() {
  if (Serial.available() > 0 && awaitingInstruction) {
    // All strings sent to arduino end with ')'
    double dX, dY, vX, vY;
    instruction = Serial.readStringUntil(';');
    if (instruction.substring(0, 8) == "SetPower") {
      vX = 0;
      vY = 0;
      int laserPower = instruction.substring(9, instruction.length()).toInt();
      analogWrite(PEN, laserPower);
      penDown = laserPower > 0;
      RequestInstruction();
      return;
    }
    else if (instruction.substring(0, 6) == "MoveBy") {
      int commaIndex = instruction.indexOf(',');
      dX = instruction.substring(7, commaIndex).toDouble();
      dY = instruction.substring(commaIndex + 1, instruction.length()).toDouble();
      double theta = atan2(dY, dX);
      vX = cos(theta); vY = sin(theta);
    }
    else {
      // All remaining possible inputs are velocity vectors
      int commaIndex0 = instruction.indexOf(',');
      int commaIndex1 = instruction.lastIndexOf(',');
      vX = instruction.substring(1, commaIndex0).toDouble();
      vY = instruction.substring(commaIndex0 + 1, commaIndex1).toDouble();
      double t = instruction.substring(commaIndex1 + 1, instruction.length()).toDouble();
      dX = vX * t; dY = vY * t;
    }
    X.setCurrentPosition(0);
    X.setMaxSpeed(MAX_SPEED * abs(vX));
    YA.setCurrentPosition(0);
    YA.setMaxSpeed(MAX_SPEED * abs(vY));
    YB.setCurrentPosition(0);
    YB.setMaxSpeed(MAX_SPEED * abs(vY));
    positions[0] = dX; positions[1] = dY; positions[2] = dY;
    steppers.moveTo(positions);
    awaitingInstruction = false;
  }
}
