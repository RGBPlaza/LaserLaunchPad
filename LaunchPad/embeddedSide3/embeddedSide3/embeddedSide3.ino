#include <AccelStepper.h>

#define X_STEP 2
#define X_DIR 5

#define YA_STEP 3
#define YA_DIR 6

#define YB_STEP 4
#define YB_DIR 7

#define PEN 11
#define ENABLE 8
#define MAX_SPEED (200 * 4 * 3)

AccelStepper X(1, X_STEP, X_DIR);
AccelStepper YA(1, YA_STEP, YA_DIR);
AccelStepper YB(1, YB_STEP, YB_DIR);

bool penDown = false;

void setup()
{

  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  analogWrite(PEN, 0);

  X.setMaxSpeed(MAX_SPEED);
  X.setSpeed(0);
  YA.setMaxSpeed(MAX_SPEED);
  YA.setSpeed(0);
  YB.setMaxSpeed(MAX_SPEED);
  YB.setSpeed(0);

  Serial.begin(115200);

}

void loop() {
    X.runSpeed();
    YA.runSpeed();
    YB.runSpeed();
}

void serialEvent() {
  if (Serial.available() > 0) {
    // All strings sent to arduino end with ';'
    String instruction = Serial.readStringUntil(';');
    ExecuteInstruction(instruction);
  }
}

void ExecuteInstruction(String instruction){
  if (instruction.substring(0, 8) == "SetPower") {
    int laserPower = instruction.substring(9, instruction.length()).toInt();
    analogWrite(PEN, laserPower);
    penDown = laserPower > 0;
  }
  else {
    // All remaining possible inputs are velocity vectors
    int commaIndex = instruction.indexOf(',');
    double vX = instruction.substring(1, commaIndex).toDouble();
    double vY = instruction.substring(commaIndex + 1, instruction.length()).toDouble();
    X.setSpeed(MAX_SPEED * vX);
    YA.setSpeed(MAX_SPEED * vY);
    YB.setSpeed(MAX_SPEED * vY);
  }
}
