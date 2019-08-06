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

String currentInstruction = "";
String nextInstruction = "";
bool penDown = false;

void setup()
{

  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  analogWrite(PEN, 0);

  X.setMaxSpeed(MAX_SPEED);
  YA.setMaxSpeed(MAX_SPEED);
  YB.setMaxSpeed(MAX_SPEED);

  Serial.begin(115200);

  RequestInstruction();

}

double timeCounter = 0;
double t = 0;
void loop() {
  if (currentInstruction != "") {
    X.runSpeed();
    YA.runSpeed();
    YB.runSpeed();
    if (timeCounter >= t) {
      currentInstruction = nextInstruction;
      ExecuteCurrentInstruction();
      nextInstruction = "";
      RequestInstruction();
      timeCounter = 0;
    }
    timeCounter += 0.1;
    delayMicroseconds(1);
  }
  else {
    RequestInstruction();
    delay(50);
  }
}

void RequestInstruction() {
  if (Serial.availableForWrite()) {
    Serial.write(":)");
  }
}

void serialEvent() {
  if (Serial.available() > 0) {
    // All strings sent to arduino end with ';'
    String instruction = Serial.readStringUntil(';');
    if (currentInstruction == "") {
      currentInstruction = instruction;
      ExecuteCurrentInstruction();
      RequestInstruction();
    }
    else if (nextInstruction == "") {
      nextInstruction = instruction;
    }
  }
}

void ExecuteCurrentInstruction(){
  if (currentInstruction.substring(0, 8) == "SetPower") {
    int laserPower = currentInstruction.substring(9, currentInstruction.length()).toInt();
    analogWrite(PEN, laserPower);
    penDown = laserPower > 0;
  }
  else {
    // All remaining possible inputs are velocity vectors
    int commaIndex0 = currentInstruction.indexOf(',');
    int commaIndex1 = currentInstruction.lastIndexOf(',');
    double vX = currentInstruction.substring(1, commaIndex0).toDouble();
    double vY = currentInstruction.substring(commaIndex0 + 1, commaIndex1).toDouble();
    t = currentInstruction.substring(commaIndex1 + 1, currentInstruction.length()).toDouble();
    X.setSpeed(MAX_SPEED * vX);
    YA.setSpeed(MAX_SPEED * vY);
    YB.setSpeed(MAX_SPEED * vY);
  }
}
