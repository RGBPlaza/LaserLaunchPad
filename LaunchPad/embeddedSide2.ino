#define X0 4
#define X1 5
#define X2 6
#define X3 7

#define Y0 9
#define Y1 10
#define Y2 11
#define Y3 12

#define PEN 3

String instruction;
bool awaitingInstruction = false;
bool penDown = false;
double posX = 0;
double posY = 0;
double vX = 0;
double vY = 0;
double t = 0;
double sX = 0;
double sY = 0;

void setup()
{
  pinMode(X0, OUTPUT);
  pinMode(X1, OUTPUT);
  pinMode(X2, OUTPUT);
  pinMode(X3, OUTPUT);

  pinMode(Y0, OUTPUT);
  pinMode(Y1, OUTPUT);
  pinMode(Y2, OUTPUT);
  pinMode(Y3, OUTPUT);

  analogWrite(PEN, 0);

  Serial.begin(9600);

}

unsigned int timeCounter = 0;
void loop(){
  if(!awaitingInstruction){
    // X-AXIS
    if(vX != 0){
      // Set new motor phases
      if(sX < 0) {
        sX += 8;
      }
      if(sX > 8){
        sX -= 8; 
      }
  
      // Change motor position
      SetPhaseX((int)floor(sX));
      
      sX += vX;
  
      // Update distance travelled
      posX += (vX/1000);
    }

    // Y-AXIS
    if(vY != 0){
      // Set new motor phases
      if(sY < 0) {
        sY += 8;
      }
      if(sY > 8){
        sY -= 8; 
      }
  
      // Change motor position
      SetPhaseY((int)floor(sY));
      
      // Takeaway velocity (rather than add) due to motor direction.
      sY -= vY;
  
      // Update distance travelled
      posY += (vY/1000);
    }
    
    if(timeCounter >= t){
      //SendPosition();
      //vX = 0;
      //vY = 0;
      RequestInstruction();
    }
    else {
      timeCounter++;
    }
  }
  delay(1);
}

void SendPosition(){
  if(Serial.availableForWrite()){
    char penState = penDown ? '1' : '0';
    String msg = '(' + String(posX,2) + ',' + String(posY,2) + ',' + penState + ')';
    Serial.print(msg);
  }
}

void RequestInstruction(){
  if(Serial.availableForWrite()){
    Serial.write(":)");
    awaitingInstruction = true;
  }
}

void serialEvent() {
  if (Serial.available() > 0 && awaitingInstruction) {
    // All strings sent to arduino end with ')'
    instruction = Serial.readStringUntil(';');
    if (instruction.substring(0,8) == "SetPower") {
      vX = 0;
      vY = 0;
      int laserPower = instruction.substring(9,instruction.length()).toInt();
      analogWrite(PEN, laserPower);
      penDown = laserPower > 0;
      RequestInstruction();
    }
    else { 
      // All remaining possible inputs are velocity vectors
      int commaIndex0 = instruction.indexOf(',');
      int commaIndex1 = instruction.lastIndexOf(',');
      vX = instruction.substring(1, commaIndex0).toDouble() * 1.5;
      vY = instruction.substring(commaIndex0 + 1, commaIndex1).toDouble() * 1.5;
      t = instruction.substring(commaIndex1 + 1, instruction.length()).toDouble();
      timeCounter = 0;
      awaitingInstruction = false;
    }
  }
}
void SetPhaseX(int s) {
  switch (s) {
    case 0:
      digitalWrite(X0, LOW);
      digitalWrite(X1, LOW);
      digitalWrite(X2, LOW);
      digitalWrite(X3, HIGH);
      break;
    case 1:
      digitalWrite(X0, LOW);
      digitalWrite(X1, LOW);
      digitalWrite(X2, HIGH);
      digitalWrite(X3, HIGH);
      break;
    case 2:
      digitalWrite(X0, LOW);
      digitalWrite(X1, LOW);
      digitalWrite(X2, HIGH);
      digitalWrite(X3, LOW);
      break;
    case 3:
      digitalWrite(X0, LOW);
      digitalWrite(X1, HIGH);
      digitalWrite(X2, HIGH);
      digitalWrite(X3, LOW);
      break;
    case 4:
      digitalWrite(X0, LOW);
      digitalWrite(X1, HIGH);
      digitalWrite(X2, LOW);
      digitalWrite(X3, LOW);
      break;
    case 5:
      digitalWrite(X0, HIGH);
      digitalWrite(X1, HIGH);
      digitalWrite(X2, LOW);
      digitalWrite(X3, LOW);
      break;
    case 6:
      digitalWrite(X0, HIGH);
      digitalWrite(X1, LOW);
      digitalWrite(X2, LOW);
      digitalWrite(X3, LOW);
      break;
    case 7:
      digitalWrite(X0, HIGH);
      digitalWrite(X1, LOW);
      digitalWrite(X2, LOW);
      digitalWrite(X3, HIGH);
      break;
    default:
      digitalWrite(X0, LOW);
      digitalWrite(X1, LOW);
      digitalWrite(X2, LOW);
      digitalWrite(X3, LOW);
      break;
  }
}

void SetPhaseY(int s) {
  switch (s) {
    case 0:
      digitalWrite(Y0, LOW);
      digitalWrite(Y1, LOW);
      digitalWrite(Y2, LOW);
      digitalWrite(Y3, HIGH);
      break;
    case 1:
      digitalWrite(Y0, LOW);
      digitalWrite(Y1, LOW);
      digitalWrite(Y2, HIGH);
      digitalWrite(Y3, HIGH);
      break;
    case 2:
      digitalWrite(Y0, LOW);
      digitalWrite(Y1, LOW);
      digitalWrite(Y2, HIGH);
      digitalWrite(Y3, LOW);
      break;
    case 3:
      digitalWrite(Y0, LOW);
      digitalWrite(Y1, HIGH);
      digitalWrite(Y2, HIGH);
      digitalWrite(Y3, LOW);
      break;
    case 4:
      digitalWrite(Y0, LOW);
      digitalWrite(Y1, HIGH);
      digitalWrite(Y2, LOW);
      digitalWrite(Y3, LOW);
      break;
    case 5:
      digitalWrite(Y0, HIGH);
      digitalWrite(Y1, HIGH);
      digitalWrite(Y2, LOW);
      digitalWrite(Y3, LOW);
      break;
    case 6:
      digitalWrite(Y0, HIGH);
      digitalWrite(Y1, LOW);
      digitalWrite(Y2, LOW);
      digitalWrite(Y3, LOW);
      break;
    case 7:
      digitalWrite(Y0, HIGH);
      digitalWrite(Y1, LOW);
      digitalWrite(Y2, LOW);
      digitalWrite(Y3, HIGH);
      break;
    default:
      digitalWrite(Y0, LOW);
      digitalWrite(Y1, LOW);
      digitalWrite(Y2, LOW);
      digitalWrite(Y3, LOW);
      break;
  }
}
