#define X0 4
#define X1 5
#define X2 6
#define X3 7

#define Y0 9
#define Y1 10
#define Y2 11
#define Y3 12

#define PEN 13

bool isFinishing = false;
String instruction;
bool penDown = false;
double posX = 0;
double posY = 0;

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

  pinMode(PEN, OUTPUT);

  Serial.begin(9600);
  Serial.write(":)");

}

void loop(){
  // 'Tis mostly an event driven program
}

void serialEvent() {
  if (Serial.available() > 0) {
    // All strings sent to arduino end with ')'
    instruction = Serial.readStringUntil(';');
    if (instruction == "That will do, cheers bud :)") {
      isFinishing = true;
      if (posX != 0 && posY != 0) {
        MoveTo(0,0, false);
      }
      if (penDown) {
        digitalWrite(PEN, LOW);
        penDown = false;
      }
      Serial.write("(:");
      Serial.end();
    }
    else if (instruction == "PenUp()") {
      if (penDown) {
        digitalWrite(PEN, LOW);
        penDown = false;
      }
      Serial.write(":)");
    }
    else if (instruction == "PenDown()") {
      if (!penDown) {
        digitalWrite(PEN, HIGH);
        penDown = true;
      }
      Serial.write(":)");
    }
    else { // All remaining possible inputs are coordinates
      int commaIndex = instruction.indexOf(',');
      double nextX = instruction.substring(1, commaIndex).toDouble();
      double nextY = instruction.substring(commaIndex+1, instruction.length() - 1).toDouble();
      MoveTo(nextX, nextY, penDown);
    }
    Serial.flush();
  }
}

double sX = 0;
double sY = 0;
void MoveTo(double nextX, double nextY, bool calcVelocity) {
  double vX;
  double vY;
  double diffX = round((nextX - posX) * 100) / 100;
  double diffY = round((nextY - posY) * 100) / 100;
  
  if(calcVelocity){  
    // Caluclate component velocities
    if (diffX != 0 && diffY != 0) {
      double theta = atan(diffY / diffX);
      vX = (diffX > 0) ? cos(theta): -cos(theta);
      //Serial.print(vX);
      //vX = (vX > 0.01) ? vX : 0; 
      vY = (diffY > 0) ? sin(theta) : -sin(theta);
      //Serial.print(vY);
      //vY = (vX > 0.01) ? vY : 0; 
    }
    else if (diffY == 0 && diffX != 0) {
      // Horizontal
      vX = (diffX > 0) ? 1 : -1;
      vY = 0;
    }
    else if (diffX == 0 && diffY != 0) {
      // Vertical
      vX = 0;
      vY = (diffY > 0) ? 1 : -1;
    }
    else {
      // Same Location
      vX = 0;
      vY = 0;
      Serial.write(":)");
      return;
    }
  }
  else {
    vX = (diffX > 0) ? 1 : -1;
    vY = (diffY > 0) ? 1 : -1;
  }
  
  while(vX != 0 || vY != 0){
    // Check if motors still need moving
    if ((posX >= nextX && vX > 0) || (posX <= nextX && vX < 0)) {
      vX = 0;
    }
    if ((posY >= nextY && vY > 0) || (posY <= nextY && vY < 0)) {
      vY = 0;
    }
    // Set new motor phases
    if(sX < 0) {
      sX += 8;
    }
    if(sX > 8){
      sX -= 8; 
    }
    if(sY < 0) {
      sY += 8;
    }
    if(sY > 8){
      sY -= 8; 
    }
    
    SetPhaseX((int)floor(sX));
    SetPhaseY((int)floor(sY));
    // Takeaway velocity (rather than add) due to motor direction.
    sX -= vX;
    sY -= vY;

    // Update distance travelled
    posX += (vX/500);
    posY += (vY/500);

    delay(1);
  }
  if (!isFinishing) {
    Serial.write(":)");
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
