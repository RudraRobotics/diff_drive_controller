#include <Encoder.h>

#define enB 5
#define in3 4
#define in4 3

#define enA 45
#define in1 46
#define in2 44

void setup()
{
  // initialize serial:
  Serial.begin(57600);

  // For motor driver
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


void setMotorVel(String inputString) {

  String left_motor_cmd = "";
  String right_motor_cmd = "";
  bool seperator = false;
  for(int i=0;i<inputString.length();i++) {
    char data = inputString[i];
    if(data==',') {
      seperator = true;
    }
    else if(!seperator && data!=",") { 
      left_motor_cmd+=data;    
    }
    else if(seperator && data!=",") {
      right_motor_cmd+=data;
    }
  }

  int left_motor_vel = left_motor_cmd.toInt();
  int right_motor_vel = right_motor_cmd.toInt();
  if (left_motor_vel < 0) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    left_motor_vel = -left_motor_vel;
  }
  else if (left_motor_vel > 0) { // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    left_motor_vel = left_motor_vel;
  }

  if (right_motor_vel < 0) {
    // Set Motor A backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    right_motor_vel = -right_motor_vel;
  }
  else if (right_motor_vel > 0) {
    // Set Motor A forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    right_motor_vel = right_motor_vel;
  }

  analogWrite(enA, left_motor_vel); // Send PWM signal to motor A
  analogWrite(enB, right_motor_vel); // Send PWM signal to motor B
}

Encoder rightEnc(18, 19);
Encoder leftEnc(2, 11);

long oldPositionL  = 0;
long oldPositionR  = 0;
String motor_vel = "";
int count = 0;
void loop()
{
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();
    motor_vel += incomingByte;

    count++;
    if (count % 16 == 0) {
      // Set motor speed
      setMotorVel(motor_vel);
      String ang_vel = "";
      float now = millis();
      
      long newPositionL = leftEnc.read();
      if(newPositionL != oldPositionL) {
        oldPositionL = newPositionL;
      }
      ang_vel.concat(oldPositionL);
      
      ang_vel.concat(",");
     
      long newPositionR = rightEnc.read();
      if(newPositionR != oldPositionR) {
        oldPositionR = newPositionR;
      }
      ang_vel.concat(oldPositionR);
      
      for (int i = 0; i < ang_vel.length(); i++) {
        Serial.write(ang_vel[i]);
      }
      motor_vel = "";
    }
  }
}
