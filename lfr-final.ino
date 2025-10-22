/*
Basic Code Template for the LF-2 robot using 16 channel Analog Sensor
*/

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
//------------------------------------------------------------


#define s0 A0  // A0 defined as digital pin 14
#define s1 A1  // A1 defined as digital pin 15
#define s2 A2  // A2 defined as digital pin 16
#define s3 A3  // A3 defined as digital pin 17


//--------Enter Line Details here---------
bool isBlackLine = 1;  //keep 1 in case of black line. In case of white line change this to 0
unsigned int numSensors = 16;
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 180;
int currentSpeed = 30;
int sensorWeight[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7 };  // The middle top and bottom sensors are ignored for normal line following

// int sensorWeight[16] = { 100, 100, 100, 6, 4, 3, 2, 0, 0, -2, -3, -4, -6, 100, 100, 100 };  // The middle top and bottom sensors are ignored for normal line following
int activeSensors;
float Kp = 0.13;
float Kd = 0.25;
float Ki = 0;
int onLine = 1;
int minValues[16], maxValues[16], threshold[16], sensorValue[16], sensorArray[16];

double startMillis, prevMillis, elapsedTime;

void setup() {

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP);  //Pushbutton
  pinMode(12, INPUT_PULLUP);  //Pushbutton
  pinMode(13, OUTPUT);        //LED

  pinMode(5, OUTPUT);     //standby for older carrier boards
  digitalWrite(5, HIGH);  //enables the motor driver
                          //in case of a newer board with motor enable jumper, you can delete these two lines which frees up digital pin 5

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1) {
    startMillis = millis();
    digitalWrite(s0, 0);
    digitalWrite(s1, 0);
    digitalWrite(s2, 0);
    digitalWrite(s3, 0);
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  //PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      if (error > 1000) {
        motor1run(-100);
        motor2run(255);
      } else if (error < -1000) {
        motor1run(255);
        motor2run(-100);
      }
    }
    elapsedTime = millis() - startMillis;

  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 16; i++) {
    if (sensorArray[i]) {
      //if(i==2||i==13) continue;
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    }
    activeSensors += sensorArray[i];
  }
  error = error / activeSensors;

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;


  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < -100) {
    lsp = -100;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -100) {
    rsp = -100;
  }
  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  for (int i = 0; i < 16; i++) {
    minValues[i] = sensorRead(i);
    maxValues[i] = sensorRead(i);
  }

  for (int i = 0; i < 3000; i++) {
    motor1run(120);
    motor2run(-120);

    for (int i = 0; i < 16; i++) {
      if (sensorRead(i) < minValues[i]) {
        minValues[i] = sensorRead(i);
      }
      if (sensorRead(i) > maxValues[i]) {
        maxValues[i] = sensorRead(i);
      }
    }
  }

  for (int i = 0; i < 16; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  for (int i = 0; i < 16; i++) {
    if (isBlackLine) {
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;

    if (sensorArray[i]) onLine = 1;
  }
  printAdjValues();
}

void printAdjValues() {
  for (int i = 0; i < 16; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(" | ");
    Serial.print(sensorValue[i]);
    Serial.print("  ");
  }
  Serial.println();
}
//--------Function to run Motor 1-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}

int sensorRead(int sensor) {
  // if(sensor == 2 || sensor == 13) continue;
  // digitalWrite(s0, sensor & 0x01);
  // digitalWrite(s1, sensor & 0x02);
  // digitalWrite(s2, sensor & 0x04);
  // digitalWrite(s3, sensor & 0x08);
  // return analogRead(4);
  if(sensor != 2 && sensor != 13){
  digitalWrite(s0, sensor & 0x01);
  digitalWrite(s1, sensor & 0x02);
  digitalWrite(s2, sensor & 0x04);
  digitalWrite(s3, sensor & 0x08);
  return analogRead(4);
  }
}
