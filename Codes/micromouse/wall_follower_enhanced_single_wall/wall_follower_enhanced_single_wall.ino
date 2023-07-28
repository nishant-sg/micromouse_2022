// date 19-11-22
// Aim - Follow wall without using both walls, if one wall is absent it should maintain appropriate distance from the other and stay in centre
// result - shaky but achievedqr

#include <util/atomic.h>

int i;

//IR inputs
int i1 = 0;
int i2 = 0;
int i3 = 0;
int i4 = 0;

//PID Controls
int prevT = 0;
int currT = 0;
int ei = 0;
float kpi = 0.4;
float kdi = 0.00;
float kii = 0.0;
float eprev1 = 0;
float eprev2 = 0;
float eintegral1 = 0;
float eintegral2 = 0;
float deti = 0;
float eintegrali = 0;
float u = 0;
float pwr = 0;

//Motors
#define M1A 5
#define M1B 6
#define M2A 9
#define M2B 10

// Encoders
#define EN1A 4
#define EN1B 3
#define EN2A 7
#define EN2B 2

// motor control
volatile long pos1 = 0;
volatile long pos2 = 0;
volatile bool driving = true;
int e1 = 0;
int e2 = 0;
int target1 = 391 * 5;
int target2 = 391 * 5;
bool first = true;

// the setup function runs once when you press reset or power the board
void setup() {
  // Switiching on IR pairs
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(EN1A, INPUT);
  pinMode(EN1B, INPUT);
  pinMode(EN2A, INPUT);
  pinMode(EN2B, INPUT);

  // Adding Interrupts
  attachInterrupt(digitalPinToInterrupt(EN1B), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2B), readEncoder2, RISING);

  // Serial Monitor
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  delay(10);
  readIR();
  //print1();
  int posA = 0;
  int posB = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posA = pos1;
    posB = pos2;
  }
  int avg = (posA + posB) / 2;
  //Serial.println(avg);

  if (i1 < 100) { // if right wall is absent
    if (avg > 390*5) {  // stay on till entire block not moved
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
    } else {              // follow right wall
      int e = i2 - 400;
      if (fabs(e) > 25) {
        pwr = fabs(e) * 1.2;
        if (e < 0) {
          analogWrite(M2A, 0);
          analogWrite(M2B, max(0, 150 + pwr) );
          analogWrite(M1A, 0);
          analogWrite(M1B, min(255, 150 - pwr));
        } else {
          analogWrite(M2A, 0);
          analogWrite(M2B, min(255, 150 - pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, max(0, 150 + pwr));
        }
      }else{
      analogWrite(M1A, 0);
      analogWrite(M1B, 150);
      analogWrite(M2A, 0);
      analogWrite(M2B, 150);}
    }
  } else if (i2 < 100) { // if left wall is absent
    if (avg > 390*5) {      // stay on till entire block not moved
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
      // pos1=0;pos2=0;
    } else {              // follow right wall
      int e = i1 - 400;
      if (fabs(e) > 25) {
        pwr = fabs(e) * 1.2;
        if (e < 0) {
          analogWrite(M2A, 0);
          analogWrite(M2B, max(0, 150 - pwr) );
          analogWrite(M1A, 0);
          analogWrite(M1B, min(255, 150 + pwr));
        } else {
          analogWrite(M2A, 0);
          analogWrite(M2B, min(255, 150 + pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, max(0, 150 - pwr));
        }
      }else{
      analogWrite(M1A, 0);
      analogWrite(M1B, 150);
      analogWrite(M2A, 0);
      analogWrite(M2B, 150);}
    }
  }
  else if (i3 > 400 && i4 > 400) { // forward wall is present
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);

  } else if (abs(i1 - i2) > 100 ) { // wall follower stay in between
    int currT = micros();
    float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
    int prevT = currT;
    ei = abs(i1 - i2);
    deti = ei / (deltaT);
    eintegrali = eintegrali + ei * deltaT;
    u = kpi * ei + kdi * deti + kii * eintegrali;
    pwr = fabs(u);
    if (i1 < i2) {
      analogWrite(M2A, 0);
      analogWrite(M2B, max(0, 150 - pwr) );
      analogWrite(M1A, 0);
      analogWrite(M1B, min(255, 150 + pwr));
    } else {
      analogWrite(M2A, 0);
      analogWrite(M2B, min(255, 150 + pwr));
      analogWrite(M1A, 0);
      analogWrite(M1B, max(0, 150 - pwr));
    }
  } else { // keep going straight
    analogWrite(M1A, 0);
    analogWrite(M1B, 150);
    analogWrite(M2A, 0);
    analogWrite(M2B, 150);
    if (avg > 391) {
      pos1 = 0;
      pos2 = 0;
    }
  }
}

void drive() {
  // PID constants
  float kp = 0.9;
  float kd = 0.0;
  float ki = 0.0;
  float kIR = 0.9;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;
  int posA = 0;
  int posB = 0;


  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posA = pos1;
    posB = pos2;

  }

  e1 = posA - target1;
  e2 = posB - target2;

  if ((e1 < 15 && e1 > -15) || (e2 < 15 && e2 > -15)) {
    driving = false;

    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
    delay(2000);
  }

  // derivative
  float dedt1 = (e1 - eprev1) / (deltaT);
  float dedt2 = (e2 - eprev2) / (deltaT);

  // integral
  eintegral1 = eintegral1 + e1 * deltaT;

  eintegral2 = eintegral2 + e2 * deltaT;

  // control signal
  float u1 = kp * e1 + kd * dedt1 + ki * eintegral1;
  float u2 = kp * e2 + kd * dedt2 + ki * eintegral2;

  // motor power
  float pwr1 = fabs(u1);
  float pwr2 = fabs(u2);


  if ( pwr1 > 200 ) {
    pwr1 = 200;
  }
  if ( pwr2 > 200 ) {
    pwr2 = 200;
  }

  //     if (abs(i1 - i2) > 100 ) {
  //    int errorIR = abs(i1 - i2)*0.1 ;
  //    if (i1 < i2) {
  //      pwr1 += errorIR* kIR;
  //      pwr2 -= errorIR* kIR;
  //    } else {
  //      pwr2 += errorIR* kIR;
  //      pwr1 -= errorIR* kIR;
  //    }
  //
  //  }

  // motor direction
  int dir1 = 1;
  int dir2 = 1;
  if (u1 < 0) {
    dir1 = -1;
  }
  if (u2 < 0) {
    dir2 = -1;
  }
  //pwr1 = pwr2;
  if (abs(i1 - i2) > 100 ) {

    int errorIR = min(50, abs(i1 - i2) * 10.1) ;
    if (i1 > i2) {
      pwr1 += errorIR;
      pwr2 -= errorIR;
    } else {
      pwr2 += errorIR;
      pwr1 -= errorIR;
    }

  }

  Serial.print(pwr1);
  Serial.print(" ");
  Serial.println(pwr2);
  setMotor(dir2, pwr2, M1A, M1B);
  setMotor(dir1, pwr1, M2A, M2B);
  //  Serial.print(target1);
  //  Serial.print(" ");
  //  Serial.print(target2);
  //  Serial.print(" ");
  //  Serial.print(pos1);
  //  Serial.print(" ");
  //  Serial.print(pos2);
  //  Serial.println();
  //noInterrupts();

}

// Writing values to motors
void setMotor(int dir, int pwmVal, int in1, int in2) {
  if (dir == 1) {
    analogWrite(in1, pwmVal);
    analogWrite(in2, 0);
  }
  else if (dir == -1) {
    analogWrite(in1, 0);
    analogWrite(in2, pwmVal);
  }
  else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}


// Reading IR's
void readIR() {
  digitalWrite(13, HIGH);
  digitalWrite(8, LOW);
  delay(1);
  i1 = analogRead(A0) ;
  i2 = analogRead(A7) ;
  digitalWrite(13, LOW);
  digitalWrite(8, HIGH);
  i3 = analogRead(A1);
  i4 = analogRead(A6);
  delay(1);
}

// ISR for Encoder 1
void readEncoder1() {
  int b = digitalRead(EN1A);
  if (b > 0) {
    pos1++;
  }
  else {
    pos1--;
  }
}

// ISR for Encoder 2
void readEncoder2() {
  int b = digitalRead(EN2A);
  if (b > 0) {
    pos2++;
  }
  else {
    pos2--;
  }
}

// Function for printing IR Values
void print1() {
  Serial.print(i1);
  Serial.print(" ");
  Serial.print(i3);
  Serial.print(" ");
  Serial.print(i4);
  Serial.print(" ");
  Serial.print(i2);
  Serial.println(" ");
}
