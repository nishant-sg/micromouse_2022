#include <LiquidCrystal.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#include <Wire.h>

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

#define EN1A 4
#define EN1B 3
#define EN2A 7
#define EN2B 2

#define M1A 5
#define M1B 6
#define M2A 9
#define M2B 10
long pos1 = 0;
long pos2 = 0;
volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev1 = 0;
float eprev2 = 0;
float eintegral1 = 0;
float eintegral2 = 0;
bool travelling = false;
char arr[50]={'F','F','F'};
int i = 0;
int i1 = 0;
int i2 = 0;
int i3 = 0;
int i4 = 0;
void setup() {
  lcd.init(); // initialize the lcd
  lcd.backlight();
  Serial.begin(9600);
  pinMode(EN1A, INPUT);
  pinMode(EN1B, INPUT);
  pinMode(EN2A, INPUT);
  pinMode(EN2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(EN1B), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2B), readEncoder2, RISING);
}

void loop() {
  forward(391,391);
}

int forward(int t1,int t2){
//int target = 450*sin(prevT/1e6);
  int target1 = t1;//136 is 90 degree
  int target2 = t2;
  // PID constants
  float kp = 0.4;
  float kd = 0.009;
  float ki = 0.001;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  //  int pos1 = 0;
    int posA = 0;
    int posB = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      posA = pos1;
      posB = pos2;
    }

  int e1 = posA - target1;
  int e2 = posB - target2;
  if (e1==0 && e2==0){
    travelling= false;
    i+=1;
  }
  // derivative
  float dedt1 = (e1 - eprev1) / (deltaT);
  float dedt2 = (e2 - eprev2) / (deltaT);

  // integral
  eintegral1 = eintegral1 + e1 * deltaT;
  
  eintegral2 = eintegral2 + e2 * deltaT;

  // control signal
  float u1 = kp * e1 + kd * dedt1 + ki * eintegral1 ;
  float u2 = kp * e2 + kd * dedt2 + ki * eintegral2;

  // motor power
  float pwr1 = fabs(u1);
  float pwr2 = fabs(u2);

  readIR();
  pwr1 = 
  
  if ( pwr1 > 255 ) {
    pwr1 = 255;
  }
  if ( pwr2 > 255 ) {
    pwr2 = 255;
  }

  // motor direction
  int dir1 = 1;
  int dir2 = 1;
  if (u1 < 0) {
    dir1 = -1;
  }
  if (u2 < 0) {
    dir2 = -1;
  }

  
  //  int a = digitalRead(EN1A);
  //  int b = digitalRead(EN1B);
  //  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  //  lcd.print(a*5);        // print message at (0, 0)
  //  lcd.setCursor(0, 1);         // move cursor to   (0, 0)
  //  lcd.print(b*5);
  //  Serial.print(a*5);
  //  Serial.print(" ");
  //  Serial.print(b*5);
  //  Serial.println();
  
  setMotor(dir1, pwr2, M1A, M1B);
  setMotor(dir1, pwr2, M2A, M2B);

  Serial.print(target1);
  Serial.print(" ");
  Serial.print(target2);
  Serial.print(" ");
  Serial.print(pos1);
  Serial.print(" ");
  Serial.print(pos2);
  Serial.println();

}


void setMotor(int dir, int pwmVal, int in1, int in2) {
  //analogWrite(pwm,pwmVal);
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


void readEncoder1() {
  int b = digitalRead(EN1A);
  if (b > 0) {
    pos1++;
  }
  else {
    pos1--;
  }
}

void readEncoder2() {
  int b = digitalRead(EN2A);
  if (b > 0) {
    pos2++;
  }
  else {
    pos2--;
  }
}

void readIR() {
  digitalWrite(13, HIGH);
  digitalWrite(8, LOW);
  i1 = analogRead(A0);
  i2 = analogRead(A7);
  digitalWrite(13, LOW);
  digitalWrite(8, HIGH);
  i3 = analogRead(A1);
  i4 = analogRead(A6);
  i2 = map(i2, 150, 1020, 0, 500);
  i1 = map(i1, 0, 400, 0, 500);
  i4 = map(i4, 150, 1020, 0, 500);
  i3 = map(i3, 50, 1020, 0, 500);
}

void print() {
  Serial.print(i1);
  Serial.print(" ");
  Serial.print(i3);
  Serial.print(" ");
  Serial.print(i4);
  Serial.print(" ");
  Serial.print(i2);
  Serial.println(" ");
}
