#include <util/atomic.h> // For the ATOMIC_BLOCK macro

int i;
int prevT = 0;
int currT = 0;
int ei = 0;
float deti = 0;
float eintegrali = 0;
float kpi = 0.4;
float kdi = 0.00;
float kii = 0.0;
float u = 0;
float pwr = 0;
#define M1A 5
#define M1B 6
#define M2A 9
#define M2B 10
#define EN1A 4
#define EN1B 3
#define EN2A 7
#define EN2B 2
int i1 = 0;
int i2 = 0;
int i3 = 0;
int i4 = 0;

volatile long pos1 = 0;
volatile long pos2 = 0;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(EN1A, INPUT);
  pinMode(EN1B, INPUT);
  pinMode(EN2A, INPUT);
  pinMode(EN2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(EN1B), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2B), readEncoder2, RISING);
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
  Serial.println(avg);
  if (i1 < 100) {
    if (avg>390){    
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
     // pos1=0;pos2=0;
    }else{
      analogWrite(M1A, 0);
      analogWrite(M1B, 150);
      analogWrite(M2A, 0);
      analogWrite(M2B, 150);
      
      
    }
  }else if (i2 < 100) {
    if (avg>390){    
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
     // pos1=0;pos2=0;
    }else{
      analogWrite(M1A, 0);
      analogWrite(M1B, 150);
      analogWrite(M2A, 0);
      analogWrite(M2B, 150);
      
      
    }
  }
  else if (i3 > 400 && i4 > 400) {
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);

  } else if (abs(i1 - i2) > 100 ) {
    int currT = micros();
    float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
    int prevT = currT;
    ei = abs(i1 - i2);
    //      Serial.println(ei);
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
      //        ei = abs(i1 - i2);
      //        //      Serial.println(ei);
      //        deti = ei / (deltaT);
      //        eintegrali = eintegrali + ei * deltaT;
      //        u = kpi * ei + kdi * deti + kii * eintegrali;
      //        pwr = fabs(u);
      //      Serial.print("right");
      analogWrite(M2A, 0);
      analogWrite(M2B, min(255, 150 + pwr));
      analogWrite(M1A, 0);
      analogWrite(M1B, max(0, 150 - pwr));

    }
  } else {
    analogWrite(M1A, 0);
    analogWrite(M1B, 150);
    analogWrite(M2A, 0);
    analogWrite(M2B, 150);
    if (avg>391){
      pos1 = 0;
      pos2 = 0;
    }
    }

  }

  //
  //  if ( pwr > 255 ) {
  //    pwr = 255;
  //  }
  //  int dir = 1;
  //  if (u < 0) {
  //    int dir = -1;
  //  }
  //  else {
  //    if (i1 < i2) {
  //      pwr = -pwr;
  //      if (dir == -1) {
  //        analogWrite(M1A, pwr);
  //        analogWrite(M1B, 0);
  //        analogWrite(M2A, pwr);
  //        analogWrite(M2B, 0);
  //      }
  //      else if (dir == 1) {
  //        analogWrite(M2A, 0);
  //        analogWrite(M2B, 150 + pwr);
  //        analogWrite(M1A, 0);
  //        analogWrite(M1B, 150 - pwr);
  //      }
  //      else {
  //        analogWrite(M1A, 0);
  //        analogWrite(M1B, 0);
  //        analogWrite(M2A, 0);
  //        analogWrite(M2B, 0);
  //      }
  //    }
  //  }
//}

void readIR() {
  digitalWrite(13, HIGH);
  digitalWrite(8, LOW);
  delay(1);
  i1 = analogRead(A0) ;//- 110; // best - 110
  i2 = analogRead(A7) ;//- 1010; // best - 1010
  digitalWrite(13, LOW);
  digitalWrite(8, HIGH);
  i3 = analogRead(A1);
  i4 = analogRead(A6);
  delay(1);
  //    i1 = map(i1,-250,-550, 0, 1000);
  //    i2 = map(i2, -50, 200, 0, 1000);
  //  i4 = map(i4, 150, 1020, 0, 1000);
  //  i3 = map(i3, 50, 1020, 0, 1000);
}

void readIR1() {
  digitalWrite(13, HIGH);
  i1 = analogRead(A0); // best - 110
  i2 = analogRead(A7); // best - 1010
  //  i1 += analogRead(A0);
  //  i1 += analogRead(A0);
  //  i1 += analogRead(A0);
  //  i1 += analogRead(A0);
  //  i1 = i1/5;
  //  Serial.print(i1);
  //  Serial.print(" ");
  //  Serial.print(i2);
  //  Serial.println(" ");
  //  digitalWrite(13, LOW);
  //int  i1b = analogRead(A0); // best - 110
  //  int i2b = analogRead(A7);
  //    Serial.print(i1);
  //  Serial.print(" ");
  //  Serial.print(i2);
  //  Serial.println(" ");
  digitalWrite(8, HIGH);
  i3 = analogRead(A1);
  i4 = analogRead(A6);
  //    i1 = map(i1,-550,-200, 0, 1000);
  //    i2 = map(i2, -50, 200, 0, 1000);
  //  i4 = map(i4, 150, 1020, 0, 1000);
  //  i3 = map(i3, 50, 1020, 0, 1000);
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
