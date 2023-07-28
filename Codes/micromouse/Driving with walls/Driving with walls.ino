// final code floodfill still under implementation

// #include <Pair.h>
#include <Queue.h>
#include <util/atomic.h>

int i;
int j;
int k;

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
int target1 = 0;
int target2 = 0;

//maze variables
#define mazeSize 16
struct node {
  int value;
  int walls[4];
};
struct point {
  int x;
  int y;
};
int c = mazeSize / 2;
char orientation = 'N';
char rat = 'N';
node maze[mazeSize][mazeSize] = {};
point start = point{ mazeSize - 1, 0 };
// point current;
// point target;
// point tmp;
point ftarget;
//int endPoints[4][2] = {{c - 1, c - 1}, {c - 1, c}, {c, c - 1}, {c, c}};

// Algo variables
bool first = true;
//int queue[][2] = {{}};
//int qIter = 0;
//int qSize = 0;
Queue<int, 20> queue;
Queue<int, 20> queue1;
Queue<int, 3> n;
Queue<int, 3> n1;
Queue<char, 3> path;
point current;
char p;
void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(EN1A, INPUT);
  pinMode(EN1B, INPUT);
  pinMode(EN2A, INPUT);
  pinMode(EN2B, INPUT);
  //initializeMaze();
  //delay(1000);
  attachInterrupt(digitalPinToInterrupt(EN1B), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2B), readEncoder2, RISING);
  //printMaze();
  Serial.println();
  queue.enqueue(start.x);
  queue1.enqueue(start.y);
  // current = start;
  Serial.begin(9600);
}

void loop() {
  //neighbours();
  // put your main code here, to run repeatedly:
  readIR();
  print1();
  // detectWalls();
  // neighbours();

  if (driving) {
    drive();
  } else {
    if (path.isEmpty()) {
      if (i % 3 == 0) {
        path.enqueue('F');
        path.enqueue('R');
        path.enqueue('R');
      } else if (i % 3 == 1) {
        path.enqueue('R');
      } else if (i % 3 == 2) {
        path.enqueue('L');
      }
      // i++;
    } else {
      p = path.front();
      path.dequeue();
      if (p == 'F') {
        target1 = 391;
        target2 = 391;
        Serial.print("Added Forward to Stack");
      } else if (p == 'R') {

        target1 = -136;
        target2 = 136;
        Serial.print("Added right to Stack");
      } else if (p == 'L') {

        target1 = 136;
        target2 = -136;
        Serial.print("Added Left to Stack");
      }
      driving = true;
    }
  }
  //  Serial.print(ftarget.x);
  //  Serial.print("  ");
  //  Serial.println(ftarget.y);
}

// Follows the target set while moving the mouse
void drive() {
  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
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
  // Serial.print(e1);
  // Serial.print("  ");
  // Serial.print(e2);
  // Serial.println("  ");
  if (fabs(e1) < 15 && fabs(e2) < 15) {
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
    pos1 = 0;
    pos2 = 0;
    driving = false;
    delay(200);
  }
  // derivative
  float dedt1 = (e1 - eprev1) / (deltaT);
  float dedt2 = (e2 - eprev2) / (deltaT);

  // integral
  eintegral1 = eintegral1 + e1 * deltaT;

  eintegral2 = eintegral2 + e2 * deltaT;
  if (p == 2) { kp = 1.7; }  ////////////////////////////////////////special condition oooorrr put a minimum value the motors can be at
  // control signal
  float u1 = kp * e1 + kd * dedt1 + ki * eintegral1;
  float u2 = kp * e2 + kd * dedt2 + ki * eintegral2;

  // motor power
  float pwr1 = fabs(u1);
  float pwr2 = fabs(u2);
  if (pwr1 > 255) {
    pwr1 = 255;
  }
  if (pwr2 > 255) {
    pwr2 = 255;
  }
  // pwr1 = max(pwr1, 50);
  // pwr2 = max(pwr2, 50);
  // motor direction
  int dir1 = 1;
  int dir2 = 1;
  if (u1 < 0) {
    dir1 = -1;
  }
  if (u2 < 0) {
    dir2 = -1;
  }

  if (p == 1) {
    if (i3 > 150 && i4 > 150) {
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
      pos1 = 0;
      pos2 = 0;
      driving = false;
      delay(1000);
    } else if (i1 > 100 && i2 > 100) {  //both walls
      int e = i2 - i1;
      // Serial.println(e);
      if (fabs(e) > 100) {
        pwr = fabs(e) * kir;
        if (e > 0) {
          analogWrite(M2A, 0);
          analogWrite(M2B, max(0, pwr1 - pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, min(255, pwr2 + pwr));
        } else {
          analogWrite(M2A, 0);
          analogWrite(M2B, min(255, pwr1 + pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, max(0, pwr2 - pwr));
        }
      } else {
        analogWrite(M2A, 0);
        analogWrite(M2B, pwr1);
        analogWrite(M1A, 0);
        analogWrite(M1B, pwr2);
      }
    } else if (i1 < 100 && i2 < 100) {
      //pass
      setMotor(dir2, pwr2, M1A, M1B);
      setMotor(dir1, pwr1, M2A, M2B);
    } else if (i1 < 100) {
      int e = i2 - 200;
      // Serial.println(e);
      if (fabs(e) > 100) {
        pwr = fabs(e) * kir;
        if (e > 0) {
          analogWrite(M2A, 0);
          analogWrite(M2B, max(0, pwr1 - pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, min(255, pwr2 + pwr));
        } else {
          analogWrite(M2A, 0);
          analogWrite(M2B, min(255, pwr1 + pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, max(0, pwr2 - pwr));
        }
      } else {
        analogWrite(M2A, 0);
        analogWrite(M2B, pwr1);
        analogWrite(M1A, 0);
        analogWrite(M1B, pwr2);
      }
    } else if (i2 < 100) {
      int e = i1 - 200;
      // Serial.println(e);
      if (fabs(e) > 100) {
        pwr = fabs(e) * kir;
        if (e < 0) {
          analogWrite(M2A, 0);
          analogWrite(M2B, max(0, pwr1 - pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, min(255, pwr2 + pwr));
        } else {
          analogWrite(M2A, 0);
          analogWrite(M2B, min(255, pwr1 + pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, max(0, pwr2 - pwr));
        }
      } else {
        analogWrite(M2A, 0);
        analogWrite(M2B, pwr1);
        analogWrite(M1A, 0);
        analogWrite(M1B, pwr2);
      }
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

  } else {
    setMotor(dir2, pwr2, M1A, M1B);
    setMotor(dir1, pwr1, M2A, M2B);
  }
  // Serial.print(target1);
  // Serial.print(" ");
  // Serial.print(target2);
  // Serial.print(" ");
  // Serial.print(pos1);
  // Serial.print(" ");
  // Serial.print(pos2);
  // Serial.println();
}

// Serial.print(pos1);
// Serial.print("  ");
// Serial.print(pos2);
// Serial.print("  ");
// Serial.print(target1);
// Serial.print("  ");
// Serial.print(target2);
// Serial.println("  ");
// }

// Print value of each node in the Serial Monitor
void printMaze() {
  for (int i = 0; i < mazeSize; i++) {
    for (int j = 0; j < mazeSize; j++) {
      Serial.print(maze[i][j].value);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Set the PWM pins for motors along with direction
void setMotor(int dir, int pwmVal, int in1, int in2) {
  //analogWrite(pwm,pwmVal);
  if (dir == 1) {
    analogWrite(in1, pwmVal);
    analogWrite(in2, 0);
  } else if (dir == -1) {
    analogWrite(in1, 0);
    analogWrite(in2, pwmVal);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

// Reading IR's
void readIR() {
  digitalWrite(13, HIGH);
  digitalWrite(8, LOW);
  delay(1);
  i1 = analogRead(A0);
  i2 = analogRead(A7);
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
  } else {
    pos1--;
  }
}

// ISR for Encoder 2
void readEncoder2() {
  int b = digitalRead(EN2A);
  if (b > 0) {
    pos2++;
  } else {
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
