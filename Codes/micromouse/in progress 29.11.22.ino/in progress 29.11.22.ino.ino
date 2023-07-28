#include <Vector.h>
#include <Queue.h>
#include <util/atomic.h>


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

#define elementCount 50
// motor control
float kp1 = 1.5;
float kd1 = 1.56;
float ki1 = 0.0;
float kp2 = 1.2;
float kd2 = 1.56;
float ki2 = 0.0;
float kir = 0.41;
// float kp1 = 4.0;
// float kd1 = 4.5;
// float ki1 = 0.000;
// float kp2 = 3.9;
// float kd2 = 4.5;
// float ki2 = 0.000;
// float kir = 0.41;
volatile long pos1 = 0;
volatile long pos2 = 0;
int prevT = 0;
float eprev1 = 0;
float eprev2 = 0;
float eintegral1 = 0;
float eintegral2 = 0;
float deti = 0;
float eintegrali = 0;
float u = 0;
float pwr = 0;
//IR inputs
int i1 = 0;
int i2 = 0;
int i3 = 0;
int i4 = 0;

bool driving = false;
int i;

#define mazeSize 16
#define c mazeSize / 2
struct node {
  int value;
  byte walls;
};
node maze[mazeSize][mazeSize] = {};
int target1 = 0;
int target2 = 0;
int orientation = 0;
char p;
struct point {
  int x;
  int y;
};
point current{ mazeSize - 1, 0 };
// Queue<char, 100> path;
int strArray[elementCount];
Vector<int> path(strArray);
int vectorFront = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(EN1A, INPUT);
  pinMode(EN1B, INPUT);
  pinMode(EN2A, INPUT);
  pinMode(EN2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(EN1B), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2B), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(11), autoCallibrate, RISING);
  Serial.begin(9600);
  // for (int i=0;i<100;i++){
  initializeMaze();
  // delay(1000);
  //     v1.push_back(0);
  // }
  Serial.write("started");
}

void loop() {
  // put your main code here, to run repeatedly:
  // printMaze();
  //neighbours();
  readIR();
  // printMaze();
  // printWalls();
  //delay(10000);
  // print1();
  // printWalls();
  //neighbours();

  if (driving) {
    drive();
  } else {
    int n = path.size();
    // Serial.println(n);
    if (n == 0) {
      // Serial.println("path empty");
      // Serial.println(orientation);
      detectWalls();
      if (i % 3 == 0) {
        path.push_back(1);
        path.push_back(3);
        path.push_back(1);
        path.push_back(3);
        path.push_back(1);
        path.push_back(4);
      } else if (i % 3 == 1) {

        path.push_back(2);
        path.push_back(1);
      } else if (i % 3 == 2) {

        path.push_back(2);
        path.push_back(1);
      }
      // Serial.print("hi");
      // i++;
      vectorFront = 0;


    } else {

      p = path[0];
      path.remove(0);
      // path.cleaer();
      // Serial.println("Printing p :");
      // Serial.println(p);
      // Serial.println("Printing Size :");
      // Serial.println(path.size());
      // reverse(path.begin(),path.end());
      //path.pop_back();
      if (p == 1) {
        target1 = 410;
        target2 = 410;
        updatePos();
        driving = true;
        // Serial.print("Added Forward to Stack");
      } else if (p == 2) {  ///righttt

        target1 = -138;
        target2 = 138;
        orientation = (orientation + 1) % 4;
        driving = true;
        // Serial.print("Added right to Stack");
      } else if (p == 3) {

        target1 = 136;
        target2 = -136;

        orientation = (orientation - 1) % 4;
        driving = true;
        // Serial.print("Added Left to Stack");
      } else if (p == 4) {
        if (i1 > i2) {
          path.push_back(3);
          path.push_back(3);
          //left
        } else {
          path.push_back(2);
          path.push_back(2);
        }
        for (int i = 0;i<path.size()-2;i++){
          int d = path[0];
          path.remove(0);
          path.push_back(d);
        }
        for (int i = 0;i<path.size();i++){
          Serial.print(path[i]);
        }
      }
      // path.reverse();
      
    }
  }
  //  Serial.print(ftarget.x);
  //  Serial.print("  ");
  //  Serial.println(ftarget.y);
}

void updatePos() {
  if (orientation == 0) {
    current.x -= 1;
  } else if (orientation == 1) {
    current.y += 1;
  } else if (orientation == 2) {
    current.x += 1;
  } else if (orientation == 3) {
    current.y -= 1;
  }
}
//mark them in the maze
void detectWalls() {
  if (orientation == 0) {
    if (i1 > 100) {
      setEastWall(current.x, current.y);
      setWestWall(current.x, current.y + 1);
    }
    if (i2 > 100) {
      setWestWall(current.x, current.y);
      setEastWall(current.x, current.y - 1);
    }
    if (i3 > 100 && i4 > 100) {
      setNorthWall(current.x, current.y);
      setSouthWall(current.x - 1, current.y);
    }
  }
  if (orientation == 1) {
    if (i1 > 100) {
      setSouthWall(current.x, current.y);
      setNorthWall(current.x + 1, current.y);
    }
    if (i2 > 100) {
      setNorthWall(current.x, current.y);
      setSouthWall(current.x - 1, current.y);
    }
    if (i3 > 100 && i4 > 100) {
      setEastWall(current.x, current.y);
      setWestWall(current.x, current.y + 1);
    }
  }
  if (orientation == 2) {
    if (i1 > 100) {
      setWestWall(current.x, current.y);
      setEastWall(current.x, current.y - 1);
    }
    if (i2 > 100) {
      setEastWall(current.x, current.y);
      setWestWall(current.x, current.y + 1);
    }
    if (i3 > 100 && i4 > 100) {
      setSouthWall(current.x, current.y);
      setNorthWall(current.x + 1, current.y);
    }
  }
  if (orientation == 3) {
    if (i1 > 100) {
      setNorthWall(current.x, current.y);
      setSouthWall(current.x - 1, current.y);
    }
    if (i2 > 100) {
      setSouthWall(current.x, current.y);
      setNorthWall(current.x + 1, current.y);
    }
    if (i3 > 100 && i4 > 100) {
      setWestWall(current.x, current.y);
      setEastWall(current.x, current.y - 1);
    }
  }

  // Serial.print(orientation);
  // Serial.print("   ");
  // Serial.print(current.x);
  // Serial.print("   ");
  // Serial.print(current.y);
  // Serial.print("   ");
  // Serial.println(maze[current.x][current.y].walls);
}


void initializeMaze() {

  for (int i = 0; i < mazeSize; i++) {
    for (int j = 0; j < mazeSize; j++) {
      maze[i][j] = node{ 0, 0b00000000 };
    }
  }

  for (int i = 0; i < mazeSize; i++) {
    setNorthWall(0, i);
  }
  for (int i = 0; i < mazeSize; i++) {
    setWestWall(i, 0);
  }
  for (int i = 0; i < mazeSize; i++) {
    setSouthWall(mazeSize - 1, i);
  }
  for (int i = 0; i < mazeSize; i++) {
    setEastWall(i, mazeSize - 1);
  }
  //  for (int i = 0; i < 4; i++) {
  //    maze[endPoints[i][0]][endPoints[i][1]].value = 1;
  //  }
  int v = 0;
  for (int m = 0; m < c; m++) {
    int x = v;
    for (int i = m; i < c; i++) {
      maze[m + c][i + c].value = x;
      x++;
    }
    x = v;
    for (int i = m; i < c; i++) {
      maze[i + c][m + c].value = x;
      x++;
    }
    v += 2;
  }
  v = 0;
  for (int m = 0; m < c; m++) {
    int x = v;
    for (int i = m; i <= c; i++) {
      maze[c - m - 1][i + c].value = x;
      x++;
    }
    x = v;
    for (int i = c - m - 1; i >= 0; i--) {
      maze[i][m + c].value = x;
      x++;
    }
    v += 2;
  }
  v = 0;
  for (int m = 0; m < c; m++) {
    int x = v;
    for (int i = c - m - 1; i >= 0; i--) {
      maze[c - m - 1][i].value = x;
      x++;
    }
    x = v;
    for (int i = c - m - 1; i >= 0; i--) {
      maze[i][c - m - 1].value = x;
      x++;
    }
    v += 2;
  }
  v = 0;
  for (int m = 0; m < c; m++) {
    int x = v;
    for (int i = m; i < c; i++) {
      maze[c + m][c - 1 - i].value = x;
      x++;
    }
    x = v;
    for (int i = m; i < c; i++) {
      maze[c + i][c - 1 - m].value = x;
      x++;
    }
    v += 2;
  }
}





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
  if (fabs(e1) == 0 && fabs(e2) == 0) {
    noInterrupts();
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
    pos1 = 0;
    pos2 = 0;
    driving = false;
    interrupts();
    delay(2000);
  } else {
    // Serial.println("else conditoin");
    // derivative
    float dedt1 = (e1 - eprev1) / (deltaT);
    float dedt2 = (e2 - eprev2) / (deltaT);
    eprev1 = e1;
    eprev2 = e2;
    // integral
    eintegral1 = eintegral1 + e1 * deltaT;

    eintegral2 = eintegral2 + e2 * deltaT;
    // if (p == 2) { kp1 = 1.7; }  ////////////////////////////////////////special condition oooorrr put a minimum value the motors can be at
    // control signal
    float u1 = kp1 * e1 + kd1 * dedt1 + ki1 * eintegral1;
    float u2 = kp2 * e2 + kd2 * dedt2 + ki2 * eintegral2;

    // motor power
    float pwr1 = fabs(u1);
    float pwr2 = fabs(u2);
    if (pwr1 > 255) {
      pwr1 = 255;
    }
    if (pwr2 > 255) {
      pwr2 = 255;
    }
    pwr1 = max(pwr1, 50);
    pwr2 = max(pwr2, 50);
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
      // Serial.println("p == 1");
      if (i3 > 150 && i4 > 150) {
        analogWrite(M1A, 0);
        analogWrite(M1B, 0);
        analogWrite(M2A, 0);
        analogWrite(M2B, 0);
        noInterrupts();
        pos1 = 0;
        pos2 = 0;
        driving = false;
        delay(2000);
        interrupts();
        delay(1000);
      } else if (i1 > 100 && i2 > 100) {  //both walls present
                                          // Serial.println("both walls prsent");
        if (fabs(i1 - i2) > 200) {
          if (i1 > i2) {
            pwr2 += fabs(i1 - i2) * kir * 0.004;
          } else {
            pwr1 += fabs(i1 - i2) * kir * 0.004;
          }
        }
        setMotor(dir2, pwr2, M1A, M1B);
        setMotor(dir1, pwr1, M2A, M2B);
      } else if (i1 < 100 && i2 < 100) {  // both walls absent
        //pass
        // Serial.println("both walls absent");
        setMotor(dir2, pwr2, M1A, M1B);
        setMotor(dir1, pwr1, M2A, M2B);
      } else if (i2 < 100) {
        // Serial.println("left wall prsent");
        if (fabs(i1 - 400) > 100) {
          if (i1 > 400) {
            pwr2 += fabs(i1 - 400) * 0.05;
          } else {
            pwr1 += fabs(i1 - 400) * 0.005;
          }
        }
        setMotor(dir2, pwr2, M1A, M1B);
        setMotor(dir1, pwr1, M2A, M2B);
      } else if (i1 < 100) {
        // Serial.println("right walls prsent");
        if (fabs(i2 - 400) > 150) {
          if (i2 < 400) {
            pwr2 += fabs(i2 - 400) * 0.005;
          } else {
            pwr1 += fabs(i2 - 400) * 0.05;
          }
        }
        setMotor(dir2, pwr2, M1A, M1B);
        setMotor(dir1, pwr1, M2A, M2B);
      }

    } else {
      // pwr2 = max(170,pwr2);
      // pwr1 = max(170,pwr1);
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

void autoCallibrate

void setNorthWall(int i, int j) {
  maze[i][j].walls = maze[i][j].walls | 0b00001000;
}

// Setting East wall of the maze cell
void setEastWall(int i, int j) {
  maze[i][j].walls = maze[i][j].walls | 0b00000100;
}

// Setting North wall of the maze cell
void setSouthWall(int i, int j) {
  maze[i][j].walls = maze[i][j].walls | 0b00000010;
}

// Setting North wall of the maze cell
void setWestWall(int i, int j) {
  maze[i][j].walls = maze[i][j].walls | 0b00000001;
}

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

void printWalls() {
  for (int i = 0; i < mazeSize; i++) {
    for (int j = 0; j < mazeSize; j++) {
      Serial.print(maze[i][j].walls, BIN);
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
