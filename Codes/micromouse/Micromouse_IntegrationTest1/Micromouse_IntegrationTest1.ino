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
float kp = 1.2;
float kd = 0.0;
float ki = 0.0;
float kir = 1.01;
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

#define newMazeSize 16
#define c newMazeSize / 2
struct node {
  int value;
  byte walls;
};
node newMaze[newMazeSize][newMazeSize] = {};
int target1 = 0;
int target2 = 0;
int orientation = 0;
char p;
struct point {
  int x;
  int y;
};
point current{ newMazeSize - 1, 0 };
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
  Serial.begin(9600);
  // for (int i=0;i<100;i++){
  initializenewMaze();
  // delay(1000);
  //     v1.push_back(0);
  // }
}

void loop() {
  // put your main code here, to run repeatedly:
  // printnewMaze();
  //neighbours();
  readIR();
  // printnewMaze();
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
    detectWalls();
    if (n == 0) {
      if (i % 3 == 0) {
        path.push_back(1);
        path.push_back(2);
        path.push_back(1);
        path.push_back(2);
        path.push_back(1);
        path.push_back(3);
        path.push_back(3);
      } else if (i % 3 == 1) {
        path.push_back(3);
      } else if (i % 3 == 2) {
        path.push_back(2);
      }
      // Serial.print("hi");
      // i++;
      vectorFront = 0;


    } else {

      p = path[0];
      path.remove(0);
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
        // Serial.print("Added Forward to Stack");
      } else if (p == 2) {  ///righttt

        target1 = -132;
        target2 = 132;
        orientation = (orientation + 1) % 4;
        // Serial.print("Added right to Stack");
      } else if (p == 3) {

        target1 = 132;
        target2 = -132;

        orientation = (orientation - 1) % 4;
        // Serial.print("Added Left to Stack");
      }
      // path.reverse();
      driving = true;
    }
  }
  //  Serial.print(ftarget.x);
  //  Serial.print("  ");
  //  Serial.println(ftarget.y);
}

bool isValid(int x, int y)
{
    if (x >= 0 && y >= 0 && x < 16 && y < 16)
        return true;

    return false;
}

void updatePos() {
  if (orientation == 1) {
    current.x -= 1;
  } else if (orientation == 2) {
    current.y += 1;
  } else if (orientation == 3) {
    current.y -= 1;
  } else if (orientation == 4) {
    current.x += 1;
  }
}
//mark them in the newMaze
void detectWalls() {
  int x2 = current.x;
  int y2 = current.y;
  if (orientation == 1) {
    if (i1 > 100) {
      setEastWall(x2, y2);
      if(isValid(x2, y2+1))
      setWestWall(x2, y2+1);
    }
    if (i2 > 100) {
      setWestWall(x2, y2);
      if(isValid(x2, y2-1))
      setEastWall(x2, y2-1);
    }
    if (i3 > 100 && i4 > 100) {
      setNorthWall(x2, y2);
      if(isValid(x2-1, y2))
      setSouthWall(x2-1, y2);
    }
  }
  if (orientation == 2) {
    if (i1 > 100) {
      setSouthWall(x2, y2);
      if(isValid(x2+1, y2))
      setNorthWall(x2+1, y2);
    }
    if (i2 > 100) {
      setNorthWall(x2, y2);
      if(isValid(x2-1, y2))
      setSouthWall(x2-1, y2);
    }
    if (i3 > 100 && i4 > 100) {
      setEastWall(x2, y2);
      if(isValid(x2, y2+1))
      setWestWall(x2, y2+1);
    }
  }
  if (orientation == 3) {
    if (i1 > 100) {
      setWestWall(x2, y2);
      if(isValid(x2, y2-1))
      setEastWall(x2, y2-1);
    }
    if (i2 > 100) {
      setEastWall(x2, y2);
      if(isValid(x2, y2+1))
      setWestWall(x2, y2+1);
    }
    if (i3 > 100 && i4 > 100) {
      setSouthWall(x2, y2);
      if(isValid(x2+1, y2))
      setNorthWall(x2+1, y2);
    }
  }
  if (orientation == 4) {
    if (i1 > 100) {
      setNorthWall(x2, y2);
      if(isValid(x2-1, y2))
      setSouthWall(x2-1, y2);
    }
    if (i2 > 100) {
      setSouthWall(x2, y2);
      if(isValid(x2+1, y2))
      setNorthWall(x2+1, y2);
    }
    if (i3 > 100 && i4 > 100) {
      setWestWall(x2, y2);
      if(isValid(x2, y2-1))
      setEastWall(x2, y2-1);
    }
  }

  Serial.write(orientation);
  Serial.write("   ");
  Serial.write(current.x);
  Serial.write("   ");
  Serial.write(current.y);
  Serial.write("   ");
  Serial.write(newMaze[current.x][current.y].walls);
}


void initializenewMaze() {

  for (int i = 0; i < newMazeSize; i++) {
    for (int j = 0; j < newMazeSize; j++) {
      newMaze[i][j] = node{ 0, 0b00000000 };
    }
  }

  for (int i = 0; i < newMazeSize; i++) {
    setNorthWall(0, i);
  }
  for (int i = 0; i < newMazeSize; i++) {
    setWestWall(i, 0);
  }
  for (int i = 0; i < newMazeSize; i++) {
    setSouthWall(newMazeSize - 1, i);
  }
  for (int i = 0; i < newMazeSize; i++) {
    setEastWall(i, newMazeSize - 1);
  }
  //  for (int i = 0; i < 4; i++) {
  //    newMaze[endPoints[i][0]][endPoints[i][1]].value = 1;
  //  }
  int v = 0;
  for (int m = 0; m < c; m++) {
    int x = v;
    for (int i = m; i < c; i++) {
      newMaze[m + c][i + c].value = x;
      x++;
    }
    x = v;
    for (int i = m; i < c; i++) {
      newMaze[i + c][m + c].value = x;
      x++;
    }
    v += 2;
  }
  v = 0;
  for (int m = 0; m < c; m++) {
    int x = v;
    for (int i = m; i <= c; i++) {
      newMaze[c - m - 1][i + c].value = x;
      x++;
    }
    x = v;
    for (int i = c - m - 1; i >= 0; i--) {
      newMaze[i][m + c].value = x;
      x++;
    }
    v += 2;
  }
  v = 0;
  for (int m = 0; m < c; m++) {
    int x = v;
    for (int i = c - m - 1; i >= 0; i--) {
      newMaze[c - m - 1][i].value = x;
      x++;
    }
    x = v;
    for (int i = c - m - 1; i >= 0; i--) {
      newMaze[i][c - m - 1].value = x;
      x++;
    }
    v += 2;
  }
  v = 0;
  for (int m = 0; m < c; m++) {
    int x = v;
    for (int i = m; i < c; i++) {
      newMaze[c + m][c - 1 - i].value = x;
      x++;
    }
    x = v;
    for (int i = m; i < c; i++) {
      newMaze[c + i][c - 1 - m].value = x;
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
void setNorthWall(int i, int j) {
  newMaze[i][j].walls = newMaze[i][j].walls | 0b1000;
}

// Setting East wall of the newMaze cell
void setEastWall(int i, int j) {
  newMaze[i][j].walls = newMaze[i][j].walls | 0b0100;
}

// Setting North wall of the newMaze cell
void setSouthWall(int i, int j) {
  newMaze[i][j].walls = newMaze[i][j].walls | 0b0010;
}

// Setting North wall of the newMaze cell
void setWestWall(int i, int j) {
  newMaze[i][j].walls = newMaze[i][j].walls | 0b0001;
}

// Print value of each node in the Serial Monitor
void printnewMaze() {
  for (int i = 0; i < newMazeSize; i++) {
    for (int j = 0; j < newMazeSize; j++) {
      Serial.print(newMaze[i][j].value);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void printWalls() {
  for (int i = 0; i < newMazeSize; i++) {
    for (int j = 0; j < newMazeSize; j++) {
      Serial.print(newMaze[i][j].walls, BIN);
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
