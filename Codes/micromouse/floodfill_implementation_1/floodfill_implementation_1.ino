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
int iw,jw;
//maze variables
#define mazeSize 16
struct node {
  int value;
  byte walls;
};
struct point {
  int x;
  int y;
};
node maze[mazeSize][mazeSize] = {};
int c = mazeSize / 2;
char orientation = 'N';
char rat = 'N';
point start = point{ mazeSize - 1, 0 };
// point current;
// point target;
// point tmp;
  point target;
  point tmp;
point ftarget;
//int endPoints[4][2] = {{c - 1, c - 1}, {c - 1, c}, {c, c - 1}, {c, c}};
int iloop;
int jloop;
int kloop;
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
  initializeMaze();
  //delay(1000);
  attachInterrupt(digitalPinToInterrupt(EN1B), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(EN2B), readEncoder2, RISING);

  // Serial.println();
  queue.enqueue(start.x);
  queue1.enqueue(start.y);
  // delay(10000);
  // current = start;
  Serial.begin(9600);
}

void loop() {
  //neighbours();
  // put your main code here, to run repeatedly:
  readIR();
  // printMaze();
  // printWalls();
  // delay(10000);
  // print1();
 // detectWalls();
  //neighbours();

  if (driving) {
    drive();
  } else {
    if (path.isEmpty()) {
      if (i % 3 == 0) {
        path.enqueue('F');
      } else if (i % 3 == 1) {
        path.enqueue('F');
      } else if (i % 3 == 2) {
        path.enqueue('F');
      }
      i++;
    } else {
      p = path.front();
      path.dequeue();
      if (p == 'F') {
        target1 = 391;
        target2 = 391;
        // Serial.print("Added Forward to Stack");
      } else if (p == 'R96') {

        target1 = -136;
        target2 = 136;
        // Serial.print("Added right to Stack");
      } else if (p == 'L') {

        target1 = 136;
        target2 = -136;
        // Serial.print("Added Left to Stack");
      }
      driving = true;
    }
  }
  //  Serial.print(ftarget.x);
  //  Serial.print("  ");
  //  Serial.println(ftarget.y);
}

// Calculate the series of steps to take to reach the next cell
void directions() {
  if (ftarget.x < current.x) {
    if (orientation == 'N') {
      path.enqueue('F');
    } else if (orientation == 'E') {
      path.enqueue('L');
      path.enqueue('F');
    } else if (orientation == 'S') {
      path.enqueue('R');
      path.enqueue('R');
      path.enqueue('F');
    } else if (orientation == 'W') {
      path.enqueue('R');
      path.enqueue('F');
    }
  } else if (ftarget.x > current.x) {
    if (orientation == 'N') {
      path.enqueue('R');
      path.enqueue('R');
      path.enqueue('F');
    } else if (orientation == 'E') {
      path.enqueue('R');
      path.enqueue('F');
    } else if (orientation == 'S') {
      path.enqueue('F');
    } else if (orientation == 'W') {
      path.enqueue('L');
      path.enqueue('F');
    }
  } else if (ftarget.y < current.y) {
    if (orientation == 'N') {
      path.enqueue('L');
      path.enqueue('F');
    } else if (orientation == 'E') {
      path.enqueue('R');
      path.enqueue('R');
      path.enqueue('F');
    } else if (orientation == 'S') {
      path.enqueue('R');
      path.enqueue('F');
    } else if (orientation == 'W') {
      path.enqueue('F');
    }
  } else if (ftarget.y > current.y) {
    if (orientation == 'N') {
      path.enqueue('R');
      path.enqueue('F');
    } else if (orientation == 'E') {
      path.enqueue('F');
    } else if (orientation == 'S') {
      path.enqueue('L');
      path.enqueue('F');
    } else if (orientation == 'W') {
      path.enqueue('R');
      path.enqueue('R');
      path.enqueue('F');
    }
  }
}

Detect the walls and mark them in the maze
void detectWalls() {
  if (orientation == 'N') {
    if (i1 > 100) {
      maze[current.x][current.y].walls[1] = 1;
    }
    if (i2 > 100) {
      maze[current.x][current.y].walls[3] = 1;
    }
    if (i3 > 600 && i4 > 600) {
      maze[current.x][current.y].walls[0] = 1;
    }
  }
  if (orientation == 'E') {
    if (i1 > 100) {
      maze[current.x][current.y].walls[2] = 1;
    }
    if (i2 > 100) {
      maze[current.x][current.y].walls[0] = 1;
    }
    if (i3 > 600 && i4 > 600) {
      maze[current.x][current.y].walls[1] = 1;
    }
  }
  if (orientation == 'S') {
    if (i1 > 100) {
      maze[current.x][current.y].walls[3] = 1;
    }
    if (i2 > 100) {
      maze[current.x][current.y].walls[1] = 1;
    }
    if (i3 > 600 && i4 > 600) {
      maze[current.x][current.y].walls[2] = 1;
    }
  }
  if (orientation == 'W') {
    if (i1 > 100) {
      maze[current.x][current.y].walls[0] = 1;
    }
    if (i2 > 100) {
      maze[current.x][current.y].walls[2] = 1;
    }
    if (i3 > 600 && i4 > 600) {
      maze[current.x][current.y].walls[3] = 1;
    }
  }
}

// Find the valid neighbour
// void neighbours() {
//   i = current.x;
//   j = current.y;
//   //  point n[]; .walls= 0b00001110 && 0b00001000 ==
//   if (i - 1 >= 0 && maze[i][j].walls == 0) {
//     n.enqueue(i - 1);
//     n1.enqueue(j);
//   }
//   if (i + 1 < mazeSize && maze[i][j].walls == 0) {
//     n.enqueue(i + 1);
//     n1.enqueue(j);
//   }
//   if (j - 1 >= 0 && maze[i][j].walls == 0) {
//     n.enqueue(i);
//     n1.enqueue(j - 1);
//   }
//   if (j + 1 < mazeSize && maze[i][j].walls == 0) {
//     n.enqueue(i);
//     n1.enqueue(j + 1);
//   }

//   target.x = n.front();
//   target.y = n1.front();
//   n.dequeue();
//   n1.dequeue();
//   for (k = 0; k < n.size(); k++) {
//     tmp.x = n.front();
//     tmp.y = n1.front();
//     if (!n.isEmpty())
//       n.dequeue();
//     if (!n1.isEmpty())
//       n1.dequeue();
//     if (maze[target.x][target.y].value > maze[tmp.x][tmp.y].value) {
//       target = tmp;
//     }
//   }
//   ftarget = target;
// }

// to calculate the floodfill values
void reEvaluateDistance() {
}

// Declare the initial value of maze with value at centre being lowest and corners being highest
void initializeMaze() {
  
  for (iloop = 0; iloop < mazeSize; iloop++) {
    for (jloop = 0; jloop < mazeSize; jloop++) {
      maze[iloop][jloop] = node{ 0, 0b00000000 };
    }
  }

  for (iloop = 0; iloop < mazeSize; iloop++) {
    iw = 0;
    jw = iloop;
    setNorthWall();
  }
  for (iloop = 0; iloop < mazeSize; iloop++) {
    iw = iloop;
    jw = 0;
    setWestWall();
  }
  for (iloop = 0; iloop < mazeSize; iloop++) {
    iw = mazeSize -1;
    jw = iloop;
    setSouthWall();
  }
  for ( iloop = 0; iloop < mazeSize; iloop++) {
    iw = iloop;
    jw = mazeSize - 1;
    setEastWall();
  }
  //  for (int i = 0; i < 4; i++) {
  //    maze[endPoints[i][0]][endPoints[i][1]].value = 1;
  //  }
  // int v = 0;
  // for (int m = 0; m < c; m++) {
  //   int x = v;
  //   for (int i = m; i < c; i++) {
  //     maze[m + c][i + c].value = x;
  //     x++;
  //   }
  //   x = v;
  //   for (int i = m; i < c; i++) {
  //     maze[i + c][m + c].value = x;
  //     x++;
  //   }
  //   v += 2;
  // }
  // v = 0;
  // for (int m = 0; m < c; m++) {
  //   int x = v;
  //   for (int i = m; i <= c; i++) {
  //     maze[c - m - 1][i + c].value = x;
  //     x++;
  //   }
  //   x = v;
  //   for (int i = c - m - 1; i >= 0; i--) {
  //     maze[i][m + c].value = x;
  //     x++;
  //   }
  //   v += 2;
  // }
  // v = 0;
  // for (int m = 0; m < c; m++) {
  //   int x = v;
  //   for (int i = c - m - 1; i >= 0; i--) {
  //     maze[c - m - 1][i].value = x;
  //     x++;
  //   }
  //   x = v;
  //   for (int i = c - m - 1; i >= 0; i--) {
  //     maze[i][c - m - 1].value = x;
  //     x++;
  //   }
  //   v += 2;
  // }
  // v = 0;
  // for (int m = 0; m < c; m++) {
  //   int x = v;
  //   for (int i = m; i < c; i++) {
  //     maze[c + m][c - 1 - i].value = x;
  //     x++;
  //   }
  //   x = v;
  //   for (int i = m; i < c; i++) {
  //     maze[c + i][c - 1 - m].value = x;
  //     x++;
  //   }
  //   v += 2;
  // }
}

// pta nhi kya krta hai
// void append(int a[]) {
//   Serial.print(a[0]);
//   Serial.print(" ");
//   Serial.print(a[1]);
//   queue[qIter] = a;
//   qIter++;
// }

// pta nhi kya krta hai
// void printQ() {
//   for (int i = 0; i < qIter; i++) {
//     Serial.print(queue[i][0]);
//     Serial.print(" ");
//     Serial.println(queue[i][1]);
//   }
// }

// Follows the target set while moving the mouse
void drive() {
  // PID constants
  float kp = 0.9;
  float kd = 0.0;
  float ki = 0.0;
  float kIR = 0.05;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;
  int posA = 0;
  int posB = 0;


  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posA = pos1;
    posB = pos2;
  }
  int avg = (posA + posB) / 2;
  //Serial.println(avg);

  if (i1 < 100) {                                                                  // if right wall is absent
    if (fabs(posA) > fabs(target1 * 0.95) || fabs(posB) > fabs(target2 * 0.95)) {  // stay on till entire block not moved
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
      pos1 = 0;
      pos2 = 0;
      driving = false;
      // Serial.println("Stoppped after completing block right wall absent");
      delay(1000);
    } else {  // follow right wall
      int e = i2 - 400;
      if (fabs(e) > 25) {
        pwr = fabs(e) * 1.2;
        if (e < 0) {
          analogWrite(M2A, 0);
          analogWrite(M2B, max(0, 150 + pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, min(255, 150 - pwr));
        } else {
          analogWrite(M2A, 0);
          analogWrite(M2B, min(255, 150 - pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, max(0, 150 + pwr));
        }
      } else {
        analogWrite(M1A, 0);
        analogWrite(M1B, 150);
        analogWrite(M2A, 0);
        analogWrite(M2B, 150);
      }
    }
  } else if (i2 < 100) {  // if left wall is absent
    if (avg > 390) {      // stay on till entire block not moved
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
      // Serial.println("Stoppped after completing block left wall absent");
      pos1 = 0;
      pos2 = 0;
      driving = false;
      delay(1000);
    } else {  // follow right wall
      int e = i1 - 400;
      if (fabs(e) > 25) {
        pwr = fabs(e) * 1.2;
        if (e < 0) {
          analogWrite(M2A, 0);
          analogWrite(M2B, max(0, 150 - pwr));
          analogWrite(M1A, 0);
          analogWrite(M1B, min(255, 150 + pwr));
        } else {
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
      }
    }
  } else if (i3 > 400 && i4 > 400) {  // forward wall is present
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
    pos1 = 0;
    pos2 = 0;
    delay(1000);
    driving = false;
    // Serial.println("Stoppped after completing block Forward wall present");

  } else if (abs(i1 - i2) > 100) {  // wall follower stay in between
    int currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    int prevT = currT;
    ei = abs(i1 - i2);
    deti = ei / (deltaT);
    eintegrali = eintegrali + ei * deltaT;
    u = kpi * ei + kdi * deti + kii * eintegrali;
    pwr = fabs(u);
    if (i1 < i2) {
      analogWrite(M2A, 0);
      analogWrite(M2B, max(0, 150 - pwr));
      analogWrite(M1A, 0);
      analogWrite(M1B, min(255, 150 + pwr));
    } else {
      analogWrite(M2A, 0);
      analogWrite(M2B, min(255, 150 + pwr));
      analogWrite(M1A, 0);
      analogWrite(M1B, max(0, 150 - pwr));
    }
  } else {  // keep going straight
    analogWrite(M1A, 0);
    analogWrite(M1B, 150);
    analogWrite(M2A, 0);
    analogWrite(M2B, 150);
    if (avg > 391) {
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
      pos1 = 0;
      pos2 = 0;
      delay(1000);
      driving = false;

      // Serial.println("Stoppped after completing block both wall present");
    }
  }


  // Serial.print(pos1);
  // Serial.print("  ");
  // Serial.print(pos2);
  // Serial.print("  ");
  // Serial.print(target1);
  // Serial.print("  ");
  // Serial.print(target2);
  // Serial.println("  ");
}

// Setting North wall of the maze cell
void setNorthWall() {
  maze[iw][jw].walls = maze[iw][jw].walls | 0b00001000;
}

// Setting East wall of the maze cell
void setEastWall() {
  maze[iw][jw].walls = maze[iw][jw].walls | 0b00000100;
}

// Setting North wall of the maze cell
void setSouthWall() {
  maze[iw][jw].walls = maze[iw][jw].walls | 0b00000010;
}

// Setting North wall of the maze cell
void setWestWall() {
  maze[iw][jw].walls = maze[iw][jw].walls | 0b00000001;
}


// Print value of each node in the Serial Monitor
void printMaze() {
  for ( iloop = 0; iloop < mazeSize; iloop++) {
    for (iloop = 0;jloop < mazeSize; jloop++) {
      // Serial.print(maze[iloop][jloop].value);
      // Serial.print(" ");
    }
    // Serial.println();
  }
}

void printWalls() {
  for (iloop = 0; iloop < mazeSize; iloop++) {
    for (jloop = 0; jloop < mazeSize; jloop++) {
      // Serial.print(maze[iloop][jloop].walls, BIN);
      // Serial.print(" ");
    }
    // Serial.println();
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
  // Serial.print(i1);
  // Serial.print(" ");
  // Serial.print(i3);
  // Serial.print(" ");
  // Serial.print(i4);
  // Serial.print(" ");
  // Serial.print(i2);
  // Serial.println(" ");
}
