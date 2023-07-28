//orientation
// code with workning left and right turns and algo intgrated. 

#include <Vector.h>
#include <Queue.h>
#include <util/atomic.h>

//for select neighbours
int neighx[4];
int neighy[4];
Vector<int> neighbourx(neighx);
Vector<int> neighboury(neighy);

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
#define elementCount1 50
// motor control
// float kp = 1.2;
// float kd = 0.0;
// float ki = 0.0;
// float kir = 1.01;
float kp1 = 4.0;
float kd1 = 4.5;
float ki1 = 0.000;
float kp2 = 3.9;
float kd2 = 4.5;
float ki2 = 0.000;
float kir = 0.41;
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
int orientation = 1;
char p;
struct point {
  int x;
  int y;
};
point current{ newMazeSize - 1, 0 };
// Queue<char, 100> path;
int strArray[elementCount];
Vector<int> path(strArray);
int elementxq[elementCount1];
int elementyq[elementCount1];
Vector<int> queuex(elementxq);
Vector<int> queuey(elementyq);
int vectorFront = 0;

//for floodfill
#define qsize 50
int xq1[qsize];
Vector<int> xq(xq1);
int yq1[qsize];
Vector<int> yq(yq1);

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
    //    updateVal(
    if (n == 0) {
      detectWalls();
      updateVal();
      selectNext();
      // if (i % 3 == 0) {
      //   path.push_back(1);
      //   path.push_back(2);
      //   path.push_back(1);
      //   path.push_back(2);
      //   path.push_back(1);
      //   path.push_back(3);
      //   path.push_back(3);
      // } else if (i % 3 == 1) {
      //   path.push_back(3);
      // } else if (i % 3 == 2) {
      //   path.push_back(2);
      // }
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
        driving = true;
        // Serial.print("Added Forward to Stack");
      } else if (p == 2) {  ///righttt

        target1 = -138;
        target2 = 138;
        // orientation = (orientation + 1) % 4;
        driving = true;
        // Serial.print("Added right to Stack");
      } else if (p == 3) {

        target1 = 136;
        target2 = -136;

        // orientation = (orientation - 1) % 4;
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
    }
  }
  //  Serial.print(ftarget.x);
  //  Serial.print("  ");
  //  Serial.println(ftarget.y);
}

// checks if the coordinates are valid
bool isValid(int x, int y) {
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

//marks the current walls and neighbour walls in the newMaze
void detectWalls() {
  int x2 = current.x;
  int y2 = current.y;
  if (orientation == 1) {
    if (i1 > 100) {
      setEastWall(x2, y2);
      if (isValid(x2, y2 + 1))
        setWestWall(x2, y2 + 1);
    }
    if (i2 > 100) {
      setWestWall(x2, y2);
      if (isValid(x2, y2 - 1))
        setEastWall(x2, y2 - 1);
    }
    if (i3 > 100 && i4 > 100) {
      setNorthWall(x2, y2);
      if (isValid(x2 - 1, y2))
        setSouthWall(x2 - 1, y2);
    }
  }
  if (orientation == 2) {
    if (i1 > 100) {
      setSouthWall(x2, y2);
      if (isValid(x2 + 1, y2))
        setNorthWall(x2 + 1, y2);
    }
    if (i2 > 100) {
      setNorthWall(x2, y2);
      if (isValid(x2 - 1, y2))
        setSouthWall(x2 - 1, y2);
    }
    if (i3 > 100 && i4 > 100) {
      setEastWall(x2, y2);
      if (isValid(x2, y2 + 1))
        setWestWall(x2, y2 + 1);
    }
  }
  if (orientation == 3) {
    if (i1 > 100) {
      setWestWall(x2, y2);
      if (isValid(x2, y2 - 1))
        setEastWall(x2, y2 - 1);
    }
    if (i2 > 100) {
      setEastWall(x2, y2);
      if (isValid(x2, y2 + 1))
        setWestWall(x2, y2 + 1);
    }
    if (i3 > 100 && i4 > 100) {
      setSouthWall(x2, y2);
      if (isValid(x2 + 1, y2))
        setNorthWall(x2 + 1, y2);
    }
  }
  if (orientation == 4) {
    if (i1 > 100) {
      setNorthWall(x2, y2);
      if (isValid(x2 - 1, y2))
        setSouthWall(x2 - 1, y2);
    }
    if (i2 > 100) {
      setSouthWall(x2, y2);
      if (isValid(x2 + 1, y2))
        setNorthWall(x2 + 1, y2);
    }
    if (i3 > 100 && i4 > 100) {
      setWestWall(x2, y2);
      if (isValid(x2, y2 - 1))
        setEastWall(x2, y2 - 1);
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

// select the neighbours for updating value (in updateVal and floodfill function)
void selecNeighbours(int x, int y) {
  int i10 = newMaze[x][y].walls;
  int N = i10 & 0b1000;
  int e = i10 & 0b0100;
  int s = i10 & 0b0010;
  int w = i10 & 0b0001;
  if (N == 0b0000)  //north is present
  {
    neighbourx.push_back(-1);
    neighboury.push_back(0);
  }
  if (e == 0b0000)  //east is present
  {
    neighbourx.push_back(0);
    neighboury.push_back(1);
  }
  if (s == 0b0000)  // south is present
  {
    neighbourx.push_back(1);
    neighboury.push_back(0);
  }
  if (w == 0b0000)  ///west is present
  {
    neighbourx.push_back(0);
    neighboury.push_back(-1);
  }
}

//solves the dead end till a node with more than one opening is reached(path vector and orientation are also being updated in here, current.x and current.y
//are being updated by the final x and y after the cell with more than one opening uis reached)
void solveDead() {
  int x2 = current.x;
  int y2 = current.y;
  int orientation1;
  while (newMaze[x2][y2].walls == 0b0111 || newMaze[x2][y2].walls == 0b1110 || newMaze[x2][y2].walls == 0b1011 || newMaze[x2][y2].walls == 0b1101) {
    if (newMaze[x2][y2].walls == 0b0111) {
      orientation1 = 1;
      if (orientation != orientation1) {
        if (orientation == 2) {
          path.push_back(3);
        } else if (orientation == 3) {
          path.push_back(2);
          path.push_back(2);
        } else if (orientation == 4) {
          path.push_back(2);
        }
      }
      orientation = orientation1;
      newMaze[x2][y2].walls = newMaze[x2][y2].walls | 0b1111;
      if (isValid(x2 - 1, y2)) {
        newMaze[x2 - 1][y2].walls = newMaze[x2 - 1][y2].walls | 0b0010;
        path.push_back(1);
        x2 = x2 - 1;
        y2 = y2;
      }
    } else if (newMaze[x2][y2].walls == 0b1110) {
      orientation1 = 4;
      if (orientation != orientation1) {
        if (orientation == 1) {
          path.push_back(3);
        } else if (orientation == 2) {
          path.push_back(2);
          path.push_back(2);
        } else if (orientation == 3) {
          path.push_back(2);
        }
      }
      orientation = orientation1;
      newMaze[x2][y2].walls = newMaze[x2][y2].walls | 0b0001;
      if (isValid(x2, y2 - 1)) {
        newMaze[x2][y2 - 1].walls = newMaze[x2][y2 - 1].walls | 0b0100;
        path.push_back(1);
        x2 = x2;
        y2 = y2 - 1;
      }
    } else if (newMaze[x2][y2].walls == 0b1011) {
      orientation1 = 2;
      if (orientation != orientation1) {
        if (orientation == 1) {
          path.push_back(2);
        } else if (orientation == 3) {
          path.push_back(3);
        } else if (orientation == 4) {
          path.push_back(2);
          path.push_back(2);
        }
      }
      orientation = orientation1;
      newMaze[x2][y2].walls = newMaze[x2][y2].walls | 0b1111;
      if (isValid(x2, y2 + 1)) {
        newMaze[x2][y2 + 1].walls = newMaze[x2][y2 + 1].walls | 0b0001;
        path.push_back(1);
        x2 = x2;
        y2 = y2 + 1;
      }
    } else if (newMaze[x2][y2].walls == 0b1101) {
      orientation1 = 3;
      if (orientation != orientation1) {
        if (orientation == 1) {
          path.push_back(2);
          path.push_back(2);
        } else if (orientation == 2) {
          path.push_back(2);
        } else if (orientation == 4) {
          path.push_back(3);
        }
      }
      orientation = orientation1;
      newMaze[x2][y2].walls = newMaze[x2][y2].walls | 0b1111;
      if (isValid(x2 + 1, y2)) {
        newMaze[x2 + 1][y2].walls = newMaze[x2 + 1][y2].walls | 0b1000;
        path.push_back(1);
        x2 = x2 + 1;
        y2 = y2;
      }
    }
  }
  current.x = x2;
  current.y = y2;
}

//if value of all the neighours of a cell are greater than the value of cell than flood fill is being called,
//and the value are pushed till we get the cell having a neighbour with smaller value than the value of the current cell
void floodfill() {
  while (!xq.empty()) {
    int x = xq[0];
    int y = yq[0];
    xq.remove(0);
    yq.remove(0);
    bool keep = true;
    void selecNeighbours();
    for (int i = 0; i < neighbourx.size(); i++) {
      if (isValid(x + neighbourx[i], y + neighboury[i])) {
        if (newMaze[x + neighbourx[i]][y + neighboury[i]].value < newMaze[x][y].value) {
          keep = false;
        }
      }
    }
    if (keep == true) {
      int val = 10000;
      for (int i = 0; i < neighbourx.size(); i++) {
        if (isValid(x + neighbourx[i], y + neighboury[i])) {
          val = min(val, newMaze[x + neighbourx[i]][y + neighboury[i]].value);
          xq.push_back(x + neighbourx[i]);
          yq.push_back(y + neighboury[i]);
        }
      }
      newMaze[x][y].value = val + 1;
    }
    neighbourx.clear();
    neighboury.clear();
  }
}

//the value at the current cell is being updated(if the changes are needed to be done they are being pushed in the queue(vector queue) and floodfill is called)
void updateVal() {
  int x2 = current.x;
  int y2 = current.y;
  if ((newMaze[x2][y2].walls == 0b0111 || newMaze[x2][y2].walls == 0b1110 || newMaze[x2][y2].walls == 0b1011 || newMaze[x2][y2].walls == 0b1101) && (x2 != 0 || y2 != 0)) {
    solveDead();
  }
  x2 = current.x;
  y2 = current.y;
  bool keep = true;
  selecNeighbours(x2, y2);
  for (int i = 0; i < neighbourx.size(); i++) {
    if (isValid(x2 + neighbourx[i], y2 + neighboury[i])) {
      if (newMaze[x2 + neighbourx[i]][y2 + neighboury[i]].value < newMaze[x2][y2].value) {
        keep = false;
      }
    }
  }
  neighbourx.clear();
  neighboury.clear();
  if (keep == true) {
    xq.push_back(x2);
    yq.push_back(y2);
    floodfill();
  }
}

//neighbours are being selected for the next move(the cells opposite to the present orientation are being
//ignored and the cells are pushed in the sequence in accordance with the orientation)
void selecNeighbours1(int x, int y) {
  int i10 = newMaze[x][y].walls;
  int n = i10 & 0b1000;
  int e = i10 & 0b0100;
  int s = i10 & 0b0010;
  int w = i10 & 0b0001;
  if (orientation == 1) {
    if (n == 0b0000)  //north is present
    {
      // if(!(orientation == ))
      neighbourx.push_back(-1);
      neighboury.push_back(0);
    }
    if (e == 0b0000)  //east is present
    {
      neighbourx.push_back(0);
      neighboury.push_back(1);
    }
    if (w == 0b0000)  ///west is present
    {
      neighbourx.push_back(0);
      neighboury.push_back(-1);
    }
  }
  if (orientation == 2) {
    if (e == 0b0000)  //east is present
    {
      neighbourx.push_back(0);
      neighboury.push_back(1);
    }
    if (s == 0b0000)  // south is present
    {
      neighbourx.push_back(1);
      neighboury.push_back(0);
    }
    if (n == 0b0000)  //north is present
    {
      // if(!(orientation == ))
      neighbourx.push_back(-1);
      neighboury.push_back(0);
    }
  }
  if (orientation == 3) {
    if (s == 0b0000)  // south is present
    {
      neighbourx.push_back(1);
      neighboury.push_back(0);
    }
    if (e == 0b0000)  //east is present
    {
      neighbourx.push_back(0);
      neighboury.push_back(1);
    }
    if (w == 0b0000)  ///west is present
    {
      neighbourx.push_back(0);
      neighboury.push_back(-1);
    }
  }
  if (orientation == 4) {
    if (w == 0b0000)  ///west is present
    {
      neighbourx.push_back(0);
      neighboury.push_back(-1);
    }
    if (s == 0b0000)  // south is present
    {
      neighbourx.push_back(1);
      neighboury.push_back(0);
    }
    if (n == 0b0000)  //north is present
    {
      // if(!(orientation == ))
      neighbourx.push_back(-1);
      neighboury.push_back(0);
    }
  }
  //    if (n == 0b0000 && !(orientation == 3)) //north is present
  //    {
  //        // if(!(orientation == ))
  //        neighbourx.push_back(-1);
  //        neighboury.push_back(0);
  //    }
  //    if (e == 0b0000 && !(orientation == 4)) //east is present
  //    {
  //        neighbourx.push_back(0);
  //        neighboury.push_back(1);
  //    }
  //    if (s == 0b0000  && !(orientation == 1)) // south is present
  //    {
  //        neighbourx.push_back(1);
  //        neighboury.push_back(0);
  //    }
  //    if (w == 0b0000  && !(orientation == 2)) ///west is present
  //    {
  //        neighbourx.push_back(0);
  //        neighboury.push_back(-1);
  //    }
}

//the orientation, current struct and the path vector is being updated according to the present orientation and the target cell)(changeX and changeY are
//the additons which are to be done to current.x and current.y to update the current cell)
void setVar(int changeX, int changeY, int x2, int y2) {
  if (changeX == -1 && changeY == 0) {
    if (orientation == 1) {
      path.push_back(1);
    } else if (orientation == 2) {
      path.push_back(3);
      path.push_back(1);
    } else if (orientation == 4) {
      path.push_back(2);
      path.push_back(1);
    }
    orientation = 1;
  }
  if (changeX == 1 && changeY == 0) {
    if (orientation == 3) {
      path.push_back(1);
    } else if (orientation == 4) {
      path.push_back(3);
      path.push_back(1);
    } else if (orientation == 2) {
      path.push_back(2);
      path.push_back(1);
    }
    orientation = 3;
  }
  if (changeX == 0 && changeY == -1) {
    if (orientation == 4) {
      path.push_back(1);
    } else if (orientation == 1) {
      path.push_back(3);
      path.push_back(1);
    } else if (orientation == 3) {
      path.push_back(2);
      path.push_back(1);
    }
    orientation = 4;
  }
  if (changeX == 0 && changeY == 1) {
    if (orientation == 2) {
      path.push_back(1);
    } else if (orientation == 3) {
      path.push_back(3);
      path.push_back(1);
    } else if (orientation == 1) {
      path.push_back(2);
      path.push_back(1);
    }
    orientation = 2;
  }
  x2 = x2 + changeX;
  y2 = y2 + changeY;
  current.x = x2;
  current.y = y2;
}

//function to select the next cell on which the bot has to move, after the detection of walls, treatment of dead end and updation of values,
//the cell is decided after selecting the min of all the neighbour cells, possX[0] and possY[0] contains the value which is to be added in current.x and current.y for the updation of cells
void selectNext() {
  int x2 = current.x;
  int y2 = current.y;
  selecNeighbours1(x2, y2);
  int val = 1000;
  for (int i = 0; i < neighbourx.size(); i++) {
    if (isValid(x2 + neighbourx[i], y2 + neighboury[i])) {
      val = min(val, newMaze[x2 + neighbourx[i]][y2 + neighboury[i]].value);
    }
  }
#define possSize 5
  int possarrx[possSize];
  int possarry[possSize];
  Vector<int> possX(possarrx);
  Vector<int> possY(possarry);
  for (int i = 0; i < neighbourx.size(); i++) {
    if (isValid(x2 + neighbourx[i], y2 + neighboury[i]) && val == newMaze[x2 + neighbourx[i]][y2 + neighboury[i]].value) {
      possX.push_back(neighbourx[i]);
      possY.push_back(neighboury[i]);
    }
  }
  neighbourx.clear();
  neighboury.clear();
  setVar(possX[0], possY[0], x2, y2);
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
    delay(200);
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
        noInterrupts();
        analogWrite(M1A, 0);
        analogWrite(M1B, 0);
        analogWrite(M2A, 0);
        analogWrite(M2B, 0);
        pos1 = 0;
        pos2 = 0;
        driving = false;
        delay(200);
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
            pwr1 += fabs(i1 - 400) * 0.003;
          }
        }
        setMotor(dir2, pwr2, M1A, M1B);
        setMotor(dir1, pwr1, M2A, M2B);
      } else if (i1 < 100) {
        // Serial.println("right walls prsent");
        if (fabs(i2 - 400) > 150) {
          if (i2 < 400) {
            pwr2 += fabs(i2 - 400) * 0.003;
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