#include <Vector.h>

#define elementCount 50

int strArray[elementCount];
Vector<int> path(strArray);
int vectorFront = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(path.size());
  path.push_back(1);
  path.push_back(2);
  path.push_back(3);
  path.push_back(4);
  path.push_back(5);
  Serial.println(path.size());
  Serial.println(path[vectorFront]);
  path.remove(vectorFront);
  Serial.println(path[vectorFront]);
  path.remove(vectorFront);
  Serial.println(path[vectorFront]);
  path.remove(vectorFront);
  
  Serial.println(path.size());
    path.remove(vectorFront);
      path.remove(vectorFront);
  Serial.println(path.empty());
}

void loop() {
  // put your main code here, to run repeatedly:
}
