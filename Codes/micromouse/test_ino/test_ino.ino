#define ENCA 2
#define ENCB 3
int o1 = 9;
int o2 = 10;
int pos = 0;

void setup(){
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(13,INPUT);
  digitalWrite(13,LOW);
  //attachInterrupt(0, updateEncoder, RISING); 
  //attachInterrupt(1, updateEncoder2, RISING);
  

}

void loop(){
 analogWrite(o1,200);
 analogWrite(o2,0);
 delay(1000);
 analogWrite(o1,0);
 analogWrite(o2,0);
 delay(1000);
 analogWrite(o1,0);
 analogWrite(o2,200);
 delay(1000);
 
 analogWrite(o1,0);
 analogWrite(o2,0);
}
  


void updateEncoder(){
  
  digitalWrite(13,HIGH);
  //int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  //Serial.println(b);
  if (b>0){
    pos++;
  }else{
    pos--;
  }
 Serial.println(pos);
  digitalWrite(13,LOW);
}
void updateEncoder2(){
  
  digitalWrite(13,HIGH);
  //int a = digitalRead(ENCA);
  int a = digitalRead(ENCA);
  //Serial.println(b);
  if (a>0){
    pos--;
  }
 Serial.println(pos);
  digitalWrite(13,LOW);
}
