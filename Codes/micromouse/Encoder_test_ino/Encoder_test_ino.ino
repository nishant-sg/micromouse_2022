#include <LiquidCrystal.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#include <Wire.h>

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

#define ENCA 4
#define ENCB 7
int o1 = 9;
int o2 = 10;
long pos = 0;
long pos1 = 0;

void setup(){
    lcd.init(); // initialize the lcd
  lcd.backlight();
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(13,INPUT);
  digitalWrite(13,LOW);
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder, RISING); 
  attachInterrupt(1, updateEncoder2, RISING);
  

}

void loop(){
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print(pos);        // print message at (0, 0)
  lcd.setCursor(0, 1);         // move cursor to   (0, 0)
  lcd.print(pos1); 
}  


void updateEncoder(){
  int b = digitalRead(ENCB);
  if (b>0){
    pos++;
  }else{
    pos--;
  }
}
void updateEncoder2(){
  int b = digitalRead(ENCA);
  if (b>0){
    pos1++;
  }else{
    pos1--;
  }
}
