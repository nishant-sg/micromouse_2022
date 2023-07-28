byte a = 0b00001111;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(a,BIN);
  delay(5000);
  a = a & 0b00000100;
  Serial.println(a,BIN);
  delay(5000);
  a = a | 0b10000011;
  Serial.println(a,BIN);
  delay(5000);
  
}
