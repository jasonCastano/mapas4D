char x = 'a';
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    x = Serial.read();
    Serial.println(x);
    }
}
