void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(49, INPUT);
pinMode(32, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 if(digitalRead(49) == LOW){
  Serial.println("49 LOW");
    digitalWrite(32, LOW);
 }
 else if(digitalRead(49) == HIGH){
  Serial.print("49 HIGH");
  digitalWrite(32, HIGH);
 }
}
