void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!Serial.available());
  char c = Serial.read();
  Serial.println(c);

  //check the serial monitor and see if w,a,s,d pressed then execute code
  
}
