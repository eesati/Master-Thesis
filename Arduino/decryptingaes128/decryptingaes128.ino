void setup() {
  Serial.begin(115200);
 Serial.setDebugOutput(true);
  unsigned char output[100]="y�9���}�K�-��";
    for (int i = 0; i < 100; i++) {
 
    char str[3];
 
    sprintf(str, "%02x", (int)output[i]);
    Serial.print(str);
  }
  
  
  
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
