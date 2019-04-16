HardwareSerial Serial1(PA10, PA9);

char cmd[5];
int cmdIndex = 0;

void setup() {
  Serial.begin(9600); //ST-LINK Virtual COM Port
  Serial1.begin(9600); // UART1
  
  pinMode(LED_BUILTIN,OUTPUT);

}

void exeCmd() {
  Serial.println("CP2");
  Serial.println(cmd);
  if(strcmp(cmd, "led 1")==0){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(1000);
  } 
  if(strcmp(cmd, "led 0")==0){
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
  }
 
}

void loop() {
  if (Serial1.available())
  {
    Serial.println("CP1");
    char c = (char)Serial1.read();
    if (c == '\n') {
      cmd[cmdIndex] = 0;
      exeCmd();
      cmdIndex = 0;
    } else {
      cmd[cmdIndex] = c;
      if (cmdIndex < 5) cmdIndex++;
    }
  } 
}
