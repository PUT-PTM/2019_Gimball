HardwareSerial Serial1(PA10, PA9);

void setup() {
  Serial.begin(9600); //ST-LINK Virtual COM Port
  Serial1.begin(9600); // UART1

  pinMode(LED_BUILTIN,OUTPUT);

}

void loop() {
  

}
