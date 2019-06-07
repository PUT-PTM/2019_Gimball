HardwareSerial Serial1(PA10, PA9);
const int led=2;
int dc_step = 10;
int speedP=0;
int speedL=0;


int STBY = 9;
int direction =5;
//Motor A
int PWMA = 6; //Speed control
int AIN1 = 8; //Direction
int AIN2 = 7; //Direction

//Motor B
int PWMB = 12; //Speed control
int BIN1 = 10; //Direction
int BIN2 = 11; //Direction


void led_blink(){
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);

  
  Serial.begin(9600); //ST-LINK Virtual COM Port
  Serial1.begin(9600); // UART1

  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  bool inPin1 = HIGH;
  bool inPin2 = LOW;

  digitalWrite(STBY, HIGH);
  
  int speedtest = 100;
  digitalWrite(AIN1, inPin1);
  digitalWrite(AIN2, inPin2);
  analogWrite(PWMA, speedtest);
  digitalWrite(BIN1, inPin1);
  digitalWrite(BIN2, inPin2);
  analogWrite(PWMB, speedtest);
  delay(1000);
  digitalWrite(STBY, LOW);

  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);

}

void loop() {
  if(Serial1.available())
  {
    char a=Serial1.read();
    if(a=='1'){
      speedP=speedP+dc_step;
      speedL=speedL+dc_step;
    }
    else if(a=='2'){
      speedP=speedP-dc_step;
      speedL=speedL-dc_step;      
    }
    else if(a=='3'){
      speedP=speedP+dc_step;
    }
    else if(a=='4'){
      speedL=speedL+dc_step;
    }
    else if(a=='5'){
     speedP=speedL;
    }
    else if(a=='6'){
      speedL=speedP;
    }
    else if(a=='7'){
      digitalWrite(LED_BUILTIN,HIGH);
    }
    else if(a=='8'){
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  move(1,speedL);
  move(2,speedP);
  stop();
}

void move(int motor, int speed){
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise
  digitalWrite(STBY, HIGH); //disable standby
  
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;
  if(speed != 0) speed=(int)(speed*0.8) + 50;
  
  if(speed>0){
    direction=1;
  } else{
    direction=0;
  }
  if(direction==1){
    inPin1=HIGH;
    inPin2=LOW;
  }
  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
   }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }

}
void stop(){
  //enable standby
  digitalWrite(STBY, LOW);
}
