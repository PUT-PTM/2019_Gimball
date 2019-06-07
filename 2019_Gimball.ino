HardwareSerial Serial1(PA10, PA9);
const int led=2;
int speedP=0;
int speedL=0;
int direction=5;

//Motor A
int PWMA = 3; //Speed control
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction

//Motor B
int PWMB = 5; //Speed control
int BIN1 = 11; //Direction
int BIN2 = 12; //Direction

void setup() {
  Serial.begin(9600); //ST-LINK Virtual COM Port
  Serial1.begin(9600); // UART1

  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  pinMode(LED_BUILTIN,OUTPUT);

}

void loop() {
  if(Serial1.available())
  {
    char a=Serial1.read();
    if(a=='1'){
      speedP=speedP+10;
      speedL=speedL+10;
    }
    else if(a=='2'){
      speedP=speedP-10;
      speedL=speedL-10;      
    }
    else if(a=='3'){
      speedP=speedP+10;
    }
    else if(a=='4'){
      speedL=speedL+10;
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
  move(1,speedP);
  move(2,speedL);
}

void move(int motor, int speed){
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise
  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;
  
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
