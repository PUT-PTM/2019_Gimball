#include <Arduino.h>

HardwareSerial Serial1(PA10, PA9);

#define LED_BUILTIN 13

#define STBY 9

#define PWMA 6 //Speed control
#define AIN1 8 //Direction
#define AIN2 7 //Direction

#define PWMB 12 //Speed control
#define BIN1 10 //Direction
#define BIN2 11 //Direction

#define MOTOR_STEP 10

int speedP = 0;
int speedL = 0;

bool s1_available = true;

int direction = 5;

void move(int motor, int speed)
{
    //Move specific motor at speed and direction
    //motor: 0 for B 1 for A
    //speed: 0 is off, and 255 is full speed
    //direction: 0 clockwise, 1 counter-clockwise
    digitalWrite(STBY, HIGH); //disable standby

    boolean inPin1 = LOW;
    boolean inPin2 = HIGH;

    if (speed != 0)
        speed = (int)(speed * 0.8) + 50;

    if (speed > 0)
    {
        direction = 1;
    }
    else
    {
        direction = 0;
    }
    if (direction == 1)
    {
        inPin1 = HIGH;
        inPin2 = LOW;
    }
    if (motor == 1)
    {
        digitalWrite(AIN1, inPin1);
        digitalWrite(AIN2, inPin2);
        analogWrite(PWMA, speed);
    }
    else
    {
        digitalWrite(BIN1, inPin1);
        digitalWrite(BIN2, inPin2);
        analogWrite(PWMB, speed);
    }
}
void stop()
{
    //enable standby
    digitalWrite(STBY, LOW);
}

void led_blink(int a, int b)
{
    for (int i = 0; i < a; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(b);
        digitalWrite(LED_BUILTIN, LOW);
        delay(b);
    }
}

void led_blink(int a, int b, int c)
{
    for (int i = 0; i < a; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(b);
        digitalWrite(LED_BUILTIN, LOW);
        delay(c);
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    led_blink(4, 100);

    Serial.begin(9600);  //ST-LINK Virtual COM Port
    Serial1.begin(9600); // UART1

    Serial.write("Configuring pins for motors...\n");

    pinMode(STBY, OUTPUT);

    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    boolean inPin1 = HIGH;
    boolean inPin2 = LOW;

    Serial.write("Running motors test...\n");

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

    Serial.write("Test finished\n");

    led_blink(3, 100);
    led_blink(1, 1000);
}

void loop()
{
    if (Serial1.available())
    {
        s1_available = true;
        char a = Serial1.read();
        if (a == '1')
        {
            speedP += MOTOR_STEP;
            speedL += MOTOR_STEP;
        }
        else if (a == '2')
        {
            speedP -= MOTOR_STEP;
            speedL -= MOTOR_STEP;
        }
        else if (a == '3')
        {
            speedP += MOTOR_STEP;
        }
        else if (a == '4')
        {
            speedL += MOTOR_STEP;
        }
        else if (a == '5')
        {
            speedP = speedL;
        }
        else if (a == '6')
        {
            speedL = speedP;
        }
        else if (a == '7')
        {
            digitalWrite(LED_BUILTIN, HIGH);
        }
        else if (a == '8')
        {
            digitalWrite(LED_BUILTIN, LOW);
        }
    }
    else
    {
        if (s1_available == true)
        {
            s1_available = false;
            Serial.write("Serial1 is not available\n");
        }
        led_blink(1, 100, 500);
    }
    move(1, speedL);
    move(2, speedP);
    stop();
}