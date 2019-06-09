// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <Arduino.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "../lib/I2Cdev/I2Cdev.h"

#include "../lib/MPU6050/MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
bool debug = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// define motors pins
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
bool inPin1 = 0x0;
bool inPin2 = 0x0;

// ports for UART1
HardwareSerial Serial1(PA10, PA9);

// variables for pid calculations
float elapsedTime, time, timePrev;
float rad_to_deg = 180 / 3.141592654;

float PIDvalue, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

/////////////////PID CONSTANTS/////////////////
double kp = 750.0;
double ki = 0.1;
double kd = 0.5;
///////////////////////////////////////////////

float desired_angle = 0.007; //This is the angle in which we want the balance to stay steady

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                        FUNCTIONS                         ===
// ================================================================

void led_blink(int times, int duration)
{
    for (int i = 0; i < times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(duration);
        digitalWrite(LED_PIN, LOW);
        delay(duration);
    }
}

void led_blink(int times, int duration, int duration_off)
{
    for (int i = 0; i < times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(duration);
        digitalWrite(LED_PIN, LOW);
        delay(duration_off);
    }
}

void move(int motor, float speed)
{
    //Move specific motor at speed and direction
    //motor: 0 for B 1 for A
    //speed: 0 is off, and 255 is full speed
    //direction: 0 clockwise, 1 counter-clockwise
    //digitalWrite(STBY, HIGH); //disable standby

    if (speed > 0.0)
    {
        inPin1 = HIGH;
        inPin2 = LOW;
        speed = (speed * 0.8) + 50;
    }
    else if (speed < 0.0)
    {
        inPin1 = LOW;
        inPin2 = HIGH;
        speed = ((speed * 0.8) - 50) * (-1);
    }
    if (motor == 1)
    {
        digitalWrite(AIN1, inPin1);
        digitalWrite(AIN2, inPin2);
        analogWrite(PWMA, (int)speed);
    }
    else
    {
        digitalWrite(BIN1, inPin1);
        digitalWrite(BIN2, inPin2);
        analogWrite(PWMB, (int)speed);
    }
}

void start()
{
    //enable standby
    digitalWrite(STBY, HIGH);
}

void stop()
{
    //enable standby
    digitalWrite(STBY, LOW);
    while (1)
        led_blink(1, 250);
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // define led pinout
    pinMode(LED_PIN, OUTPUT);

    // initialize serial communication
    Serial.begin(9600); //ST-LINK Virtual COM Port
    //Serial1.begin(9600); // UART1

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    if (mpu.testConnection())
    {
        Serial.println(F("MPU6050 connection successful"));
        led_blink(1, 100);
    }
    else
    {
        Serial.println(F("MPU6050 connection failed"));
        led_blink(2, 100, 250);
    }

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-675);
    mpu.setYAccelOffset(771);
    mpu.setZAccelOffset(1144);
    mpu.setXGyroOffset(1);
    mpu.setYGyroOffset(-43);
    mpu.setZGyroOffset(32);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        led_blink(1, 100);
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        led_blink(4, 50);
        led_blink(1, 50, 250);
    }

    // pid
    time = millis(); //Start counting time in milliseconds

    Serial.println(F("Configuring pins for motors..."));
    // motors
    pinMode(STBY, OUTPUT);

    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    Serial.println(F("Running motors test..."));

    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 100);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 100);
    delay(1000);
    digitalWrite(STBY, LOW);

    Serial.println(F("Test finished"));

    led_blink(1, 1000, 0);
    start();
}

void loop()
{

    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        if (mpuInterrupt && fifoCount < packetSize)
        {
            // try to get out of the infinite loop
            fifoCount = mpu.getFIFOCount();
        }
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // getting data from DMP
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        // check verticality
        if (abs(ypr[2]) > 60.0)
        {
            stop();
        }

        // calculate pid
        timePrev = time; // the previous time is stored before the actual time read
        time = millis(); // actual time read
        elapsedTime = (time - timePrev) / 1000;

        error = ypr[2] - desired_angle;
        pid_p = kp * error;
        if (-3.0 < error && error < 3.0)
        {
            pid_i = pid_i + (ki * error);
        }
        pid_d = kd * ((error - previous_error) / elapsedTime);

        PIDvalue = pid_p + pid_i + pid_d;

        if (PIDvalue < -255)
        {
            PIDvalue = -255;
            stop();
        }
        if (PIDvalue > 255)
        {
            PIDvalue = 255;
            stop();
        }

        move(1, PIDvalue);
        move(2, PIDvalue);

        // if (Serial1.available())
        // {
        //     bt = Serial1.readString();
        //     char type = bt[0];
        //     String b = bt.substring(2);
        //     if (type == 'p')
        //         kp = b.toFloat();
        //     if (type == 'i')
        //         ki = b.toFloat();
        //     if (type == 'd')
        //         kd = b.toFloat();
        // }

        // Serial.print("ypr\t");
        // Serial.print(ypr[0] * 180 / 3.14);
        // Serial.print("\t");
        // Serial.print(ypr[1] * 180 / 3.14);
        // Serial.print("\t");
        // Serial.print(ypr[2] * 180 / 3.14);
        // Serial.print("\t");
        // Serial.print(kp);
        // Serial.print("\t");
        // Serial.print(ki);
        // Serial.print("\t");
        // Serial.print(kd);
        // Serial.print("\t");
        // Serial.print(F("PID "));
        // Serial.print(PIDvalue);
        // Serial.println(F(")"));

        previous_error = error; //Remember to store the previous error.
    }
}
