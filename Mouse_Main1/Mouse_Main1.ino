/*Arduino Pro mini>>>>>>>>>>>>>>> MPU6050
          Vcc>>>>>>>>>>>>>>>>>>>>>Vcc
          GND>>>>>>>>>>>>>>>>>>>>GND
          GND>>>>>>>>>>>>>>>>>>>>AD0
          A4>>>>>>>>>>>>>>>>>>>>SDA
          A5>>>>>>>>>>>>>>>>>>>>SLA
          INT>>>>>>>>>>>>>>>>>>>>Pin2
          A0>>>>>>>>>>>>>>>>>>>>input from sound
          D8>>>>>>>>>>>>>>>>>>>>>TX
          D7>>>>>>>>>>>>>>>>>>>>RX
 */
 
#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 8); // RX, TX
#define BOUNCE
#ifdef BOUNCE
#include <Bounce.h>
  
#define L_MOUSE_BUTTON 5
//#define LED 6 is defined in MPU6050 code as LED_PIN

// Instantiate a Bounce object with a 5 millisecond debounce time
Bounce deBouncer = Bounce( L_MOUSE_BUTTON, 5 );
#endif
/*
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// Changelog:
//     2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//     2012-06-20 - improved FIFO overflow handling and simplified read process
//     2012-06-19 - completely rearranged DMP initialization code and simplification
//     2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//     2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//     2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                - add 3D math helper file to DMP6 example sketch
//                - add Euler output and Yaw/Pitch/Roll output formats
//     2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//     2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//     2012-05-30 - basic DMP initialization working
*/

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

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

#include <avr/wdt.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using mySerial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */




#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

	float X;
	float Y;
	float Z;

        float lastXYPR;
        float lastYYPR;
        float lastZYPR;
        
	float lastXAccel;
	float lastYAccel;
	float lastZAccel;

        int loopcount = 0;
        int loopval = 0;
        int loopval1 = 0;
        int blowcount = 0;
     

    bool run = false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    wdt_disable();
      // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // initialize mySerial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    mySerial.begin(38400);
    while (!mySerial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
  //  mySerial.println(F("Initializing I2C devices..."));
    mpu.initialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  //  mpu.setXGyroOffset(220);
   // mpu.setYGyroOffset(76);
   // mpu.setZGyroOffset(-85);
   // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // verify connection
 //   mySerial.println(F("Testing device connections..."));
  mySerial.print(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/*
    // wait for ready
    mySerial.print(F("\nSend any character to begin ASSISTIVE PROTOTYPE wand mouse: "));
    while (mySerial.available() && mySerial.read()); // empty buffer
    while (!mySerial.available());                 // wait for data
    while (mySerial.available() && mySerial.read()); // empty buffer again
*/
    run = true;


    // load and configure the DMP
    mySerial.print(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
      //  mySerial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
     //   mySerial.print(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
     //   mySerial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
     //   mySerial.print(F("DMP Initialization failed (code "));
       // mySerial.print(devStatus);
      //  mySerial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

#ifdef BOUNCE
  pinMode(L_MOUSE_BUTTON,INPUT);
//  pinMode(LED,OUTPUT);   // already done via MPU6050 code
#endif

        // *** >>>>>>> should read values & use those here!!!!!
	lastXYPR = 0;
	lastYYPR = 0;
	lastZYPR = 0;

        lastXAccel = 0;
         lastYAccel = 0;
          lastZAccel = 0;
        

      wdt_enable(WDTO_4S);
      
        
        
        
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
     
     wdt_reset();
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
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


    // using run & mySerial text to allow DISABLING of reading data AND moving the mouse
     while (mySerial.available() && mySerial.read()){ // empty buffer again
        run = false;
       // inc++;
    }
    if (run) {
      
      loopcount = loopcount + 1;//This is essentially a timer. Everytime the loop runs, it will increase by one, just like a clock.

        getGyro();

       moveMouseRelative(X, Y);
    }

    // now read & act on mouse switch(es)
     #ifdef BOUNCE
     // Update the dedeBouncer
      deBouncer.update ( );

     // Get the update value
     int MouseLeft = deBouncer.read();

     // Turn on or off the LED and mouse click
     
     //   digitalWrite(LED_PIN, HIGH );
     
    
    // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
    #endif


}


inline void getGyro (){
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //mySerial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
            mpu.dmpGetQuaternion(&q, fifoBuffer); //procure the Quaternion, Euler, gravity and Yaw, Pitch, Roll values from the DMP module
            mpu.dmpGetEuler(euler, &q);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
     

            X =(ypr[0] * 180/M_PI) - lastXYPR; // These essentially output the change in either X, Y, or Z. They will be near 0 if not moving and go up to 20 when moving
            Y =(ypr[2] * 180/M_PI) - lastYYPR;
            Z =(ypr[1] * 180/M_PI) - lastZYPR;
       

            lastXYPR = (ypr[0] * 180/M_PI);// This is the last value of X,Y or Z. It is subtracted from the present value to give the change in value. This way, subtracting of offset or integration against time is not required
            lastYYPR = (ypr[2] * 180/M_PI);       
            lastZYPR = (ypr[1] * 180/M_PI);
          
          //mySerial.print("X=");            
         // mySerial.print(X);
          //mySerial.print("\n");
          //mySerial.print("Y=");
          //mySerial.print(Y);
         // mySerial.print("\n");


    }
}

inline void moveMouseRelative(long accelX, long accelY){

 if(accelX != 0 || accelY != 0){ 
  int x, y;
   x = accelX * 10 ;
   y = accelY * 8 ;
          // This command takes the X and Y values and multiplies them into the appropriate hard-coded scaling factor to move the mouse 
          //Mouse.move((accelX * 10), (accelY * 8),0);
   String val, zal;
   zal = ", ";
   val = x + zal + y;
   
   mySerial.print(val);
   mySerial.print('\n');   
 }    
          
          if(analogRead(A0) > 300 ) // The sound sensor reads a value greater than 300 when blown into.
          {  
           
         loopval = loopcount - loopval ; //This takes the variable "loopval" and finds the difference between itself and the clock. 
         
         if (loopval < 10) //If loopval < 10, it means that the person is currently blowing. So, update blowcount and set loppval to the value of the clock so the same condition will work the next time
         {
         blowcount = blowcount + 1;
         loopval = loopcount;
         }
         else if (loopval > 10) // If loopval > 10, the person has just begun blowing. So, left click at that place and reset blowcount and loopval
         {
         mySerial.print("1");
       mySerial.print('\n');  
         blowcount = 0;
         loopval = loopcount;
         }
          
        }
          else if (analogRead(A0) < 300) // If it is less than 300, the person is not blowing
          {
            
          loopval1 = loopcount - loopval; // Find the difference between this variable and the clock
          
          if(loopval1 < 10 )// If it is < 10, the person has just now stopped blowing and an action must be taken.
          {
          if(blowcount > 30) // If blowcount is > 30, it was a long blow and the person wants to right-click. So, do so and then reset blowcount
          {
          //mySerial.print(blowcount);
          //mySerial.print("\n");  
          mySerial.print("2");
          mySerial.print('\n');
          blowcount = 0;
          }
          
          if(blowcount > 15 && blowcount < 30) // If blowcount is between 15 and 30, it was an intermediate blow and the person wants to double-click (the first click is the single cick that happens when the person starts blowing). 
          {                                    //So, do so and then reset blowcount   
         mySerial.print("1");
       mySerial.print('\n'); 
          blowcount = 0;
          
          }
          
          }
          
          }
          
        if(analogRead(A0) > 300 && (accelY > 0.5 || accelY < -0.5))// If the person is blowing and moving up and down, then scroll the screen by the amount he/she is moving.
    
      {
     if (accelY != 0){
       String aal, bal;
     int scl = -accelY ;
     aal = "3, ";
    bal = aal + scl;
     mySerial.print(bal);
     mySerial.print('\n');
        //  Mouse.move(0,0,-accelY);
     }
      }    
         
        
        
 


    
  }
