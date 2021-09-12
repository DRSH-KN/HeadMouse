#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"

MPU6050 accelgyro(0X69);//I2C addreess for mpu6050 is 0X69

float X=0;
float Y=0;
float Z=0;


float prevX=0;
float prevY=0;


float accelX=0;
float accelY=0;

float scalerX = 10;
float scalerY = 8;


int loopcount = 0;
int blowcount = 0;


void setup() {
  
  Wire.begin();// put your setup code here, to run once:
  Serial.begin(38400);
  accelgyro.initialize();

  



}

void loop() {
accelgyro.getAcceleration(&X, &Y, &Z);


accelX = ((X - prevX)/2);
accelY = ((Y - prevX)/2);

prevX = X;
prevY = Y;
    
accelX = accelX * scalerX;
accelY = accelY * scalerY;
      


if(accelX != 0 || accelY != 0){ 
   String mousecode;

   mousecode = accelX + "," + accelY;//accelX,AccelY - "10,-5"
   
   Serial.print(mousecode);
   Serial.print('\n');   
 }    

 
          
          if( analogRead(A0) > 600 ){  
             loopcount = loopcount +1;//loopcount++
          
              if (loopcount = 10){
                   blowcount = blowcount + 1;//1 - Left Click
         
                   }
                else if(loopcount = 20){
                    blowcount = blowcount +1;//2 - Right Click
                }
      
       else if(analogRead(A0) < 600){
       
         if(loopcount >10 && loopcount <20){
        Serial.print(blowcount);// blowcount = 1 for click
          Serial.print('\n');//execute click
       loopcount = 0;
      blowcount = 0; 
      }
      else if( loopcount > 20){
       Serial.print(blowcount);// blowcount == 2 for right click
       Serial.print('\n');//right click
       loopcount = 0;
      blowcount = 0; 
      }
      }
 
}
}
