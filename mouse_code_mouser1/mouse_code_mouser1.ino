 #include <SoftwareSerial.h>
 #include<Mouse.h>
SoftwareSerial Blan(8, 9); // RX, TX
int firstbyte, secondbyte;
int x, y;
void setup() {
Blan.begin(9600);
Mouse.begin();
x = 0;
y = 0;

}

void loop() {
 
   while(Blan.available() > 0){
  delay(1);
int recieved = Blan.parseInt();
firstbyte = recieved;
 if (Blan.read() == '\n') {
    if(firstbyte == 1){
    Mouse.click();//CLICK
    }
    else if (firstbyte == 2){
    Mouse.click(MOUSE_RIGHT);//Doubleclick
    }
    else {
    //ERROR!
    }
 }
 else {
   
  int recieved2 = Blan.parseInt();
  secondbyte = recieved2;
  y = secondbyte;
 if (Blan.read() == '\n') {
  if(firstbyte == 3){
    
  Mouse.move(0, 0, y);//SCROLL WHERE Y IS THE -ACCELY.
  }
  else{
  x = firstbyte;
 Mouse.move(x, y, 0);//MOUSE.MOVE(firstbyte, secondbyte). 
  }
}
 
  }
 firstbyte = 0;
 secondbyte = 0;
 
 



 
 }
}
