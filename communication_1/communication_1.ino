char val;
char nal;
int x,y,z,u;
String inData;
String inData2;
String stringOne, stringTwo, stringThree;
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial1.begin(9600);
Serial2.begin(9600);
stringOne = String(" ,");

}

void loop() {
  
  while(Serial.available() > 0){
   char recieved = Serial.read();
        if (recieved == '\n')
        {
         stringTwo = inData;
  stringTwo += ",";
  stringTwo += "45";
  Serial.println(stringTwo);
  Serial1.print(stringTwo);
  Serial1.print('\n');
  
   inData = ""; 
        }
        else {
        inData += recieved;
      }
        
             

           
           // Clear recieved buffer

        
  
 /* stringTwo = String(val);
  stringTwo += stringOne;
  stringTwo += stringThree;
  stringTwo += ", 23, 34";
  Serial1.print(stringTwo);
  Serial1.print('\n');
  Serial.println(stringTwo);
Serial.flush();
  */
   

  
  }
   while(Serial2.available() > 0){
  
       
       x = Serial2.parseInt();
 if (Serial2.read() == '\n') {
    Serial.print('1');
     Serial.print('C');
 }
 else {
 y = Serial2.parseInt();
 if (Serial2.read() == '\n') {
  Serial.print('2');
 Serial.print('d');
}
 else { 
  z = Serial2.parseInt();
  if (Serial2.read() == '\n') {
  
   Serial.print('3');
   Serial.print('s');
 }
  else{
   u = Serial2.parseInt();
    if (Serial2.read() == '\n') {
    Serial.print('4');
Serial.print('m');
    }
  }
 }
 


}
 
 }
}
