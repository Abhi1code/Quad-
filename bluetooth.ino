#include<SoftwareSerial.h>
SoftwareSerial BT(0,1);
String readdata;
void setup()
{
  BT.begin(9600);
  Serial.begin(9600);
  
}
void loop()
{
  while (BT.available())
  {
    delay(10);
    char c = BT.read();
    readdata += c;
    
  }
  if(readdata.length()>0)
  {
    Serial.print(readdata);
    readdata="";
    
  }
  Serial.print("aaaa#");
}

