#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 9

Servo motor;
int speed1 = 1050;

void setup() {
  Serial.begin(9600);
  
  Serial.println("Power the ESC now and send any key when ready");

  motor.attach(MOTOR_PIN);
  motor.writeMicroseconds(MAX_SIGNAL);

  // Wait for input
  //while (!Serial.available());
  //Serial.read();
  delay(1000);
  motor.writeMicroseconds(MIN_SIGNAL);
  delay(4000);

  motor.writeMicroseconds(1600); //start changing motor speed
  delay(3000);
  motor.writeMicroseconds(1000);

  Serial.println("Start loop...");
}


void loop() { 
  //test(); 
  /*if(Serial.available() > 0) {
    int incomingByte = Serial.read();
    if(incomingByte == 43)      // sends the character '+'
      speed1+=50;                  
    else if(incomingByte == 45) // sends the character '-'
      speed1-=50;                  
    else if(incomingByte == 48) // sends the character '0'
      speed1 = MIN_SIGNAL;         


    Serial.print("Set speed to: ");
    Serial.println(speed1);

    motor.writeMicroseconds(speed1);
  }*/
}
void test()
{
    for (int i = 1200; i <= 1600; i += 50) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        
        motor.writeMicroseconds(i);
        //motB.writeMicroseconds(i);
        //motC.writeMicroseconds(i);
        //motD.writeMicroseconds(i);
        
        delay(200);
       
        }
    //}

    Serial.println("STOP");
    //motA.writeMicroseconds(MIN_PULSE_LENGTH);
    //motB.writeMicroseconds(MIN_PULSE_LENGTH);
    //motC.writeMicroseconds(MIN_PULSE_LENGTH);
    //motD.writeMicroseconds(MIN_PULSE_LENGTH);
}

