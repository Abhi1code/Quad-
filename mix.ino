#include <Wire.h>
#include<SoftwareSerial.h>
#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN_1 3
#define MOTOR_PIN_2 5
#define MOTOR_PIN_3 6
#define MOTOR_PIN_4 9

SoftwareSerial BT(10,11);

Servo motor1, motor2, motor3, motor4;
double esc1, esc2, esc3, esc4;
int speed1 = 1000;

int readdata;
char c;
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal ,acc_x_cal , acc_y_cal , acc_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc, angle_yaw_acc;

float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output, angle_yaw_output;

long loop_timer;
int temp;
int motor = 13;
int increase_by_100 = 1000;
int increase_by_10 = 0;

////////////////////////////////////////////////////////////////////////
// pid variables
/*working variables*/
unsigned long lastTime;
double pitchinput, rollinput, yawinput;
double pitchoutput, rolloutput, yawoutput;
double pitchsetpoint, rollsetpoint, yawsetpoint;
double pitchiterm, rolliterm, yawiterm; 
double pitchlastinput, rolllastinput, yawlastinput;
double pitchkp = 1.3;
double pitchki = 0.04;
double pitchkd = 20.0;
double rollkp = 1.3;
double rollki = 0.04;
double rollkd = 20.0;
double yawkp = 4.0;
double yawki = 0.02;
double yawkd = 0.0;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = true;
int motorstart = 1;
 
#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

/////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(13, OUTPUT);
  BT.begin(9600);

  motor1.attach(MOTOR_PIN_1);
  motor2.attach(MOTOR_PIN_2);
  motor3.attach(MOTOR_PIN_3);
  motor4.attach(MOTOR_PIN_4);
  
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL);

  delay(1000);
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);
  delay(2000);

  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  
  Wire.begin();                                                        //Start I2C as master
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    read_mpu_6050_data();                                             
    gyro_x_cal += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z offset to the gyro_z_cal variable
    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z;
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }

  // divide by 1000 to get avarage offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;                                                 
  acc_x_cal /= 1000;
  acc_y_cal /= 1000;
  acc_z_cal /= 1000;

  
  //Serial.begin(9600);
  
  //Serial.begin(115200);

  

  
  loop_timer = micros();                                               //Reset the loop timer

  digitalWrite(13, HIGH);
}

void loop(){
  
  while (BT.available())
  {
    delay(10);
    c = BT.read();
    
    
  }
  
  
    if(c == '0')increase_by_100 = 1000;
    if(c == '1')increase_by_100 = 1100;
    if(c == '2')increase_by_100 = 1200;
    if(c == '3')increase_by_100 = 1300;
    if(c == '4')increase_by_100 = 1400;
    if(c == '5')increase_by_100 = 1500;
    if(c == '6')increase_by_100 = 1600;
    if(c == '7')increase_by_100 = 1700;
    if(c == '8')increase_by_100 = 1700;
    if(c == '9')increase_by_100 = 1700;
    if(c == 'F')
    {
      if(increase_by_10 >= 90){
            
          }else{
            increase_by_10 += 1;
          }
    }
    if(c == 'B')
    {
      if(increase_by_10 >= 1){
            increase_by_10 -= 1;
          }
    }
    if(c == 'D')
    {
      increase_by_100 = 1000;
    }
    if(c == 'R')

          {
            motorstart = 2;
          }
          if(c == 'L')
          {
            motorstart = 1;
            motor1.writeMicroseconds(1000);
            motor2.writeMicroseconds(1000);
            motor3.writeMicroseconds(1000);
            motor4.writeMicroseconds(1000);
          }
          


  /*if(readdata.length()>0)
  {
    BT.print(readdata);
    readdata="";
    
  }*/
  //Serial.print("aaa/#");
  
  //digitalWrite(motor, LOW);
  read_mpu_6050_data();   
 //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611; 
  angle_yaw += gyro_z * 0.0000611;
  //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  angle_yaw_acc = asin((float)acc_z/acc_total_vector)* 57.296; 
  
  angle_pitch_acc -= 0.1;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.1;                                               //Accelerometer calibration value for roll
  angle_yaw_acc -= acc_z_cal;

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996  + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    //angle_yaw = angle_yaw * 0.9996 + angle_yaw_acc * 0.0004;
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                       //Set the gyro roll angle equal to the accelerometer roll angle 
    //angle_yaw = angle_yaw_acc;
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;
  //Serial.print(" | Pitch  = "); Serial.print(angle_pitch_output);
  //Serial.print("  | Roll  = "); Serial.print(angle_roll_output);
  //Serial.print("  | Yaw  = "); Serial.println(angle_yaw_output);

 
 //digitalWrite(motor, LOW);

 pitchinput = angle_pitch_output;
 rollinput = angle_roll_output;
 yawinput = angle_yaw_output;

 pitchsetpoint = 0.0;
 rollsetpoint = 0.0;
 yawsetpoint = 0.0;

 findpid();
 
 //Serial.print(" | PidPitch  = "); Serial.print(pitchoutput);
 //Serial.print(" | PidRoll  = "); Serial.print(rolloutput);
 //Serial.print(" | PidYaw  = "); Serial.print(yawoutput);
 
 int throttle = increase_by_100 + increase_by_10;

 //Serial.print(" | Throttle  = "); Serial.println(throttle);
 
 if (throttle > 1800){
  throttle = 1700;  
 }

if(motorstart == 2){
  

esc1 = throttle - pitchoutput + rolloutput - yawoutput;
esc2 = throttle + pitchoutput + rolloutput + yawoutput;
esc3 = throttle + pitchoutput - rolloutput - yawoutput;
esc4 = throttle - pitchoutput - rolloutput + yawoutput;

if (esc1 < 1100) esc1 = 1100;                                         //Keep the motors running.
    if (esc2 < 1100) esc2 = 1100;                                         //Keep the motors running.
    if (esc3 < 1100) esc3 = 1100;                                         //Keep the motors running.
    if (esc4 < 1100) esc4 = 1100;                                        //Keep the motors running.

    if(esc1 > 1900)esc1 = 1900;                                           //Limit the esc-1 pulse to 2000us.
    if(esc2 > 1900)esc2 = 1900;                                           //Limit the esc-2 pulse to 2000us.
    if(esc3 > 1900)esc3 = 1900;                                           //Limit the esc-3 pulse to 2000us.
    if(esc4 > 1900)esc4 = 1900;                                           //Limit the esc-4 pulse to 2000us


    motor1.writeMicroseconds(esc1);
    motor2.writeMicroseconds(esc2);
    motor3.writeMicroseconds(esc3);
    motor4.writeMicroseconds(esc4);

}
    
 

 while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
 loop_timer = micros();//Reset the loop timer
}




void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                             
}


void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}

//////////////////////////////////////////////////////////////////////////
void findpid()
{
  SetOutputLimits(-300, +300);
  Compute();
}

void Compute()
{
   if(!inAuto) return;
   
   unsigned long now = millis();
   //delay(100);
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      /*double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);*/
 
      /*Compute PID Output*/
      /*Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;*/
 
      /*Remember some variables for next time*/
      /*lastInput = Input;
      lastTime = now;*/
      
      //Pitch PID Controls
      SetTunings(pitchkp, pitchki, pitchkd);
      double pitcherror = pitchsetpoint - pitchinput;
      pitchiterm += (ki * pitcherror);
      if(pitchiterm > outMax) pitchiterm = outMax;
      else if(pitchiterm < outMin) pitchiterm = outMin;
      double pitchdinput = (pitchinput - pitchlastinput);

      pitchoutput = kp * pitcherror + pitchiterm - kd * pitchdinput;
      if(pitchoutput > outMax) pitchoutput = outMax;
      else if(pitchoutput < outMin) pitchoutput = outMin;

      pitchlastinput = pitchinput;


      //Roll PID Controls
      SetTunings(rollkp, rollki, rollkd);
      double rollerror = rollsetpoint - rollinput;
      rolliterm += (ki * rollerror);
      if(rolliterm > outMax) rolliterm = outMax;
      else if(rolliterm < outMin) rolliterm = outMin;
      double rolldinput = (rollinput - rolllastinput);

      rolloutput = kp * rollerror + rolliterm - kd * rolldinput;
      if(rolloutput > outMax) rolloutput = outMax;
      else if(rolloutput < outMin) rolloutput = outMin;

      rolllastinput = rollinput;


      

      //Yaw PID Controls
      SetTunings(yawkp, yawki, yawkd);
      double yawerror = yawsetpoint - yawinput;
      yawiterm += (ki * yawerror);
      if(yawiterm > outMax) yawiterm = outMax;
      else if(yawiterm < outMin) yawiterm = outMin;
      double yawdinput = (yawinput - yawlastinput);

      yawoutput = kp * yawerror + yawiterm - kd * yawdinput;
      if(yawoutput > outMax) yawoutput = outMax;
      else if(yawoutput < outMin) yawoutput = outMin;

      yawlastinput = yawinput;
 
      lastTime = now;
      //Serial.print('hhhhh');
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
 
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
 
/*void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}*/
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
 
   /*if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;*/
}
 
/*void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  
        Initialize();
    }
    inAuto = newAuto;
}*/
 
/*void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}*/
 
/*void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}*/














