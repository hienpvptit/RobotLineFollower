#include "robot_car.h"

#define EN1 5
#define EN2 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define SW1 12
#define SW2 11

Robot robot(EN2, IN4, IN3, IN1, IN2, EN1);

// A0: 50:300 / Right
// A1: 150:430 /
// A2: 80:300/
// A3: 350:550 /   Left

int sensor[4];
int value[4];
int thresh[4] = {300, 430, 300, 550};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  sensor[0] = 0; 
  sensor[1] = 0;
  sensor[2] = 0;
  sensor[3] = 0;
  robot.stop();
}

void sensor_read()
{
  sensor[0] = analogRead(A0);
  sensor[1] = analogRead(A1);
  sensor[2] = analogRead(A2);
  sensor[3] = analogRead(A3);

  for(int i=0; i<4; i++)
  {
    value[i] = (sensor[i]>=thresh[i]) ? 1 : 0;  
  }
}

int check()
{
  // Left -- Right
  if(value[3]==0 and value[2]==0 and value[1]==0 and value[0]==0) // Over
    return 99;
  //
  else if(value[3]==0 and value[2]==0 and value[1]==0 and value[0]==1) // Right Max -> Turn Right x2
    return -1;
  else if(value[3]==0 and value[2]==0 and value[1]==1 and value[0]==1) // Right Max -> Turn Right x2
    return -2;
  else if(value[3]==0 and value[2]==0 and value[1]==1 and value[0]==0) // Right -> Turn Right
    return -3;
  else if(value[3]==0 and value[2]==1 and value[1]==1 and value[0]==0) // balance -> Forward
    return 0;
  //
  else if(value[3]==1 and value[2]==0 and value[1]==0 and value[0]==0) // Left Max -> Turn Left x2
    return 1;
  else if(value[3]==1 and value[2]==1 and value[1]==0 and value[0]==0) // Left Max -> Turn Left x2
    return 2;
  else if(value[3]==0 and value[2]==1 and value[1]==0 and value[0]==0) // Left -> Turn Left
    return 3;
  //
  else return 100;  // Error
}

int old = 99;
int speed_fw = 100;
double Error = 0, SumError = 0, LastError = 0;
double BasePWM = 75; 
double Kp = 10;       // Proporsional 
double Ki = 0.1;      // Integral     
double Kd = 30;      // Diferensial
double Ts = 1;        // Time sampling
int Kec_Max = 255;
int Kec_Min = 0;

void lineFollow() {
sensor_read();
  int NilaiPosisi = check();
  Serial.println(NilaiPosisi);
  if(NilaiPosisi==100 or NilaiPosisi==99) return;
  int SetPoint = 0;                      // Setpoint yang diinginkan
  Error = SetPoint - NilaiPosisi;        // Error
  double DeltaError = Error - LastError; // Delta Error (Selisih error sekarang e(t) dengan error sebelumya e(t-1))
  SumError += LastError;                 // Akumulasi error
  double P = Kp * Error;                 // Kontrol proporsional
  double I = Ki * SumError * Ts;         // Kontrol integral
  double D = ((Kd / Ts) * DeltaError);   // Kontrol derivative
  LastError = Error;                     // Error sebelumnya
  int  outPID = P + I + D;                    // Output PID
  double motorKi = BasePWM - outPID;     // Motor Kiri
  double motorKa = BasePWM + outPID;     // Motor Kanan
  /*** Pembatasan kecepatan ***/
  if (motorKi > Kec_Max)motorKi = Kec_Max;
  if (motorKi < Kec_Min)motorKi = Kec_Min;
  if (motorKa > Kec_Max)motorKa = Kec_Max;
  if (motorKa < Kec_Min)motorKa = Kec_Min;
  robot.turn(motorKa, motorKi);
  Serial.print(motorKa); Serial.print(" "); Serial.println(motorKi);
}

void loop() {
  // put your main code here, to run repeatedly:
  //while(digitalRead(SW1)==1);
//  sensor_read();
//  int ck = check();
//  robot.forward(80);
//  while(ck==99){sensor_read(); ck=check();};
//  while(1)
//  {
//    sensor_read();
//    ck = check();
//    switch(ck)
//    {
//      case 99:
//        if(old==99 || old==0)
//          robot.stop();
//        else if(old>0)
//        {
//          while(1)
//          {
//            robot.turn(0, 140);
//            sensor_read();
//            if(check()!=99)
//              break;  
//          }  
//        }
//        else if(old<0)
//        {
//          while(1)
//          {
//            robot.turn(140, 0);
//            sensor_read();
//            if(check()!=99)
//              break;  
//          }  
//        }
//        else
//          robot.stop();
//        break;
//      case 100:
//        robot.forward(100);
//        break;
//      case 0:
//        robot.forward(100);
//        break;
//      case 1: // turn left x2
//        robot.turn(40, 100);
//        break;
//      case 2: // turn left x2
//        robot.turn(40, 100);
//        break;
//      case 3: // turn left
//        robot.turn(80, 100);
//        break;
////        
//      case -1: // turn right x2
//        robot.turn(100, 60);
//        break;
//      case -2: // turn right x2
//        robot.turn(100, 60);
//        break;
//      case -3: // turn right
//        robot.turn(100, 80);
//        break;
//    }
//    
//    if(old!=ck)
//      old = ck;
//  }
  int ck;
  while(digitalRead(SW1)==1);
  while(true)
  {
    lineFollow();
//    sensor_read();
//    ck = check();
//    switch(ck)
//    {
//      case 0: // forward
//        robot.forward(speed_fw);
//        break;
//      case 1: // turn left x2
//        robot.turn(speed_fw/4, speed_fw);
//        break;
//      case 2: // turn left x2
//        robot.turn(speed_fw/4, speed_fw);
//        break;
//      case 3: // turn left
//        robot.turn(speed_fw*2/3, speed_fw);
//        break;
////        
//      case -1: // turn right x2
//        robot.turn(speed_fw, speed_fw/4);
//        break;
//      case -2: // turn right x2
//        robot.turn(speed_fw, speed_fw/4);
//        break;
//      case -3: // turn right
//        robot.turn(speed_fw, speed_fw*2/3);
//        break;
//    } 
  }
}
