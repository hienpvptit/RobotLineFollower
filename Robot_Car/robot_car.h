#ifndef _ROBOT_CAR_H_
#define _ROBOT_CAR_H_

#include "Arduino.h"

class Motor{
   public:
		void setMotor(uint8_t EN, uint8_t INx, uint8_t INy);
    
		void setSpeed(uint8_t speed);
    
		void setDirect(uint8_t direct);
    
		void enable();
    
  private:
		uint8_t EN, INx, INy;
		uint8_t speed, direct;
};

class Robot{
  public:
		Robot(uint8_t EN1, uint8_t IN1, uint8_t IN2, uint8_t IN3, uint8_t IN4, uint8_t EN2);
  		
		void forward(uint8_t speed);
    
		void back(uint8_t speed);
		
		void crolLeft(uint8_t speed);
		
		void crolRight(uint8_t speed);
		
		void turn(uint8_t speedL, uint8_t speedR);
    
		void stop();
    
  private:
		Motor leftMotor, rightMotor;
};



#endif
