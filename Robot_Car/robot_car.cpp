#include "robot_car.h"

void Motor::setMotor(uint8_t EN, uint8_t INx, uint8_t INy){
	this->EN = EN;
    this->INx = INx;
    this->INy = INy;
    pinMode(this->EN, OUTPUT);
    pinMode(this->INx, OUTPUT);
    pinMode(this->INy, OUTPUT);
    this->direct = 0;
    this->speed = 0;
}

void Motor::setSpeed(uint8_t speed){
	this->speed = speed;
}

void Motor::setDirect(uint8_t direct){
	this->direct = direct;
}

void Motor::enable(){
	if(this->direct==1){
    	digitalWrite(this->INx, HIGH);
    	digitalWrite(this->INy, LOW);
        analogWrite(this->EN, this->speed);   
    }
    else if(direct==0){
        analogWrite(this->EN, 0);  
    }
    else if(direct==2){
        digitalWrite(this->INx, LOW);
    	digitalWrite(this->INy, HIGH);
        analogWrite(this->EN, this->speed);
    }
    else;
}


//

Robot::Robot(uint8_t EN1, uint8_t IN1, uint8_t IN2, uint8_t IN3, uint8_t IN4, uint8_t EN2){
    this->leftMotor.setMotor(EN1, IN1, IN2);
    this->rightMotor.setMotor(EN2, IN3, IN4);
}

void Robot::forward(uint8_t speed){
    this->leftMotor.setDirect(1);
    this->leftMotor.setSpeed(speed);
    this->rightMotor.setDirect(1);
    this->rightMotor.setSpeed(speed);
    this->leftMotor.enable();
    this->rightMotor.enable();
}

void Robot::back(uint8_t speed){
    this->leftMotor.setDirect(2);
    this->leftMotor.setSpeed(speed);
    this->rightMotor.setDirect(2);
    this->rightMotor.setSpeed(speed);
    this->leftMotor.enable();
    this->rightMotor.enable();
}

void Robot::crolLeft(uint8_t speed){
	this->leftMotor.setDirect(2);
	this->leftMotor.setSpeed(speed);
	this->rightMotor.setDirect(1);
	this->rightMotor.setSpeed(speed);
	this->leftMotor.enable();
	this->rightMotor.enable();
}

void Robot::crolRight(uint8_t speed){
	this->leftMotor.setDirect(1);
	this->leftMotor.setSpeed(speed);
	this->rightMotor.setDirect(2);
	this->rightMotor.setSpeed(speed);
	this->leftMotor.enable();
	this->rightMotor.enable();
}


void Robot::turn(uint8_t speedL, uint8_t speedR){
	this->leftMotor.setDirect(1);
	this->leftMotor.setSpeed(speedL);
	this->rightMotor.setDirect(1);
	this->rightMotor.setSpeed(speedR);
	this->leftMotor.enable();
	this->rightMotor.enable();
}

void Robot::stop(){
    this->leftMotor.setSpeed(0);
    this->rightMotor.setSpeed(0);
    this->leftMotor.enable();
    this->rightMotor.enable();
}

