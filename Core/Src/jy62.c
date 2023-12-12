/*
 * jy62.c
 *
 *  Created on: Dec 10, 2023
 *      Author: sohweimeng
 */


#include"jy62.h"


uint8_t initAngle[3] = {0xFF,0xAA,0x52};
uint8_t calibrateAcce[3] = {0xFF,0xAA,0x67};
uint8_t setBaud115200[3] = {0xFF,0xAA,0x63};
uint8_t setBaud9600[3] = {0xFF,0xAA,0x64};
uint8_t setHorizontal[3] = {0xFF,0xAA,0x65};
uint8_t setVertical[3] = {0xFF,0xAA,0x66};
uint8_t sleepAndAwake[3] = {0xFF,0xAA,0x60};
UART_HandleTypeDef* jy62_huart;

struct Acce Accelerate;//储存加速度
struct Velo Velocity;//储存角速度
struct Angl Angle;  //储存角度值
struct Temp Temperature;  //储存温度值
/***********************接口****************************/

void jy62_Init(UART_HandleTypeDef *huart)
{
	jy62_huart = huart;
}

void jy62MessageRecord(void)
{


}

void SetBaud(int Baud)
{
	if(Baud == 115200)
	{
		HAL_UART_Transmit(jy62_huart,setBaud115200,3,100);
	}
	else if(Baud == 9600)
	{
		HAL_UART_Transmit(jy62_huart,setBaud115200,3,100);
	}
}

void SetHorizontal()
{
	HAL_UART_Transmit(jy62_huart,setHorizontal,3,100);
}

void SetVertical()
{
	HAL_UART_Transmit(jy62_huart,setVertical,3,100);
}

void InitAngle()
{
	HAL_UART_Transmit(jy62_huart,initAngle,3,100);
}

void Calibrate()
{
	HAL_UART_Transmit(jy62_huart,calibrateAcce,3,100);
}

void SleepOrAwake()
{
	HAL_UART_Transmit(jy62_huart,sleepAndAwake,3,100);
}


float GetRoll()
{
	return Angle.roll;
}
float GetPitch()
{
	return Angle.pitch;
}
float GetYaw()
{
	return Angle.yaw;
}

float GetTemperature()
{
	return Temperature.temperature;
}

float GetAccX()
{
	return Accelerate.accelerate_x;
}
float GetAccY()
{
	return Accelerate.accelerate_y;
}
float GetAccZ()
{
	return Accelerate.accelerate_z;
}

float GetVeloX()
{
	return Velocity.velocity_x;
}
float GetVeloY()
{
	return Velocity.velocity_y;
}
float GetVeloZ()
{
	return Velocity.velocity_z;
}


/***************************************************/

void DecodeAngle(uint8_t* jy62Message)
{
	Angle.roll = (float)((jy62Message[3]<<8)|jy62Message[2])/32768 * 180 ;
	Angle.pitch = (float)((jy62Message[5]<<8)|jy62Message[4])/32768 * 180 ;
	Angle.yaw =  (float)((jy62Message[7]<<8)|jy62Message[6])/32768 * 180 ;
}

void DecodeAccelerate(uint8_t* jy62Message)
{
	Accelerate.accelerate_x = (float)((jy62Message[3]<<8)|jy62Message[2])/32768 * 16 * g ;
	Accelerate.accelerate_y = (float)((jy62Message[5]<<8)|jy62Message[4])/32768 * 16 * g ;
	Accelerate.accelerate_z = (float)((jy62Message[7]<<8)|jy62Message[6])/32768 * 16 * g ;
}

void DecodeVelocity(uint8_t* jy62Message)
{
	Velocity.velocity_x = (float)((jy62Message[3]<<8)|jy62Message[2])/32768 * 2000 ;
	Velocity.velocity_y = (float)((jy62Message[5]<<8)|jy62Message[4])/32768 * 2000 ;
	Velocity.velocity_z = (float)((jy62Message[7]<<8)|jy62Message[6])/32768 * 2000 ;
}

void DecodeTemperature(uint8_t* jy62Message)
{
	Temperature.temperature = ((short)(jy62Message[9])<<8 | jy62Message[8])/340 +36.53;
}


void Decode(uint8_t* jy62Message)
{
    switch (jy62Message[1])
	{
	    case 0x51: DecodeAccelerate(jy62Message); break;
		case 0x52: DecodeVelocity(jy62Message);  break;
		case 0x53: DecodeAngle(jy62Message); break;
	}
	DecodeTemperature(jy62Message);
}
