/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"usbd_cdc_if.h"
#include"string.h"
#include"stdio.h"
#include"math.h"
#include"stdlib.h"
#include"stdbool.h"
#include "AESK_Data_Pack_lib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

static void Parameter_Receive_PC(void);			//c# üzerinden gelen PID değerlerini alan fonksiyon.
static void Parameter_Transmit_PC(void);		//Parametleri gönderdiğimiz fonksiyon.
static void All_Parameters_Send_Tekseferde(void);
static void Error_Flag_Function(void);
static void Button_Kontrol_Function(void);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68 << 1
//#define PWR_MGMT_1_REG 0x68
#define SMPLRT_DIV_REG 0x19
#define GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C
#define RAD_TO_DEG 57.295779513082320876798154814105

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

//*************************************mpu6050 değişkenler************************
uint8_t data;
uint8_t buffer[2],tuffer[6],cuffer[6];
int16_t gyro_raw[3],acc_raw[3];
float gyro_cal[3];
uint16_t acc_total_vector;
float angle_pitch_gyro,angle_roll_gyro;
float angle_pitch_acc,angle_roll_acc;
float angle_pitch,angle_roll;
uint16_t raw_temp;
float temp;
int i;
float prevtime,prevtime1,time1,elapsedtime1,prevtime2,time2,elapsedtime2,Elapsedtime;
HAL_StatusTypeDef set_gyro;

//*********************************************************************************


typedef union
{
	struct
	{
		float data;  		// Float olan bir veri 4byte'dır.
	};					 	// Burada datayı uint8 cinsinden 4 byte şeklinde ayıran bir struct vardır.
	struct
	{
		uint8_t bytes[4];
	};
} float_to_byte;  //struct ismi.
typedef union
{
	struct
	{
		double data;  		// Float olan bir veri 4byte'dır.
	};					 	// Burada datayı uint8 cinsinden 4 byte şeklinde ayıran bir struct vardır.
	struct
	{
		uint8_t bytes[4];
	};
} double_to_byte;  //struct ismi.

double_to_byte otolat1;
double_to_byte otolng1;


float_to_byte Pitch_intgral;
float_to_byte Roll_intgral;
float_to_byte Yaw_intgral;
float_to_byte Pitch_ControlSignal;
float_to_byte Roll_ControlSignal;
float_to_byte Yaw_ControlSignal;
float_to_byte Heading;


								//struct için fonk ismi tanımaladık.Gelen verileri kaydetmek için.
float_to_byte X_kpbytes;
float_to_byte X_kibytes;		//RollPitch PID verileri.
float_to_byte X_kdbytes;

float_to_byte Y_kpbytes;
float_to_byte Y_kibytes;		//Yaw PID verileri.
float_to_byte Y_kdbytes;

float_to_byte Z_kpbytes;		//Altitude PID verileri.
float_to_byte Z_kibytes;
float_to_byte Z_kdbytes;

float_to_byte recx_kpbytes;   //c# üzerinden RollPitch PID değerlerini içine attığımız değişken.
float_to_byte recx_kibytes;
float_to_byte recx_kdbytes;

float_to_byte recy_kpbytes;   //c# üzerinden YAW PID değerlerini içine attığımız değişken.
float_to_byte recy_kibytes;
float_to_byte recy_kdbytes;

float_to_byte recz_kpbytes;   //c# üzerinden Altitude PID değerlerini içine attığımız değişken.
float_to_byte recz_kibytes;
float_to_byte recz_kdbytes;

float_to_byte gyro_xbytes;		//Gyro_Angle verileri.
float_to_byte gyro_ybytes;
float_to_byte gyro_zbytes;

float_to_byte gyrox_errorbytes;		//Gyro_Angle verileri.
float_to_byte gyroy_errorbytes;
float_to_byte gyroz_errorbytes;

float_to_byte Barometer_Altitude;	//Barometre verileri.

float_to_byte Altitude_ControlSignal;
float_to_byte Altitude_integral;

float_to_byte DroneKontrol_Dizi;




uint8_t packet_header_s1_char = 'A';		//Header 1 = 66.
uint8_t packet_header_s2_char = 'B';		//Header 2 = 65.
uint8_t packet_header_end_char = 0;		//Header son = 'Z'.



#define SIZE_OF_RX_BUFFER		(128)		//Receive paketleri 64 byte olarak belirlendi.

#define SIZE_OF_OnePacket_BUFFER		(125)


unsigned char tx_Send_OnePacket[SIZE_OF_OnePacket_BUFFER] = {0};


int button_valueon;
int button_valuesol;
int button_valuearka;
int button_valuesag;
int i=0;

uint8_t rx_buffer[SIZE_OF_RX_BUFFER];		//Receive paketi tanımlandı.

uint16_t Motor1_Duty=1000;					//uint16 tipinde motor_PWM değişkenleri tanımlandı.
uint16_t Motor2_Duty=1000;
uint16_t Motor3_Duty=1000;
uint16_t Motor4_Duty=5000;

float Barometer_altitude = 2.45f;			//float tipinde Barometer değişkeni tanımlandı.
uint32_t GPS_Lattitude = (41.0257612)*(10000000);		//float tipinde GPS değişkenleri tanımlandı.
uint32_t GPS_Longtitude =(28.88924095)*(100000000);

float X_kp=5.4f;							//float tipinde RollPitch_PID değişkenleri tanımlandı.
float X_ki=1.2f;
float X_kd=2.4f;

float Y_kp=5.4f;							//float tipinde YAW_PID değişkenleri tanımlandı.
float Y_ki=1.2f;
float Y_kd=2.4f;

float Z_kp=5.4f;							//float tipinde Altitude_PID değişkenleri tanımlandı.
float Z_ki=1.2f;
float Z_kd=2.4f;

float gyrox ;     					//float tipinde Gyro_Angle değişkenleri tanımlandı.
float gyroy ;
float gyroz = 11.2f;

float gyrox_error = 1.0f;
float gyroy_error = 2.0f;
float gyroz_error = 25.2f;

float Pitch_integral_float=16;
float Roll_integral_float=16;
float Yaw_integral_float=15;
float Pitch_ControlSignal_float=17;
float Roll_ControlSignal_float=18;
float Yaw_ControlSignal_float=19;
float Heading_float=5;

float Altitude_ControlSignal_float = 11 ;
float Altitude_integral_float = 5 ;

int16_t KumandaOnOff=1;
int16_t KumandaFailsafe=1;

double otogpslat1;
double temp_oto;
uint8_t silbuton;
double otogpslng1;

float Dronekomut_dizi_float;

float ComplementaryFAngleRoll;
float ComplementaryFAnglePitch;
float CFilterRoll;
float CFilterPitch;
float Roll_Accelometer;
float Pitch_Accelometer;

uint8_t saat;
uint8_t saniye;

uint16_t Error_Flag = 0 ;

float Error_Flag_Buffer1 = 0 ;
float Error_Flag_Buffer2 = 1 ;
float Error_Flag_Buffer3 = 2 ;
uint8_t sayac =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

int error_flag =0 ;

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);


	}

}
void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;

	Roll_Accelometer  = atan((Ay)/sqrt(pow((Ax),2) + pow((Az),2)))*RAD_TO_DEG;	//pow fonksiyonu Ax in 2 üssünü alır burda.
	Pitch_Accelometer = atan(-1*(Ax)/sqrt(pow((Ay),2) + pow((Az),2)))*RAD_TO_DEG;


}

void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;

}

static void Parameter_Receive_PC(void)			//c# üzerinden gelen PID değerlerini alan fonksiyon.
{
	if (rx_buffer[0] == 'R')
	{
		recx_kpbytes.bytes[0]=rx_buffer[1];     // union struct yapısı ile gelen verileri 4 byte şekilde alıyoruz.
		recx_kpbytes.bytes[1]=rx_buffer[2];		// çünkü float 4 byte bu nedenle 4 byte içinde alıyoruzz.
		recx_kpbytes.bytes[2]=rx_buffer[3];
		recx_kpbytes.bytes[3]=rx_buffer[4];

		X_kp=recx_kpbytes.data;                 //Dataları birleştirip float cinsindeki kp değişkeninin içine atıyoruz.

		recx_kibytes.bytes[0]=rx_buffer[5];
		recx_kibytes.bytes[1]=rx_buffer[6];
		recx_kibytes.bytes[2]=rx_buffer[7];
		recx_kibytes.bytes[3]=rx_buffer[8];

		X_ki=recx_kibytes.data;

		recx_kdbytes.bytes[0]=rx_buffer[9];
		recx_kdbytes.bytes[1]=rx_buffer[10];
		recx_kdbytes.bytes[2]=rx_buffer[11];
		recx_kdbytes.bytes[3]=rx_buffer[12];

		X_kd=recx_kdbytes.data;
	}

	if (rx_buffer[0] == 'Y')
	{
		recy_kpbytes.bytes[0]=rx_buffer[13];     // union struct yapısı ile gelen verileri 4 byte şekilde alıyoruz.
		recy_kpbytes.bytes[1]=rx_buffer[14];		// çünkü float 4 byte bu nedenle 4 byte içinde alıyoruzz.
		recy_kpbytes.bytes[2]=rx_buffer[15];
		recy_kpbytes.bytes[3]=rx_buffer[16];

		Y_kp=recy_kpbytes.data;                 //Dataları birleştirip float cinsindeki kp değişkeninin içine atıyoruz.

		recy_kibytes.bytes[0]=rx_buffer[17];
		recy_kibytes.bytes[1]=rx_buffer[18];
		recy_kibytes.bytes[2]=rx_buffer[19];
		recy_kibytes.bytes[3]=rx_buffer[20];

		Y_ki=recy_kibytes.data;

		recy_kdbytes.bytes[0]=rx_buffer[21];
		recy_kdbytes.bytes[1]=rx_buffer[22];
		recy_kdbytes.bytes[2]=rx_buffer[23];
		recy_kdbytes.bytes[3]=rx_buffer[24];

		Y_kd=recy_kdbytes.data;
	}

	if (rx_buffer[0] == 'L')
	{
		recz_kpbytes.bytes[0]=rx_buffer[25];     // union struct yapısı ile gelen verileri 4 byte şekilde alıyoruz.
		recz_kpbytes.bytes[1]=rx_buffer[26];		// çünkü float 4 byte bu nedenle 4 byte içinde alıyoruzz.
		recz_kpbytes.bytes[2]=rx_buffer[27];
		recz_kpbytes.bytes[3]=rx_buffer[28];

		Z_kp=recz_kpbytes.data;                 //Dataları birleştirip float cinsindeki kp değişkeninin içine atıyoruz.

		recz_kibytes.bytes[0]=rx_buffer[29];
		recz_kibytes.bytes[1]=rx_buffer[30];
		recz_kibytes.bytes[2]=rx_buffer[31];
		recz_kibytes.bytes[3]=rx_buffer[32];

		Z_ki=recz_kibytes.data;

		recz_kdbytes.bytes[0]=rx_buffer[33];
		recz_kdbytes.bytes[1]=rx_buffer[34];
		recz_kdbytes.bytes[2]=rx_buffer[35];
		recz_kdbytes.bytes[3]=rx_buffer[36];

		Z_kd=recz_kdbytes.data;
	}
	if (rx_buffer[0] == 'F')
	{

		 otolat1.bytes[0] = rx_buffer[37];
		 otolat1.bytes[1] = rx_buffer[38];
		 otolat1.bytes[2] = rx_buffer[39];
		 otolat1.bytes[3] = rx_buffer[40];
		 otolat1.bytes[4] = rx_buffer[41];
		 otolat1.bytes[5] = rx_buffer[42];
		 otolat1.bytes[6] = rx_buffer[43];
		 otolat1.bytes[7] = rx_buffer[44];


		 otogpslat1=otolat1.data;

		 otolng1.bytes[0] = rx_buffer[45];
		 otolng1.bytes[1] = rx_buffer[46];
		 otolng1.bytes[2] = rx_buffer[47];
		 otolng1.bytes[3] = rx_buffer[48];
		 otolng1.bytes[4] = rx_buffer[49];
		 otolng1.bytes[5] = rx_buffer[50];
		 otolng1.bytes[6] = rx_buffer[51];
		 otolng1.bytes[7] = rx_buffer[52];

		 otogpslng1=otolng1.data;

	}
	if (rx_buffer[0] == 'D')
		{

			 DroneKontrol_Dizi.bytes[0] = rx_buffer[53];
			 DroneKontrol_Dizi.bytes[1] = rx_buffer[54];
			 DroneKontrol_Dizi.bytes[2] = rx_buffer[55];
			 DroneKontrol_Dizi.bytes[3] = rx_buffer[56];



			 Dronekomut_dizi_float=DroneKontrol_Dizi.data;

		}



}

static void Parameter_Transmit_PC(void)				//Parametleri gönderdiğimiz fonksiyon.
{


	All_Parameters_Send_Tekseferde(); //tek paketle yollamak için.



	CDC_Transmit_FS(tx_Send_OnePacket, SIZE_OF_OnePacket_BUFFER);
	HAL_Delay(200);
}
static void All_Parameters_Send_Tekseferde(void)
{


		X_kpbytes.data=X_kp;
		X_kibytes.data=X_ki;
		X_kdbytes.data=X_kd;

		Y_kpbytes.data=Y_kp;
		Y_kibytes.data=Y_ki;
		Y_kdbytes.data=Y_kd;

		Z_kpbytes.data = Z_kp;
		Z_kibytes.data=Z_ki;
		Z_kdbytes.data=Z_kd;


		gyro_xbytes.data=gyrox;     // ana karttan gelen gyrox verilerini bytlerına ayırıyoruz.
		gyro_ybytes.data=gyroy;
		gyro_zbytes.data=gyroz;

		gyrox_errorbytes.data=gyrox_error;
		gyroy_errorbytes.data=gyroy_error;
		gyroz_errorbytes.data=gyroz_error;

		Barometer_Altitude.data=Barometer_altitude;     // ana karttan gelen Barometre verilerini bytlerına ayırıyoruz.


		Pitch_intgral.data=Pitch_integral_float;
		Roll_intgral.data=Roll_integral_float;
		Yaw_intgral.data=Yaw_integral_float;

		Pitch_ControlSignal.data=Pitch_ControlSignal_float;
		Roll_ControlSignal.data=Roll_ControlSignal_float;
		Yaw_ControlSignal.data=Yaw_ControlSignal_float;

		Altitude_ControlSignal.data = Altitude_ControlSignal_float;
		Altitude_integral.data = Altitude_integral_float;

		Heading.data=Heading_float;

		tx_Send_OnePacket[0]=packet_header_s1_char;  //65
		tx_Send_OnePacket[1]=packet_header_s2_char; // 66 olmalı !!




		tx_Send_OnePacket[2] =gyro_xbytes.bytes[0];  		// burada da 4 byte şeklinde c# a gönderiyoruz.
		tx_Send_OnePacket[3] =gyro_xbytes.bytes[1];
		tx_Send_OnePacket[4] =gyro_xbytes.bytes[2];
		tx_Send_OnePacket[5] =gyro_xbytes.bytes[3];

		tx_Send_OnePacket[6]  =gyro_ybytes.bytes[0];
		tx_Send_OnePacket[7]  =gyro_ybytes.bytes[1];
		tx_Send_OnePacket[8]  =gyro_ybytes.bytes[2];
		tx_Send_OnePacket[9] =gyro_ybytes.bytes[3];

		tx_Send_OnePacket[10] =gyro_zbytes.bytes[0];
		tx_Send_OnePacket[11] =gyro_zbytes.bytes[1];
		tx_Send_OnePacket[12] =gyro_zbytes.bytes[2];
		tx_Send_OnePacket[13] =gyro_zbytes.bytes[3];

		tx_Send_OnePacket[14] =X_kpbytes.bytes[0];		//Xpıd
		tx_Send_OnePacket[15] =X_kpbytes.bytes[1];
		tx_Send_OnePacket[16] =X_kpbytes.bytes[2];
		tx_Send_OnePacket[17] =X_kpbytes.bytes[3];

		tx_Send_OnePacket[18] =X_kibytes.bytes[0];
		tx_Send_OnePacket[19] =X_kibytes.bytes[1];
		tx_Send_OnePacket[20] =X_kibytes.bytes[2];
		tx_Send_OnePacket[21] =X_kibytes.bytes[3];

		tx_Send_OnePacket[22] =X_kdbytes.bytes[0];
		tx_Send_OnePacket[23] =X_kdbytes.bytes[1];
		tx_Send_OnePacket[24] =X_kdbytes.bytes[2];
		tx_Send_OnePacket[25] =X_kdbytes.bytes[3];

		tx_Send_OnePacket[26] =Y_kpbytes.bytes[0];   //ypıd
		tx_Send_OnePacket[27] =Y_kpbytes.bytes[1];
		tx_Send_OnePacket[28] =Y_kpbytes.bytes[2];
		tx_Send_OnePacket[29] =Y_kpbytes.bytes[3];

		tx_Send_OnePacket[30]  =Y_kibytes.bytes[0];
		tx_Send_OnePacket[31]  =Y_kibytes.bytes[1];
		tx_Send_OnePacket[32]  =Y_kibytes.bytes[2];
		tx_Send_OnePacket[33] =Y_kibytes.bytes[3];

		tx_Send_OnePacket[34] =Y_kdbytes.bytes[0];
		tx_Send_OnePacket[35] =Y_kdbytes.bytes[1];
		tx_Send_OnePacket[36] =Y_kdbytes.bytes[2];
		tx_Send_OnePacket[37] =Y_kdbytes.bytes[3];

		tx_Send_OnePacket[38] = Z_kpbytes.bytes[0];			//zpıd
		tx_Send_OnePacket[39] = Z_kpbytes.bytes[1];
		tx_Send_OnePacket[40] = Z_kpbytes.bytes[2];
		tx_Send_OnePacket[41] = Z_kpbytes.bytes[3];

		tx_Send_OnePacket[42] =Z_kibytes.bytes[0];
		tx_Send_OnePacket[43] =Z_kibytes.bytes[1];
		tx_Send_OnePacket[44] =Z_kibytes.bytes[2];
		tx_Send_OnePacket[45] =Z_kibytes.bytes[3];

		tx_Send_OnePacket[46] =Z_kdbytes.bytes[0];
		tx_Send_OnePacket[47] =Z_kdbytes.bytes[1];
		tx_Send_OnePacket[48] =Z_kdbytes.bytes[2];
		tx_Send_OnePacket[49] =Z_kdbytes.bytes[3];

		tx_Send_OnePacket[50] =(uint8_t)Motor1_Duty;
		tx_Send_OnePacket[51] =(uint8_t)(Motor1_Duty >> 8) ;
		tx_Send_OnePacket[52] =(uint8_t)Motor2_Duty;
		tx_Send_OnePacket[53] =(uint8_t)(Motor2_Duty >> 8) ;

		tx_Send_OnePacket[54] =(uint8_t)Motor3_Duty;
		tx_Send_OnePacket[55] =(uint8_t)(Motor3_Duty >> 8) ;
		tx_Send_OnePacket[56] =(uint8_t)Motor4_Duty;
		tx_Send_OnePacket[57]=(uint8_t)(Motor4_Duty >> 8);

		tx_Send_OnePacket[58] =Barometer_Altitude.bytes[0];
		tx_Send_OnePacket[59] =Barometer_Altitude.bytes[1];
		tx_Send_OnePacket[60] =Barometer_Altitude.bytes[2];
		tx_Send_OnePacket[61] =Barometer_Altitude.bytes[3];


		tx_Send_OnePacket[62] =(uint8_t)GPS_Lattitude;
		tx_Send_OnePacket[63] =(uint8_t)(GPS_Lattitude >> 8);
		tx_Send_OnePacket[64] =(uint8_t)(GPS_Lattitude >> 16);
		tx_Send_OnePacket[65] =(uint8_t)(GPS_Lattitude >> 24);


		tx_Send_OnePacket[66] =(uint8_t)GPS_Longtitude;
		tx_Send_OnePacket[67] =(uint8_t)(GPS_Longtitude >> 8);
		tx_Send_OnePacket[68] =(uint8_t)(GPS_Longtitude >> 16);
		tx_Send_OnePacket[69] =(uint8_t)(GPS_Longtitude >> 24);


		tx_Send_OnePacket[70] =gyrox_errorbytes.bytes[0];  		// burada da 4 byte şeklinde c# a gönderiyoruz.
		tx_Send_OnePacket[71] =gyrox_errorbytes.bytes[1];
		tx_Send_OnePacket[72] =gyrox_errorbytes.bytes[2];
		tx_Send_OnePacket[73] =gyrox_errorbytes.bytes[3];

		tx_Send_OnePacket[74]  =gyroy_errorbytes.bytes[0];
		tx_Send_OnePacket[75]  =gyroy_errorbytes.bytes[1];
		tx_Send_OnePacket[76]  =gyroy_errorbytes.bytes[2];
		tx_Send_OnePacket[77]  =gyroy_errorbytes.bytes[3];

		tx_Send_OnePacket[78] =gyroz_errorbytes.bytes[0];
		tx_Send_OnePacket[79] =gyroz_errorbytes.bytes[1];
		tx_Send_OnePacket[80] =gyroz_errorbytes.bytes[2];
		tx_Send_OnePacket[81] =gyroz_errorbytes.bytes[3];



		tx_Send_OnePacket[82] =Pitch_intgral.bytes[0];
		tx_Send_OnePacket[83] =Pitch_intgral.bytes[1];
		tx_Send_OnePacket[84] =Pitch_intgral.bytes[2];
		tx_Send_OnePacket[85] =Pitch_intgral.bytes[3];

		tx_Send_OnePacket[86] =Roll_intgral.bytes[0];
		tx_Send_OnePacket[87] =Roll_intgral.bytes[1];
		tx_Send_OnePacket[88] =Roll_intgral.bytes[2];
		tx_Send_OnePacket[89] =Roll_intgral.bytes[3];

		tx_Send_OnePacket[90] =Yaw_intgral.bytes[0];
		tx_Send_OnePacket[91] =Yaw_intgral.bytes[1];
		tx_Send_OnePacket[92] =Yaw_intgral.bytes[2];
		tx_Send_OnePacket[93] =Yaw_intgral.bytes[3];



		tx_Send_OnePacket[94] =Pitch_ControlSignal.bytes[0];
		tx_Send_OnePacket[95] =Pitch_ControlSignal.bytes[1];
		tx_Send_OnePacket[96] =Pitch_ControlSignal.bytes[2];
		tx_Send_OnePacket[97] =Pitch_ControlSignal.bytes[3];


		tx_Send_OnePacket[98] =Roll_ControlSignal.bytes[0];
		tx_Send_OnePacket[99] =Roll_ControlSignal.bytes[1];
		tx_Send_OnePacket[100] =Roll_ControlSignal.bytes[2];
		tx_Send_OnePacket[101] =Roll_ControlSignal.bytes[3];

		tx_Send_OnePacket[102] =Yaw_ControlSignal.bytes[0];
		tx_Send_OnePacket[103] =Yaw_ControlSignal.bytes[1];
		tx_Send_OnePacket[104] =Yaw_ControlSignal.bytes[2];
		tx_Send_OnePacket[105] =Yaw_ControlSignal.bytes[3];


		tx_Send_OnePacket[106] =Heading.bytes[0];
		tx_Send_OnePacket[107] =Heading.bytes[1];
		tx_Send_OnePacket[108] =Heading.bytes[2];
		tx_Send_OnePacket[109] =Heading.bytes[3];

		tx_Send_OnePacket[110] =(uint8_t)KumandaOnOff;
		tx_Send_OnePacket[111] =(uint8_t)(KumandaOnOff >> 8 );

		tx_Send_OnePacket[112] =(uint8_t)KumandaFailsafe;
		tx_Send_OnePacket[113] =(uint8_t)(KumandaFailsafe >> 8 );

		tx_Send_OnePacket[114] =Altitude_ControlSignal.bytes[0];
		tx_Send_OnePacket[115] =Altitude_ControlSignal.bytes[1];
		tx_Send_OnePacket[116] =Altitude_ControlSignal.bytes[2];
		tx_Send_OnePacket[117] =Altitude_ControlSignal.bytes[3];

		tx_Send_OnePacket[118] =Altitude_integral.bytes[0];
		tx_Send_OnePacket[119] =Altitude_integral.bytes[1];
		tx_Send_OnePacket[120] =Altitude_integral.bytes[2];
		tx_Send_OnePacket[121] =Altitude_integral.bytes[3];

		tx_Send_OnePacket[122] = error_flag;
		tx_Send_OnePacket[123] = error_flag >> 8 ;

		tx_Send_OnePacket[124]= packet_header_end_char;


}
static void Error_Flag_Function(void)
{


			if(sayac == 0)
			{
				Error_Flag_Buffer1 = gyrox;

			}
			if(sayac == 1)
			{
				Error_Flag_Buffer2 = gyrox;

			}
			sayac++;
			if(sayac == 3)
			{
				Error_Flag_Buffer3 = gyrox;
				sayac = 0 ;
			}
			if (Error_Flag_Buffer3 == Error_Flag_Buffer1)
			{
				Error_Flag = 1 ;
				error_flag = 1 ;
			}
			if (Error_Flag_Buffer3 != Error_Flag_Buffer1 )
			{
				error_flag = 0;
			}

}
static void Button_Kontrol_Function(void)
{
	button_valueon= HAL_GPIO_ReadPin(button_GPIO_Port, button_Pin);
		  	button_valuesag=HAL_GPIO_ReadPin(button_sag_GPIO_Port,button_sag_Pin);
		  	button_valuesol=HAL_GPIO_ReadPin(button_sol_GPIO_Port,button_sol_Pin);
		  	button_valuearka=HAL_GPIO_ReadPin(button_geri_GPIO_Port,button_geri_Pin);
		  	if (button_valueon == 1)
		  	{
		  		GPS_Lattitude += + 1000;
		  		HAL_Delay(100);
		  	}

		  	if (button_valuesag == 1)
				{
					GPS_Longtitude += + 1000;
					HAL_Delay(100);
				}

		  	if (button_valuesol == 1)
				{
					GPS_Longtitude += - 1000;
					HAL_Delay(100);
				}
		  	if (button_valuearka == 1)
				{
		  			GPS_Lattitude += - 1000;
					HAL_Delay(100);
				}

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  	HAL_TIM_Base_Start_IT(&htim2);


   //MPU6050_Init();
   //HAL_Delay (1000);  // wait for 1 sec


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	Error_Flag_Function();

	  	if(Error_Flag)
	  	{
	  		MX_I2C1_Init();
	  		Error_Flag = 0;
	  	}



	  	MPU6050_Init();
	  	MPU6050_Read_Accel();
	  	MPU6050_Read_Gyro();

		gyrox=Roll_Accelometer;
		gyroy=Pitch_Accelometer;


		packet_header_end_char = ((packet_header_s1_char * packet_header_s2_char) + (KumandaOnOff * KumandaFailsafe) + (21+07+97)  + (Motor1_Duty + Motor3_Duty)) / 100 ;




		Button_Kontrol_Function();
	  	Parameter_Receive_PC();				//PID değerlerini c# üzerinden alan fonksiyon.
	  	Parameter_Transmit_PC();			//Parametreleri Transmit eden fonksiyon.




  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 13;
  sTime.Minutes = 17;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  DateToUpdate.Month = RTC_MONTH_JULY;
  DateToUpdate.Date = 28;
  DateToUpdate.Year = 21;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led1_Pin|led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led1_Pin led2_Pin */
  GPIO_InitStruct.Pin = led1_Pin|led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : button_geri_Pin */
  GPIO_InitStruct.Pin = button_geri_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(button_geri_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : button_Pin button_sol_Pin button_sag_Pin */
  GPIO_InitStruct.Pin = button_Pin|button_sol_Pin|button_sag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
