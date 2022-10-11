#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "sys.h"
#include "usart.h"	  

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef struct 
{
		uint8_t send_check;
		uint8_t send_version;
		uint8_t send_status;
		uint8_t send_senser;
		uint8_t send_senser2;
		uint8_t send_pid1;
		uint8_t send_pid2;
		uint8_t send_pid3;
		uint8_t send_pid4;
		uint8_t send_pid5;
		uint8_t send_pid6;
		uint8_t send_rcdata;
		uint8_t send_offset;
		uint8_t send_motopwm;
		uint8_t send_power;
		uint8_t send_user;
		uint8_t send_speed;
		uint8_t send_location;

} dt_flag_t;
typedef struct 
{ 
	float Acc_x;//加速度
	float Acc_y;
	float Acc_z;
	float Gyro_x;//陀螺仪
	float Gyro_y;
	float Gyro_z;
	float Mag_x; //电子罗盘
	float Mag_y;
	float Mag_z;
  float Roll;  //欧拉角
	float Pitch;
	float Yaw;
	uint16_t volt;//电池电压
	uint16_t PWM; //占空比驱动电机
	uint16_t P;//控制PID
	uint16_t I;
	uint16_t D;
} dt_struct_t;
typedef struct
{
	float Kp;
	float Ki;
	float Kd;

}PID_struct_t;
extern dt_flag_t f;
extern dt_struct_t info;

void USTB_DT_Data_Exchange(void);
void USTB_DT_Data_Receive_Prepare(u8 data);
void USTB_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void USTB_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void USTB_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void USTB_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void USTB_DT_Send_Senser2(s32 bar_alt,u16 csb_alt);
void USTB_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void USTB_DT_Send_Power(u16 votage, u16 current);
void USTB_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void USTB_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void USTB_DT_Send_User(void);
void USTB_DT_Send_Speed(float,float,float);
void USTB_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle);

#endif

