#include "sys.h"
#include "data_transfer.h"
//#include "DRV8874.h"
//#include "adc.h"

extern u16 L_PWM;// = 150; // 左翼
extern int R_PWM;// = 150; // 右翼
extern u16 T_PWM;// = 150; // 左翼

extern u32 uptimech1;
extern u32 uptimech2;
extern u32 uptimech3;
extern u32 uptimech4;

extern long int  time1;
extern int  send_flag;
extern float pitch,roll,yaw;       //欧拉角
extern short aacx,aacy,aacz;       //加速度传感器原始数据
extern short gyrox,gyroy,gyroz;    //陀螺仪原始数据
extern short temp;                 //温度

extern float agl0;
extern int agl_init;
extern float agl_initBuf[5];
extern int te1;

/*绝对式PID算法，接口参数结构类型*/
typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 float errILim;//误差积分上限
 
 float errNow;//当前的误差
 float ctrOut;//控制量输出
 
 /*PID算法内部变量，其值不能修改*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;


/*增量式PID算法，接口参数结构类型*/
typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 int32_t kp;     //比例系数
 int32_t ki;     //积分系数
 int32_t kd;     //微分系数
 
 int32_t errNow; //当前的误差
 int32_t dCtrOut;//控制增量输出
 int32_t  ctrOut;//控制输出
 
 /*PID算法内部变量，其值不能修改*/
 int32_t errOld1;
 int32_t errOld2;
 
}PID_IncrementType;

extern PID_AbsoluteType PID_Control;
extern PID_IncrementType PID_Control_1;
extern PID_AbsoluteType dir_pd;
extern int whole_encoder;
extern int32_t d_yaw_control,d_pitch_control,d_roll_control ;
extern int32_t dir_pwm_pitch, dir_pwm_yaw;
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM1_Int_Init(u16 arr,u16 psc);
void PID_ControlInit(void);
void TIM1_UP_IRQHandler(void);
void PID_AbsoluteMode(PID_AbsoluteType* PID);
void PID_IncrementMode(PID_IncrementType* PID);
void PID(void);
void User_PidSpeedControl(void) ;
void Att_Control(void) ;
void Att_Control_2(void) ;
