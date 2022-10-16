#include "schedule.h"
#include <math.h>


float pitch,roll,yaw;       //欧拉角--MPU6050解算
short aacx,aacy,aacz;       //加速度传感器原始数据
short gyrox,gyroy,gyroz;    //陀螺仪角速度原始数据
short temp;                 //温度
int whole_encoder=0;
//注意两种开发板接线不一致，这里需要调整
//这是旧版
//#define PERIOD uptimech4
//#define OFFSET uptimech3
//#define TURNLR uptimech2
//这是新版
#define PERIOD uptimech2
#define TAILUD uptimech4
#define OFFSET uptimech3
#define TURNLR uptimech1
#define min(a,b) ((a<b)?a:b)
#define max(a,b) ((a>b)?a:b)


//void TIM2_Int_Init(u16 arr, u16 psc)
//{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能

//    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
//    TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

//    TIM_ITConfig(TIM2, TIM_IT_Update,	ENABLE); // 允许TIM3更新中断

//    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

//		TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设
//}
void TIM1_Int_Init(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能

  TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
  TIM_TimeBaseStructure.TIM_Prescaler =(psc-1);//设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//??????
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//??????

  TIM_ITConfig(      //?????????TIM??
    TIM1,            //TIM1
    TIM_IT_Update  | //TIM ?????
    TIM_IT_Trigger,  //TIM ????? 
    ENABLE  	     //??
    );
	
  //?????
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//?????0?
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  	   //????0?
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  TIM_Cmd(TIM1, ENABLE);  //使能TIMx外设
	
	
	
	
}



extern int16_t Speed_set; // 控制电机摇杆输入量对应的值
extern int16_t D_yaw_control,D_pitch_control,D_roll_control;  // 控制两个舵机摇杆输入量对应的值0~100
//int32_t d_yaw_control,d_pitch_control;        // 将D_yaw_control从0~100转换成期望的角度范围
int dir_pwm_yaw=150,dir_pwm_pitch = 150,dir_pwm_roll = 150;    // 控制两个舵机P波值，150对应中值



int32_t speed_set=0, speed_control=200;//电机控制量
void User_PidSpeedControl(void)  //控制电机来控制飞行高度
{
    speed_set = Speed_set*1.10;
		speed_control=200 - speed_set;
		if(speed_control>200)
			speed_control=200;
		if(speed_control<90)
			speed_control=90;
		TIM_SetCompare4(TIM4, speed_control);
}



u16 yaw_Mid = 150;         //控制yaw舵机的中值
u16 pitch_Mid = 150;       //控制pitch舵机的中值
u16 roll_Mid = 150;  
//int yaw_up_limit = 45;     //样机期望角度的范围
//int yaw_down_limit = -45;  //样机期望角度的范围
//int pitch_up_limit = 45;   //样机期望角度的范围
//int pitch_down_limit = -45;//样机期望角度的范围
PID_AbsoluteType yaw_PID;  				//角度环PID控制器
PID_AbsoluteType pitch_PID;
PID_AbsoluteType roll_PID;

PID_AbsoluteType yaw_speed_PID;		//角速度环PID控制器
PID_AbsoluteType pitch_speed_PID;
PID_AbsoluteType roll_speed_PID;

PID_AbsoluteType yaw_acc_PID;			//角加速度环PID控制器
PID_AbsoluteType pitch_acc_PID;
PID_AbsoluteType roll_acc_PID;

int yaw_speed_limit = 20000;		 //角速度限幅-21846~21846
int pitch_speed_limit = 20000;
int roll_speed_limit = 20000;

int yaw_acc_limit = 20000;		 //角加速度限幅-21846~21846
int pitch_acc_limit = 20000;
int roll_acc_limit = 20000;

int yaw_out_limit = 20;		 //舵机输出限幅，直接作用于舵机P波
int pitch_out_limit = 5;
int roll_out_limit = 5;

int target_yaw = 90; 			 //样机期望角度
int target_pitch = 0;
int target_roll = 0;

void Att_Control() //加自稳的姿态控制
{
	//yaw姿态角的三个串级PID
	/************************************换样机时注意**************************************************/
	target_yaw = (D_yaw_control-50)*90/100;   //根据遥控器传过来的D_yaw_control计算期望的角度，限幅为-45~45°竖直的时候pitch是90左右
	yaw_PID.errNow = target_yaw - yaw;  //样机的yaw对应IMU的roll,
	/**************************************************************************************************/
	PID_AbsoluteMode(&yaw_PID);
	yaw_PID.ctrOut = yaw_PID.ctrOut>yaw_speed_limit? yaw_speed_limit:yaw_PID.ctrOut;
	yaw_PID.ctrOut = yaw_PID.ctrOut<-yaw_speed_limit? -yaw_speed_limit:yaw_PID.ctrOut;
		
	
		//roll姿态角的三个串级PID
	/************************************换样机时注意**************************************************/
	target_roll = (D_roll_control-50)*90/100;   //根据遥控器传过来的D_roll_control计算期望的角度，限幅为-45~45°竖直的时候pitch是90左右
	roll_PID.errNow = target_roll - roll;  
	/**************************************************************************************************/
	PID_AbsoluteMode(&roll_PID);
	roll_PID.ctrOut = roll_PID.ctrOut>roll_speed_limit? roll_speed_limit:roll_PID.ctrOut;
	roll_PID.ctrOut = roll_PID.ctrOut<-roll_speed_limit? -roll_speed_limit:roll_PID.ctrOut;
	
	
	//pitch姿态角的三个串级PID
	/************************************换样机时注意**************************************************/
	target_pitch	= (D_pitch_control-50)*90/100+100;
	pitch_PID.errNow = target_pitch - pitch;//样机的pitch对应IMU的pitch,竖直的时候roll是0左右
	/**************************************************************************************************/
	if(pitch_PID.errNow<5 && pitch_PID.errNow>-5)
		pitch_PID.kp = 30;
	else{
		pitch_PID.kp =80;
	}
	PID_AbsoluteMode(&pitch_PID);
	pitch_PID.ctrOut = pitch_PID.ctrOut>pitch_speed_limit? pitch_speed_limit:pitch_PID.ctrOut;
	pitch_PID.ctrOut = pitch_PID.ctrOut<-pitch_speed_limit? -pitch_speed_limit:pitch_PID.ctrOut;
	
}
void Att_Control_2() //加自稳的姿态控制
{
	yaw_speed_PID.errNow = yaw_PID.ctrOut - gyroz; //gyrox,y,z？？？？？？
	PID_AbsoluteMode(&yaw_speed_PID);
	yaw_speed_PID.ctrOut = yaw_speed_PID.ctrOut>yaw_out_limit? yaw_out_limit:yaw_speed_PID.ctrOut;
	yaw_speed_PID.ctrOut = yaw_speed_PID.ctrOut<-yaw_out_limit? -yaw_out_limit:yaw_speed_PID.ctrOut;
	dir_pwm_yaw = yaw_Mid + yaw_speed_PID.ctrOut;

	
	pitch_speed_PID.errNow = pitch_PID.ctrOut - gyrox; //gyrox,y,z？？？？？？
	PID_AbsoluteMode(&pitch_speed_PID);
	pitch_speed_PID.ctrOut = pitch_speed_PID.ctrOut>pitch_out_limit? pitch_out_limit:pitch_speed_PID.ctrOut;
	pitch_speed_PID.ctrOut = pitch_speed_PID.ctrOut<-pitch_out_limit? -pitch_out_limit:pitch_speed_PID.ctrOut;
	dir_pwm_pitch = pitch_Mid - pitch_speed_PID.ctrOut;
	if( dir_pwm_pitch>146 && dir_pwm_pitch<154)
		dir_pwm_pitch = 150;
	
	roll_speed_PID.errNow = roll_PID.ctrOut - gyrox; //gyrox,y,z？？？？？？
	PID_AbsoluteMode(&roll_speed_PID);
	roll_speed_PID.ctrOut = roll_speed_PID.ctrOut>roll_out_limit? roll_out_limit:roll_speed_PID.ctrOut;
	roll_speed_PID.ctrOut = roll_speed_PID.ctrOut<-roll_out_limit? -roll_out_limit:roll_speed_PID.ctrOut;
	dir_pwm_roll = roll_Mid - roll_speed_PID.ctrOut;
	if( dir_pwm_roll>146 && dir_pwm_roll<154)
		dir_pwm_roll = 150;
	

	TIM_SetCompare2(TIM4, dir_pwm_yaw);  //小尾翼
	TIM_SetCompare3(TIM4, dir_pwm_pitch); //大尾翼  D_pitch_control*100/50*1.7
	
}



//long int time1=0;
//int send_flag = 0;

void TIM1_UP_IRQHandler(void) //2ms中断
{ 	    	  	     
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//?????TIM??????:TIM ??? 
	{
//		User_PidSpeedControl(); //油门
//		Att_Control();    			//姿态控制    
//		time1++;
//		if(time1>60000000)
//			time1 = 0;
//		if(time1%5 == 0) //10ms发一次
//			//send_flag = 1;
//			Att_Control_2();
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}	 	
} 




//PID参数初始化
void PID_ControlInit(void)
{
    //yaw
		yaw_PID.ctrOut = 0;
    yaw_PID.errNow = 0;
    yaw_PID.errOld = 0;
    yaw_PID.kd = 50;
    yaw_PID.kp = 100;
    yaw_PID.ki = 0;
	
		yaw_speed_PID.ctrOut = 0;
    yaw_speed_PID.errNow = 0;
    yaw_speed_PID.errOld = 0;
    yaw_speed_PID.kd = 0.02;
    yaw_speed_PID.kp = 0.05;
    yaw_speed_PID.ki = 0;
	
		yaw_acc_PID.ctrOut = 0;
    yaw_acc_PID.errNow = 0;
    yaw_acc_PID.errOld = 0;
    yaw_acc_PID.kd = 0.3;
    yaw_acc_PID.kp = 1;
    yaw_acc_PID.ki = 0;
		
		//pitch
		pitch_PID.ctrOut = 0;
    pitch_PID.errNow = 0;
    pitch_PID.errOld = 0;
    pitch_PID.kd = 30;
    pitch_PID.kp = 80;
    pitch_PID.ki = 0;
		
		pitch_speed_PID.ctrOut = 0;
    pitch_speed_PID.errNow = 0;
    pitch_speed_PID.errOld = 0;
    pitch_speed_PID.kd = 0.005;
    pitch_speed_PID.kp = 0.01;
    pitch_speed_PID.ki = 0;
		
		pitch_acc_PID.ctrOut = 0;
    pitch_acc_PID.errNow = 0;
    pitch_acc_PID.errOld = 0;
    pitch_acc_PID.kd = 0.3;
    pitch_acc_PID.kp = 1;
    pitch_acc_PID.ki = 0;
		
		roll_PID.ctrOut = 0;
    roll_PID.errNow = 0;
    roll_PID.errOld = 0;
    roll_PID.kd = 30;
    roll_PID.kp = 80;
    roll_PID.ki = 0;
		
		roll_speed_PID.ctrOut = 0;
    roll_speed_PID.errNow = 0;
    roll_speed_PID.errOld = 0;
    roll_speed_PID.kd = 0.005;
    roll_speed_PID.kp = 0.01;
    roll_speed_PID.ki = 0;
		
		roll_acc_PID.ctrOut = 0;
    roll_acc_PID.errNow = 0;
    roll_acc_PID.errOld = 0;
    roll_acc_PID.kd = 0.3;
    roll_acc_PID.kp = 1;
    roll_acc_PID.ki = 0;
}



//位置式PID算法
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{
     if(PID->kp      < 0)    PID->kp      = -PID->kp;
     if(PID->ki      < 0)    PID->ki      = -PID->ki;
     if(PID->kd      < 0)    PID->kd      = -PID->kd;
     if(PID->errILim < 0)    PID->errILim = -PID->errILim;

     PID->errP = PID->errNow;  //读取现在的误差，用于kp控制

     PID->errI += PID->errNow; //误差积分，用于ki控制

     PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

     PID->errOld = PID->errNow;	//保存现在的误差
     
     PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

}


/*******************************************************************************************************/

#define errDeath 10
//增量式PID算法
void PID_IncrementMode(PID_IncrementType* PID)
{
    float dErrP, dErrI, dErrD;

    if(PID->kp < 0)    PID->kp = -PID->kp;
    if(PID->ki < 0)	PID->ki = -PID->ki;
    if(PID->kd < 0)    PID->kd = -PID->kd;

    dErrP = PID->errNow - PID->errOld1;

    dErrI = PID->errNow;

    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

    PID->errOld2 = PID->errOld1; //二阶误差微分
    PID->errOld1 = PID->errNow;  //一阶误差微分

    /*增量式PID计算*/
    //PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
    if((PID->errNow > 0 && PID->errNow < errDeath) || (PID->errNow > -errDeath && PID->errNow < 0))
    {
     ;//do nothing
    }
    else
    {
     if(PID->kp == 0 && PID->ki == 0 && PID->kd == 0)   
         PID->ctrOut = 0;

     else 
         PID->ctrOut += PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
    }

//    if(PID->ctrOut > 90000)
//     PID->ctrOut = 90000;
//    else if(PID->ctrOut <= -90000)
//     PID->ctrOut = -90000;

    PID->ctrOut = PID->ctrOut/100;
}



//void TIM3_IRQHandler(void)   //TIM3中断
//{

//    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
//    {
//				time1++;
//        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源

//    }
//}
