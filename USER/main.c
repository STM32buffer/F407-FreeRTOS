
#include "FreeRTOS.h"
#include "task.h"

#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "mpu6050.h"
//#include "usmart.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "data_transfer.h"
#include "delay.h"
#include "timer.h"
#include "motors.h"
#include "schedule.h"





int a=150,b=150,c=150;
extern int16_t HIGH,highset;//????
extern int32_t highTag, highNow, control;//????????,????,???
int time2 = 0;
int time3 = 0;
int time4 = 0;
int time5 = 0;
int time6 = 0;
int time7 = 0;

int agl_count = 0;
float agl_sum0 = 0;
int encoderCount=0;
int te1 =0;
int test1=150, test2=150, test3=0;
int pre_count = 0;
int PITCH_me,YAW_me,ROLL_me;
extern  uint8_t BTNumberRece[16];
/************************************************

************************************************/

//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define USTB_SENSOR_TASK_PRIO		2
//任务堆栈大小	
#define USTB_SENSOR_STK_SIZE 		128  
//任务句柄
TaskHandle_t USTB_SensorTask_Handler;
//任务函数
void USTB_sensor_task(void *pvParameters);

//任务优先级
#define USTB_DateExchang_TASK_PRIO		3
//任务堆栈大小	
#define USTB_DateExchang_STK_SIZE 		128  
//任务句柄
TaskHandle_t USTB_DateExchangTask_Handler;
//任务函数
void USTB_DateExchang_task(void *pvParameters);

//任务优先级
#define USTB_AttitudeControl_TASK_PRIO		4
//任务堆栈大小	
#define USTB_AttitudeControl_STK_SIZE 		128  
//任务句柄
TaskHandle_t USTB_AttitudeControlTask_Handler;
//任务函数
void USTB_AttitudeControl_task(void *pvParameters);



int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);					//初始化延时函数
	uart_init(115200);     				//初始化串口
  whole_encoder =0;
	TIM_SetCompare4(TIM4, 1800); // PB9  ??
	TIM_SetCompare1(TIM2, 1890); // PB7	 2?????
	TIM_SetCompare2(TIM2, 1800); // PB7  1?????
	TIM_SetCompare2(TIM4, 1800); // ???
	MPU_Init(); 
	PID_ControlInit();
	delay_ms(3000); 
	TIM1_Int_Init(19,7199);
	while(mpu_dmp_init())//???????????
   {
			delay_ms(200);
   }
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	//创建KEY任务
	xTaskCreate((TaskFunction_t )USTB_sensor_task,             
                (const char*    )"USTB_sensor_task",           
                (uint16_t       )USTB_SENSOR_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )USTB_SENSOR_TASK_PRIO,        
                (TaskHandle_t*  )&USTB_SensorTask_Handler);  
    //创建USTB_DateExchang任务
    xTaskCreate((TaskFunction_t )USTB_DateExchang_task,             
                (const char*    )"USTB_DateExchang_task",           
                (uint16_t       )USTB_DateExchang_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )USTB_DateExchang_TASK_PRIO,        
                (TaskHandle_t*  )&USTB_DateExchangTask_Handler);   
    //创建USTB_AttitudeControl任务
    xTaskCreate((TaskFunction_t )USTB_AttitudeControl_task,     
                (const char*    )"USTB_AttitudeControl_task",   
                (uint16_t       )USTB_AttitudeControl_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )USTB_AttitudeControl_TASK_PRIO,
                (TaskHandle_t*  )&USTB_AttitudeControlTask_Handler); 
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//传感器任务函数
void USTB_sensor_task(void *pvParameters)
{
	while(1)
	{
		if(mpu_dmp_get_data(&roll,&pitch,&yaw)==0)		//??3ms
			{
					MPU_Get_Accelerometer(&aacx,&aacy,&aacz);   //??????????	//???1ms
					MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //???????		//???1m
			}
			vTaskDelay(10);  
	}
}

//数据交互任务函数
void USTB_DateExchang_task(void *pvParameters)
{
	while(1)
	{
		USTB_DT_Data_Exchange();
     vTaskDelay(100);                           //延时1s，也就是1000个时钟节拍	
	}
}

long int time1=0;
//姿态控制任务函数
void USTB_AttitudeControl_task(void *pvParameters)
{
	while(1)
	{
		User_PidSpeedControl(); //油门
		Att_Control();    			//姿态控制    
		time1++;
		if(time1>60000000)
			time1 = 0;
		if(time1%5 == 0) //10ms发一次
			Att_Control_2();
    vTaskDelay(3);                           //延时1s，也就是1000个时钟节拍	
	}
}

