
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

//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define USTB_SENSOR_TASK_PRIO		2
//�����ջ��С	
#define USTB_SENSOR_STK_SIZE 		128  
//������
TaskHandle_t USTB_SensorTask_Handler;
//������
void USTB_sensor_task(void *pvParameters);

//�������ȼ�
#define USTB_DateExchang_TASK_PRIO		3
//�����ջ��С	
#define USTB_DateExchang_STK_SIZE 		128  
//������
TaskHandle_t USTB_DateExchangTask_Handler;
//������
void USTB_DateExchang_task(void *pvParameters);

//�������ȼ�
#define USTB_AttitudeControl_TASK_PRIO		4
//�����ջ��С	
#define USTB_AttitudeControl_STK_SIZE 		128  
//������
TaskHandle_t USTB_AttitudeControlTask_Handler;
//������
void USTB_AttitudeControl_task(void *pvParameters);



int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	delay_init(168);					//��ʼ����ʱ����
	uart_init(115200);     				//��ʼ������
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
	//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	//����KEY����
	xTaskCreate((TaskFunction_t )USTB_sensor_task,             
                (const char*    )"USTB_sensor_task",           
                (uint16_t       )USTB_SENSOR_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )USTB_SENSOR_TASK_PRIO,        
                (TaskHandle_t*  )&USTB_SensorTask_Handler);  
    //����USTB_DateExchang����
    xTaskCreate((TaskFunction_t )USTB_DateExchang_task,             
                (const char*    )"USTB_DateExchang_task",           
                (uint16_t       )USTB_DateExchang_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )USTB_DateExchang_TASK_PRIO,        
                (TaskHandle_t*  )&USTB_DateExchangTask_Handler);   
    //����USTB_AttitudeControl����
    xTaskCreate((TaskFunction_t )USTB_AttitudeControl_task,     
                (const char*    )"USTB_AttitudeControl_task",   
                (uint16_t       )USTB_AttitudeControl_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )USTB_AttitudeControl_TASK_PRIO,
                (TaskHandle_t*  )&USTB_AttitudeControlTask_Handler); 
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

//������������
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

//���ݽ���������
void USTB_DateExchang_task(void *pvParameters)
{
	while(1)
	{
		USTB_DT_Data_Exchange();
     vTaskDelay(100);                           //��ʱ1s��Ҳ����1000��ʱ�ӽ���	
	}
}

long int time1=0;
//��̬����������
void USTB_AttitudeControl_task(void *pvParameters)
{
	while(1)
	{
		User_PidSpeedControl(); //����
		Att_Control();    			//��̬����    
		time1++;
		if(time1>60000000)
			time1 = 0;
		if(time1%5 == 0) //10ms��һ��
			Att_Control_2();
    vTaskDelay(3);                           //��ʱ1s��Ҳ����1000��ʱ�ӽ���	
	}
}

