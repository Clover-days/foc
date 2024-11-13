
#include "stm32f10x.h"
#include <stdlib.h>
#include "MyProject.h"


#define LED_blink    GPIOC->ODR^=(1<<13)
/******************************************************************************/
float target;
/******************************************************************************/
void commander_run(void);
/******************************************************************************/
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//使能GPIOA,GPIOB,GPIOC,AFIO;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         //PC13是LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  //速度
	GPIO_Init(GPIOC, &GPIO_InitStructure);             //对选中管脚初始化
	GPIO_SetBits(GPIOC,GPIO_Pin_13);                   //上电点亮LED
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          //PB9是motor1的使能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_9);                  //低电平解除,Motor_init()中使能
}
/******************************************************************************/
int main(void)
{
	unsigned int count_i=0;
	
	GPIO_Config();
	uart_init(115200);
#if M1_AS5600
	I2C_Init_();               //AS5600
	printf("AS5600\r\n");
#elif M1_TLE5012B
	SPI2_Init_();              //TLE5012B
	printf("TLE5012B\r\n");
#elif M1_MT6701
	I2C_Init_();
	printf("MT6701\r\n"); 
#endif
	TIM2_PWM_Init();
	TIM3_1ms_Init();           //interrupt per 1ms
	
	delay_ms(1000);            //Wait for the system to stabilize
	MagneticSensor_Init();     //AS5600 or TLE5012B
	
	voltage_power_supply=12;   //V
	voltage_limit=4;           //V，最大值需小于12/1.732=6.9
	velocity_limit=20;         //rad/s angleOpenloop() and PID_angle() use it
	voltage_sensor_align=3;    //V     alignSensor() and driverAlign() use it，大功率电机0.5-1，小功率电机2-3
	torque_controller=Type_voltage;  //当前只有电压模式
	controller=Type_velocity_openloop;  //Type_angle; //Type_torque;    //
	target=3;
	
	Motor_init();
	Motor_initFOC();
	PID_init();
  printf("Motor ready.\r\n");
	
	systick_CountMode();   //不能再调用delay_us()和delay_ms()函数
	
	while(1)
	{
		count_i++;
		
		if(time1_cntr>=200)  //0.2s
		{
			LED_blink;
		}
		if(time2_cntr>=1000)
		{
			time2_cntr=0;
			count_i=0;
		}
		move(target);
		loopFOC();
		commander_run();
	}
}
/******************************************************************************/
void commander_run(void)
{
	if((USART_RX_STA&0x8000)!=0)
	{
		switch(USART_RX_BUF[0])
		{
			case 'H':
				printf("Hello World!\r\n");
				break;
			case 'T':   //T6.28
				target=atof((const char *)(USART_RX_BUF+1));
				printf("RX=%.4f\r\n", target);
				break;
			case 'D':   //D
				M1_Disable;
				printf("OK!\r\n");
				break;
			case 'E':   //E
				M1_Enable;
				printf("OK!\r\n");
				break;
		}
		USART_RX_STA=0;
	}
}
/******************************************************************************/



