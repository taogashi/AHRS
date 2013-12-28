#include "pwm.h"

/*定时器2比较奇葩
首先，它的两个通道的指定管脚被USART2的TX、RX占据
其次，它要负责产生一路标准的、周期为20ms的PWM波，来控制舵机
再次，它要负责捕获超声波传感器的脉宽*/
void TIM2_Config(void) //PWM正脉宽捕获
{
	u16 CCR1_Val=1000;
	u16 CCR2_Val=1400;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 	//使能定时器2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				//定时器2通道1，管脚PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;			  //定时器2通道2，管脚PA1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_BaseInitStructure.TIM_Period = 20000; //20ms       //定时器2计时周期   
    TIM_BaseInitStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1; //1us       
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
    TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 

//--------------------------------------通道1，PWM输出------------------------------------------
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //定时器2通道1PWM输出
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //PWM功能使能
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;                            //写比较值(占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;   //置高
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
//--------------------------------------通道2，PWM输出------------------------------------------
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //定时器2通道2PWM输出
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //PWM功能使能
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;                            //写比较值(占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;   //置高
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

//--------------------------------------通道3，输入捕获------------------------------------------
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 		   //定时器2通道3输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM2, &TIM_ICInitStructure);

//--------------------------------------通道4，输入捕获------------------------------------------
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 		   //定时器2通道4输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	   	
	TIM_ITConfig(TIM2,TIM_IT_CC3 | TIM_IT_CC4, ENABLE); 
  	TIM_Cmd(TIM2, ENABLE); 
}

void TIM2_IT_Config(void)	//定时器2中断
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM3_Config(void)	//PWM输出
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	uint16_t CCR1_Val = 1000;
	uint16_t CCR2_Val = 1200;
	uint16_t CCR3_Val = 1000;
	uint16_t CCR4_Val = 1000;
	uint16_t PrescalerValue = 0;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	PrescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1;
	TIM_TimeBaseStructure.TIM_Period = 5000;                                  //周期
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;              //分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;                             //时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	                //计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                    //初始TIM3
	
	/*************************** 通道1 ********************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //PWM功能使能
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;                            //写比较值(占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;   //置高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/****************************** 通道2 ******************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/******************************* 通道3 *********************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/****************************** 通道4 *********************************/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    //PWM2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);                        //
	TIM_Cmd(TIM3, ENABLE);                                          //使能计数
}

void TIM4_Config(void) //PWM正脉宽捕获
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
	TIM_ICInitTypeDef  TIM_ICInitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_BaseInitStructure.TIM_Period = 40000; //40ms          
    TIM_BaseInitStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1; //1us       
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM4, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM4, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM4, &TIM_ICInitStructure); 

   	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 		   //定时器2通道1输入捕获模式
   	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; 
   	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
   	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
   	TIM_ICInitStructure.TIM_ICFilter = 0x0;     
   	TIM_ICInit(TIM4, &TIM_ICInitStructure); 
   	
	TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE); 
  	TIM_Cmd(TIM4, ENABLE);    
}

void TIM4_IT_Config(void)	//定时器4中断
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
