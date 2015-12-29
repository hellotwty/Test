#include "stm32f10x.h"
#include "usart1.h"
#include "main.h"
#include "W5100.h"

#define SET_SCS         GPIO_SetBits(GPIOA, GPIO_Pin_4);   //SCS=1; 
#define RESET_SCS       GPIO_ResetBits(GPIOA, GPIO_Pin_4); //SCS=0; 

#define SP_EN         	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)  //第一个按键

#define KXT22_AB_VIN_P	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7)//Kxt22 对讲/打点按钮

#define KXT22_AB_VIN_N	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_8)//Kxt22 急停按钮

#define DECTECTION_CALL	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) //检测打点信号的输入




#define LED0_ON			GPIO_ResetBits(GPIOD, GPIO_Pin_5);
#define LED0_OFF 		GPIO_SetBits(GPIOD, GPIO_Pin_5);
#define LED1_ON			GPIO_ResetBits(GPIOD, GPIO_Pin_6);
#define LED1_OFF 		GPIO_SetBits(GPIOD, GPIO_Pin_6);
#define LED2_ON			GPIO_ResetBits(GPIOD, GPIO_Pin_7);
#define LED2_OFF 		GPIO_SetBits(GPIOD, GPIO_Pin_7);

#define LED2_ON			GPIO_ResetBits(GPIOD, GPIO_Pin_7);
//#define watchdog 		GPIO_SetBits(GPIOD, GPIO_Pin_7);

#define watchdog 		GPIO_WriteBit(GPIOE, GPIO_Pin_13,(BitAction)(1-(GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_13))));


#define SWITCH_7261_EN_OFF 			GPIO_SetBits(GPIOE, GPIO_Pin_15);
#define PA_SHDN_OFF 				GPIO_SetBits(GPIOE, GPIO_Pin_12);
#define KXT22_AB_VOUT_P_OFF 		GPIO_SetBits(GPIOE, GPIO_Pin_10);check_kxt22=0;
#define CD4060_EN_OFF 				GPIO_SetBits(GPIOE, GPIO_Pin_11);
#define CALL_SWITCH_EN_OFF 			GPIO_SetBits(GPIOE, GPIO_Pin_14);
#define ACTIVE_NODE_OUT1_EN_OFF 	GPIO_SetBits(GPIOE, GPIO_Pin_3);
#define ACTIVE_NODE_OUT2_EN_OFF 	GPIO_SetBits(GPIOE, GPIO_Pin_4);
#define PASSIVE_NODE_OUT_EN_OFF 	GPIO_SetBits(GPIOC, GPIO_Pin_5);
#define DD_EN_OFF 					GPIO_SetBits(GPIOC, GPIO_Pin_4);
#define ANBP_CTL_OFF				GPIO_SetBits(GPIOE, GPIO_Pin_9);


#define SWITCH_7261_EN_ON 			GPIO_ResetBits(GPIOE, GPIO_Pin_15);
#define PA_SHDN_ON 					GPIO_ResetBits(GPIOE, GPIO_Pin_12);
#define KXT22_AB_VOUT_P_ON 			GPIO_ResetBits(GPIOE, GPIO_Pin_10);check_kxt22=1;//对22操作标志置1
#define CD4060_EN_ON 				GPIO_ResetBits(GPIOE, GPIO_Pin_11);
#define CALL_SWITCH_EN_ON 			GPIO_ResetBits(GPIOE, GPIO_Pin_14);
#define ACTIVE_NODE_OUT1_EN_ON 		GPIO_ResetBits(GPIOE, GPIO_Pin_3);
#define ACTIVE_NODE_OUT2_EN_ON		GPIO_ResetBits(GPIOE, GPIO_Pin_4);
#define PASSIVE_NODE_OUT_EN_ON 		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
#define DD_EN_ON					GPIO_ResetBits(GPIOC, GPIO_Pin_4);
#define ANBP_CTL_ON					GPIO_ResetBits(GPIOE, GPIO_Pin_9);



extern void delay(__IO uint32_t nCount);
extern void delayMs(__IO uint32_t nCount);

unsigned short Encode_Flag=0;//本地对讲按钮
//unsigned short Ticker_Flag=1;//本地打点按钮



unsigned short kxt22_ticker_number=0;//存放kxt22 打点数量

//放入上位机的IP mask  port

unsigned char PC_Port[2]={0x13,0x88}; 	  /* PC Port number   */
unsigned char PC_DIP[4]={192,168,200,10};      /* PC Destination IP Address */
unsigned char PC_DPort[2]={0x13,0x88}; 		/* PC Destination Port number 5000 */
unsigned char PC_Data[12];    /* PC Data 4位IP地址+2位累加+2位命令+2位数量 */


unsigned char ticker_start=0;	//打点用变量
unsigned char ticker_on=0;
unsigned char ticker_off=0;
unsigned char ticker_1s=0;

unsigned char emergency_stop_1s=0;		//急停用变量


unsigned char common_ticker_start=0;  //传统打点用变量
unsigned char common_ticker_3s=0;
unsigned char common_ticker_on=0;

unsigned char check_kxt22=0;//用于判断是否有设备向kxt22喊话


///////////////////////////////////////////////////////////////////////////////

void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}


void SPI1_w5100_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable SPI1 and GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD, ENABLE);

    /*!< SPI_ _SPI Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    /*!< AFIO Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /*!< Configure SPI_ _SPI pins: SCK */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*!< Configure SPI_ _SPI pins: MISO */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*!< Configure SPI_ _SPI pins: MOSI */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*!< Configure SPI_ _SPI_CS_PIN pin: SPI_  nss pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* SPI1 configuration */
    // w5100: data input on the DIO pin is sampled on the rising edge of the CLK. 
    // Data on the DO and DIO pins are clocked out on the falling edge of CLK.
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;  // spi mode 0
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);


    SPI_Cmd(SPI1, ENABLE);


    GPIO_ResetBits(GPIOA, GPIO_Pin_4);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //GPIO_SetBits  (GPIOA, GPIO_Pin_2);Delay(9999);

    GPIO_ResetBits(GPIOA, GPIO_Pin_2);Delay(9999);

    GPIO_SetBits  (GPIOA, GPIO_Pin_2);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
}


void GPIO_test(void)
{
	
	//使能GPIO对应的外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    
    //GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
    GPIO_InitTypeDef GPIO_InitStructure; //申明结构，体结构体原型由GPIO_InitTypeDef确定

	//初始化PD.5.6.7口推挽输出for test led
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOD, GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);//初始化成高电平


    
}


u8 SPI1_ReadWriteByte(u8 TxData)//SPI读写函数
{
	u16 retry=0;  
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)//检查指定的SPI标志位设置与否:发送缓存空标志位
    {
		retry++;
         if(retry>1000) return 0;
    }

    SPI_I2S_SendData(SPI1, TxData);
    
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
         if(retry>1000) return 0;
    }

    return SPI_I2S_ReceiveData(SPI1); 

}


void Write_W5100_SPI(unsigned int addr, unsigned char dat)
{
    RESET_SCS /* SPI的片选置低 */

    /* 输出的第一个字节为写命令 */
    SPI1_ReadWriteByte(0xf0); /* 写命令*/

    /* 输出的第二个字节为地址高8位 */
    SPI1_ReadWriteByte(addr / 256); /* 地址高8位 */

    /* 输出的第三个字节为地址低8位 */
    SPI1_ReadWriteByte(addr); /* 地址低8位 */

    /* 输出的第四个字节为写入的数据 */
    SPI1_ReadWriteByte(dat);

    SET_SCS /* SPI的片选置高 */
}


/********************************************************************
通过SPI读取W5100的数据
输入参数：读取地址addr，
返回参数：读取的数据
 ********************************************************************/
unsigned char Read_W5100_SPI(unsigned int addr)
{
    unsigned char j;

    RESET_SCS /* SPI的片选置低 */

    /* 输出的第一个字节为写命令 */
    SPI1_ReadWriteByte(0x0f); /* 读命令*/

    /* 输出的第二个字节为地址高8位 */
    SPI1_ReadWriteByte(addr / 256); /* 地址高8位 */

    /* 输出的第三个字节为地址低8位 */
    SPI1_ReadWriteByte(addr); /* 地址低8位 */

    /* 输出一个空字节，读取一个字节的数据 */
    j = SPI1_ReadWriteByte(0xaa);

    SET_SCS /* SPI的片选置高电平 */
    return j;

}

void CML_CBUSInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable  GPIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/*!< Configure CBUS pins: SCLK */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //  推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*!< Configure CBUS pins: RESETEN*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  //  推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/*!< Configure CBUS pins: CSN */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //  推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/*!< Configure CBUS _PIN pin:  CDATA */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //  推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	/*!< Configure CBUS _PIN pin:  RDATA */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //  输入 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	/*!< Configure CBUS _PIN pin:  IRQN */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //  输入 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	/*!< AFIO Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

}



////////////////////////////////////////////////////////////////////////////////
/*!< Configure   W5100  :  INT */

void W5100_NVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;                 //第一结构体
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);      //选择中断分组2 优先级
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;     //选择中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //使能中断
    NVIC_Init(&NVIC_InitStructure);
	
}

// 需要设置将GPIO相应的引脚和中断线连接起来
void EXTI_W5100(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    //清空中断标志
    EXTI_ClearITPendingBit(EXTI_Line0);

    //选择中断管脚 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); //PA0中断

    EXTI_InitStructure.EXTI_Line = EXTI_Line0; //选择中断线路 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置为中断请求，非事件请求
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //设置中断触发方式为上下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;		 //外部中断使能
    EXTI_Init(&EXTI_InitStructure);
}


////////////////////////////////////////////////////////////////////////////////
/*!< Configure   22.1急停  :  INT */

void Kxt22_stop_NVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;                 //第一结构体
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);      //选择中断分组2 优先级
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;     //选择中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //使能中断
    NVIC_Init(&NVIC_InitStructure);
	
}

// 需要设置将GPIO相应的引脚和中断线连接起来
void EXTI_Kxt22_stop(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    //清空中断标志
    EXTI_ClearITPendingBit(EXTI_Line8);

    //选择中断管脚 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource8); //PE8中断

    EXTI_InitStructure.EXTI_Line = EXTI_Line8; //选择中断线路 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置为中断请求，非事件请求
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //设置中断触发方式为上下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;		 //外部中断使能
    EXTI_Init(&EXTI_InitStructure);
}





///////////////////
//对外IO口初始化

void Switch_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable  GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);

	//拨码开关初始化成浮空输入(PD8-14)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//按键开关初始化成浮空输入(PC6-9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	//开关初始化成浮空输入(PE2&5-8)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//开关初始化成推挽输出(PE3-4&9-11&14-15)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_Init(GPIOE, &GPIO_InitStructure);


	//开关初始化成开漏输出(PE12)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	
	//开关初始化成推挽输出(PC4-5)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	//开关初始化成浮空输入(PB0)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//开关初始化成推挽输出(PB1)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//关闭全部的开关
	SWITCH_7261_EN_OFF 		
	PA_SHDN_OFF 			
	KXT22_AB_VOUT_P_OFF 	
	CD4060_EN_OFF 			
	CALL_SWITCH_EN_OFF 		
	ACTIVE_NODE_OUT1_EN_OFF 
	ACTIVE_NODE_OUT2_EN_OFF 
	PASSIVE_NODE_OUT_EN_OFF 
	//DD_EN_OFF 
	DD_EN_ON //打开打点检测使能
	ANBP_CTL_OFF


}
//手机开始对讲开关控制函数
void Mobile_speak_on(void)
{
	SWITCH_7261_EN_ON
	PA_SHDN_ON
	KXT22_AB_VOUT_P_ON

}

//手机结束对讲开关控制函数
void Mobile_speak_off(void)
{
	SWITCH_7261_EN_OFF
	PA_SHDN_OFF
	KXT22_AB_VOUT_P_OFF
}

//急停开关控制函数
void emergency_stop(void)
{

	ACTIVE_NODE_OUT2_EN_ON
	PASSIVE_NODE_OUT_EN_ON
	emergency_stop_1s=0;   //定时1S计数清零
	printp(uart_putchar,"执行急停动作\r\n" );

}

//急停开关结束函数
void emergency_stop_clear(void)
{

	ACTIVE_NODE_OUT2_EN_OFF
	PASSIVE_NODE_OUT_EN_OFF
	printp(uart_putchar,"急停动作结束\r\n" );

}


//手机打点开控制
void Mobile_ticker_ON(void)
{
	PA_SHDN_ON
	KXT22_AB_VOUT_P_ON
	CD4060_EN_ON
	CALL_SWITCH_EN_ON
	ACTIVE_NODE_OUT1_EN_ON

}

//手机打点关控制
void Mobile_ticker_OFF(void)
{
	PA_SHDN_OFF
	KXT22_AB_VOUT_P_OFF
	CD4060_EN_OFF
	CALL_SWITCH_EN_OFF
	ACTIVE_NODE_OUT1_EN_OFF

}


//KXT22打点开关控制
void kxt22_ticker(void)
{

	PA_SHDN_ON
	ACTIVE_NODE_OUT1_EN_ON	
	DD_EN_ON

}







unsigned short Rtp_seq = 0x1223; 

unsigned long Rtp_Timestamp = 0x14cdf70;

void TIM2_IRQHandler(void)          //20ms定时器中断，给发过来数据打包成网络包
{
    unsigned short i=0;

	unsigned short s;

	static unsigned char P_20Bytes[2][20];

	static unsigned short overtime=0; 
	if(Tx_Buffer[1][7]==0x03)//定时1s for 打点信号
	{
		emergency_stop_1s++;
		if (emergency_stop_1s==50)
		{
			Tx_Buffer[1][7]=0x00;
			emergency_stop_1s=0;

			emergency_stop_clear();
			

		}
	}

 
        
        if(ticker_start==1)//定时1s for 打点信号
	{
		ticker_1s++;
		if (ticker_1s==1)
		{
			ticker_on=1;//一次打点开始标志

		}
		if (ticker_1s==31)
		{
			ticker_off=1;//一次打点结束标志
			ticker_1s=0;//定时计数器清零
			
		}
	}
	if( common_ticker_start==1)//定时3s for 打点信号
	{
			common_ticker_3s++;

			if (common_ticker_3s==250)
			{
				common_ticker_on=1;
				common_ticker_3s=0;
			}
	}

/*
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) 
    {
        TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update); 

		if(Cur_p>=10) 
		{

			for(i=0;i<20; i++)
			{
				P_20Bytes[1][i] = P_20Bytes[0][i];   
			}
			
			for(i=0;i<10; i++)
			{
				P_20Bytes[0][2*i] = Send_G_729a_Buff[i]>>8;    
				P_20Bytes[0][2*i+1] = Send_G_729a_Buff[i] & 0x00ff; 
			}
			
			
			s=0;
			for(i=0;i<=23; i++)
			{
				Tx_Buffer[s][i] = Head_ptt_send[i];
			}
			for(i=24;i<=35; i++) 
			{
				Tx_Buffer[s][i] = Head_Rtp[i-24];
			}
			for(i=36;i<=55; i++) 
			{
				Tx_Buffer[s][i] =  P_20Bytes[1][i-36];
			}
			for(i=56;i<=59; i++)
			{
				Tx_Buffer[s][i] = 0x5a;
			}
			for(i=60;i<=71; i++) 
			{
				Tx_Buffer[s][i] = Head_Rtp[i-60];
			}
			for(i=72;i<=91; i++) 
			{
				Tx_Buffer[s][i] =  P_20Bytes[0][i-72];
			}
			Tx_Buffer[s][26] = Rtp_seq >> 8;
			Tx_Buffer[s][27] = Rtp_seq & 0x00ff;

			Rtp_seq ++;

			Tx_Buffer[s][62] = (Rtp_seq) >> 8;
			Tx_Buffer[s][63] = (Rtp_seq) & 0x00ff;
			
			Tx_Buffer[s][28] = (Rtp_Timestamp>>24)& 0x000000ff;
			Tx_Buffer[s][29] = (Rtp_Timestamp>>16)& 0x000000ff;
			Tx_Buffer[s][30] = (Rtp_Timestamp>>8) & 0x000000ff;
			Tx_Buffer[s][31] =  Rtp_Timestamp & 0x000000ff;

			Rtp_Timestamp = Rtp_Timestamp+160;
			
			Tx_Buffer[s][64] = (Rtp_Timestamp>>24)& 0x000000ff;
			Tx_Buffer[s][65] = (Rtp_Timestamp>>16)& 0x000000ff;
			Tx_Buffer[s][66] = (Rtp_Timestamp>>8) & 0x000000ff;
			Tx_Buffer[s][67] =  Rtp_Timestamp & 0x000000ff;
			

			for(i=0;i<=83; i++) 
			{
				Tx_Buffer[0][i] = Tx_Buffer[0][i+8];
			}


			
			S_tx_process(0, 84);
			

			
			for(i=0; i<Cur_p-10; i++)
			{
				Send_G_729a_Buff[i] = Send_G_729a_Buff[10+i];
			}
			Cur_p = Cur_p - 10;		

		}	
		else if( Cur_p<=9 && Cur_p>=1)  
		{
			overtime++;
			if(overtime>=16)
			{
				overtime = 0;
				
				Cur_p = 0;
			}
		}
		else
		{
			 
		}
		
    }

    */
}




void Timer2_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//设置优先分级组
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;	
	//先占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;		
	//从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE; 		
	//使能中断向量配置
	NVIC_Init(&NVIC_InitStructure);			
	//按以上设置初始化中断向量

	
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    //定义TIM结构体变量
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    //使能TIM2外设
    TIM_DeInit(TIM2);//复位时钟TIM2，恢复到初始状态
	//采用内部时钟给TIM2提供时钟源
	TIM_InternalClockConfig(TIM2);
    TIM_TimeBaseStructure.TIM_Period=200-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;//预分频系数为7200-1，这样计数器时钟为72MHz/7200=10KHz=0.1ms
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //TIM2 作用是做一段延时
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //计数方式
    //定时时间T计算公式：  T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK=(35999+1)*(1999+1)/72MHz=1s
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure); //初始化
    TIM_ClearFlag(TIM2,TIM_FLAG_Update);   //清除标志
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //使能中断源
    TIM_Cmd(TIM2,ENABLE); //使能TIM2

}


void EXTI9_5_IRQHandler(void) //急停中断函数

{

       if(EXTI_GetITStatus(EXTI_Line8)!= RESET)
	   	{

            EXTI_ClearITPendingBit(EXTI_Line8);
			LED2_ON;
            emergency_stop();
			Tx_Buffer[1][0] = 0x0A;//默认kxt22的IP地址为10.10.10.10
			Tx_Buffer[1][1] = 0x0A;
			Tx_Buffer[1][2] = 0x0A;
			Tx_Buffer[1][3] = 0x0A;
			Tx_Buffer[1][4] = 0x00;
			Tx_Buffer[1][5] = ++SendPC_count;//序列号
			Tx_Buffer[1][6] = 0x00;
			Tx_Buffer[1][7] = 0x03;
			Tx_Buffer[1][8] = 0x00;
			Tx_Buffer[1][9] = 0x00;//添加命令内容

			S_tx_process(1,12);//将构建成功的上位机显示包发送出去

        }

}












/////////////////////////////////////////////////////////////////////
int main(void)
{
    unsigned short i=0;
    unsigned short Size_num=9;
    unsigned char n=0;

	unsigned char common_ticker_numbe_tem=0;
	unsigned short Pre_DECTECTION_CALL=1;//存放前一个状态的打点识别状态
	unsigned short Pre_KXT22_AB_VIN_P=1;//存放前一个状态的讲话识别状态

	static unsigned short in_=0; 

    GPIO_test();

    /* USART1 config 115200 8-N-1 */
    USART1_Config();

    printp(uart_putchar,"天地科技,STM32 \r\n");  

    SPI1_w5100_Init(); //w5100与stm32 spi接口初始化

    EXTI_W5100();//定义w5100外部中断

    W5100_NVIC();

    W5100_Init();//w5100初始化函数

    Socket_Init(0);//socket 0 init for w5100

    Socket_Init(3); //socket 3 init for cmx7261

    i = Socket_UDP(3); //socket3 set UDP model
    
    i = Socket_UDP_Multicasting(0);//socket set UDP  multicast model

	//i = Socket_UDP(0);


	Socket_Init(1);//socket 1 init for w5100

	i = Socket_UDP(1); //socket1 set UDP model

    Switch_Init() ;

	Timer2_init();
	Kxt22_stop_NVIC();
	EXTI_Kxt22_stop();
	LED2_ON
	
    while(1)
    {

 
		watchdog  //看门狗引脚反转 TPS382x 1.6s最小反转时间

		//测试用
        if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8)==1)
        {
            //GPIO_SetBits(GPIOD, GPIO_Pin_5 );
            LED0_ON
        }
        else  // GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)==0
        {
            //GPIO_ResetBits(GPIOD, GPIO_Pin_5 );  
			LED0_OFF
			//Cmx7261_G729A_Dec_Inttest();
					    
        }

 
/*

		if(check_kxt22==0&&ticker_number==0&&Encode_Flag==0 )//先判断是否有手机在打点或讲话，如没有则响应Kxt22打点或对将信号
		{
			//检测到 Kxt22对讲或打点信号，KXT22_AB_VIN_P=0
			if (KXT22_AB_VIN_P==0)
			{
				
				if (KXT22_AB_VIN_P==0)
				{
					PA_SHDN_ON //打开功放使能，先发声 在判断是打点信号还是讲话
				}
			delayMs(200); //DECTECTION_CALL信号大概比KXT22_AB_VIN_P延时100ms左右出现


			if ((DECTECTION_CALL==0)&&(KXT22_AB_VIN_P==0))//判断是否是打点信号
					{
						delayMs(10);
						
						if((DECTECTION_CALL==0)&&(Pre_DECTECTION_CALL==1))
						{
							
							Pre_DECTECTION_CALL =0;
							ACTIVE_NODE_OUT1_EN_ON
							Tx_Buffer[1][0] = 0x0F;//默认kxt22的IP地址为10.10.10.10
							Tx_Buffer[1][1] = 0x0F;
							Tx_Buffer[1][2] = 0x0F;
							Tx_Buffer[1][3] = 0x0F;
							Tx_Buffer[1][4] = 0x00;
							Tx_Buffer[1][5] = ++SendPC_count;//序列号
							Tx_Buffer[1][6] = 0x00;
							Tx_Buffer[1][7] = 0x04;//开始讲话命令
							Tx_Buffer[1][8] = 0x00;
							Tx_Buffer[1][9] = 0x00;//添加命令内容
							Tx_Buffer[1][10] = 0xff;
							Tx_Buffer[1][11] = 0xff;
						
							//i = S_tx_process(1,12);//将构建成功的上位机显示包发送出去
						}

						
					}
			if((DECTECTION_CALL==1)&&(KXT22_AB_VIN_P==0))//判断是否是讲话信号
					{
						delayMs(10);

						if ((DECTECTION_CALL==1)&&(Pre_KXT22_AB_VIN_P==1))
						{
							Pre_KXT22_AB_VIN_P = 0;
							Tx_Buffer[1][0] = 0x0A;//默认kxt22的IP地址为10.10.10.10
							Tx_Buffer[1][1] = 0x0A;
							Tx_Buffer[1][2] = 0x0A;
							Tx_Buffer[1][3] = 0x0A;
							Tx_Buffer[1][4] = 0x00;
							Tx_Buffer[1][5] = ++SendPC_count;//序列号
							Tx_Buffer[1][6] = 0x00;
							Tx_Buffer[1][7] = 0x04;//开始讲话命令
							Tx_Buffer[1][8] = 0x00;
							Tx_Buffer[1][9] = 0x00;//添加命令内容
							Tx_Buffer[1][10] = 0xff;
							Tx_Buffer[1][11] = 0xff;
						
							i = S_tx_process(1,12);//将构建成功的上位机显示包发送出去
						}

					}
			}

			
			
			if ( KXT22_AB_VIN_P==1)
			{
				PA_SHDN_OFF 
				
				ACTIVE_NODE_OUT1_EN_OFF
				
				if( Pre_KXT22_AB_VIN_P==0)
				{
						
					Pre_KXT22_AB_VIN_P = 1;
					Tx_Buffer[1][0] = 0x0A;//默认kxt22的IP地址为10.10.10.10
					Tx_Buffer[1][1] = 0x0A;
					Tx_Buffer[1][2] = 0x0A;
					Tx_Buffer[1][3] = 0x0A;
					Tx_Buffer[1][4] = 0x00;
					Tx_Buffer[1][5] = ++SendPC_count;//序列号
					Tx_Buffer[1][6] = 0x00;
					Tx_Buffer[1][7] = 0x05;//结束讲话命令
					Tx_Buffer[1][8] = 0x00;
					Tx_Buffer[1][9] = 0x00;//添加命令内容
					Tx_Buffer[1][10] = 0xff;
					Tx_Buffer[1][11] = 0xff;
				
					i = S_tx_process(1,12);//将构建成功的上位机显示包发送出去
					
				}

				if(DECTECTION_CALL==1&&Pre_DECTECTION_CALL==0)
						
						{
							

							kxt22_ticker_number++;
							Pre_DECTECTION_CALL = DECTECTION_CALL;
							common_ticker_start=1;// 3s计时开始 
							common_ticker_3s=0;
								
						}

						//判断是否最后一次打点后有3s时间
				if(kxt22_ticker_number&&common_ticker_on==1)
				{
					
						common_ticker_start=0;
						common_ticker_on=0;
						common_ticker_3s=0;	
						
						printp(uart_putchar,"传统打点结束一共%d下\r\n" ,kxt22_ticker_number);
						
						Tx_Buffer[1][0] = 0x0A;//默认kxt22的IP地址为10.10.10.10
						Tx_Buffer[1][1] = 0x0A;
						Tx_Buffer[1][2] = 0x0A;
						Tx_Buffer[1][3] = 0x0A;
						Tx_Buffer[1][4] = 0x00;
						Tx_Buffer[1][5] = ++SendPC_count;//序列号
						Tx_Buffer[1][6] = 0x00;
						Tx_Buffer[1][7] = 0x01;//打点命令
						Tx_Buffer[1][8] = 0x00;
						Tx_Buffer[1][9] = kxt22_ticker_number;//添加命令内容
						Tx_Buffer[1][10] = 0xff;
						Tx_Buffer[1][11] = 0xff;
						
						i = S_tx_process(1,12);//将构建成功的上位机显示包发送出去

						kxt22_ticker_number=0;

								
							
				}
			}
			
			
			
		}


*/
		
		//打点函数for 手机单键打点及传统打点
		if (ticker_number!=0)
		{
			ticker_start=1;//定时器开始计时，1s打点一次
			
		  	if (ticker_on==1)//判断打点开始
			{
				ticker_on=0;
				//printp(uart_putchar,"打点第%d下开始\r\n" ,ticker_number);
				Mobile_ticker_ON();
			
			}
			
			else if (ticker_off==1)//判断打点结束
			{

				ticker_off=0;

				//printp(uart_putchar,"打点第%d下结束\r\n" ,ticker_number);
			
				Mobile_ticker_OFF();
				
				ticker_number--;//打点数量减1
				if (ticker_number==0)
					ticker_start=0;
			}


		}


		if (common_ticker_numbe!=0)//判断传统打点结束后发包给上位机
		{
			
			if(common_ticker_numbe_tem!=common_ticker_numbe)
			{
				common_ticker_numbe_tem=common_ticker_numbe;
				common_ticker_3s=0;
				common_ticker_start=1;// 3s计时开始	
			}	
			
			if(common_ticker_on==1&&common_ticker_numbe_tem==common_ticker_numbe)
			{
				common_ticker_on=0;

				common_ticker_start=0;
				printp(uart_putchar,"传统打点结束一共%d下\r\n" ,common_ticker_numbe);

				Tx_Buffer[1][4] = 0x00;
				Tx_Buffer[1][5] = ++SendPC_count;//序列号
				Tx_Buffer[1][8] = 0x00;
				Tx_Buffer[1][9] = common_ticker_numbe;//添加命令内容
				Tx_Buffer[1][10] = 0xff;
				Tx_Buffer[1][11] = 0xff;

				i = S_tx_process(1,12);//将构建成功的上位机显示包发送出去
				common_ticker_numbe=0;
				common_ticker_numbe_tem=0;
				Rx_Buffer3[1]=0xff;


				
				
			}
			

		}

		

}
}
		
		
   
	



