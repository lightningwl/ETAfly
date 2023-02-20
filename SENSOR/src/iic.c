#include "iic.h"
#include "delay.h"

/****************************** IIC硬件初始化 ***********************************
 * 使用IIC1
 * PB8 -- SCL
 * PB9 -- SDA
 *******************************************************************************/
 

#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} 

#define IIC_SCL   PBout(6) 						//SCL
#define IIC_SDA   PBout(7) 						//SDA
#define READ_SDA  PBin(7)  						//输入SDA

#define IIC_GPIO   GPIOB
#define IIC_PIN    GPIO_PIN_6 | GPIO_PIN_7
#define IIC_ENCLK() __HAL_RCC_GPIOB_CLK_ENABLE()


#define IIC_DELAY_1		1		// 默认为4
#define IIC_DELAY_2		1		// 默认为2
#define IIC_DELAY_3		0		// 默认为1



/* 
 * IIC硬件初始化 		
 */
 
void IIC_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;							// GPIO初始化结构体
	IIC_ENCLK();																			// 使能 IIC 时钟
	
	GPIO_InitStructure.Pin 		= IIC_PIN;
	GPIO_InitStructure.Mode 	= GPIO_MODE_OUTPUT_PP;	// 推挽输出
	GPIO_InitStructure.Speed 	= GPIO_SPEED_FAST;			// 100MHz
	GPIO_InitStructure.Pull 	= GPIO_PULLUP;					// 上拉
	HAL_GPIO_Init(IIC_GPIO, &GPIO_InitStructure);			// 初始化
	
	IIC_SCL = 1;										
	IIC_SDA = 1;	
}

/****************************** IIC基本操作方法 ***********************************/
/* 
 * IIC 起始信号			
 * SCL=1时，SDA从1到0	
 */

void IIC_Start(void)
{
	SDA_OUT();     			// SDA输出模式
	IIC_SDA = 1;	  	  
	IIC_SCL = 1;				// SCL为1
	DelayUs(IIC_DELAY_1);
 	IIC_SDA = 0;				// 开始：SDA从1到0
	DelayUs(IIC_DELAY_1);
	IIC_SCL = 0;				//钳住I2C总线，准备发送或接收数据 
}	  

/* 
 * 产生IIC停止信号
 * SCL=1时，SDA从0到1	
 */

void IIC_Stop(void)
{
	SDA_OUT();				
	IIC_SCL = 0;
	IIC_SDA = 0;				
 	DelayUs(IIC_DELAY_1);
	IIC_SCL = 1; 
	IIC_SDA = 1;
	DelayUs(IIC_DELAY_1);	
}

/* 
 * 等待应答信号到来 							
 * 从机正常应答，SDA = 0						
 * 返回值：0，接收到应答						
 *        1，未接收到应答，对应开始注释第2条	
 */

uint8_t IIC_WaitAck(void)
{
	uint8_t cnt = 0;
	SDA_IN();      			//SDA设置为输入  
	IIC_SDA = 1;
	DelayUs(IIC_DELAY_3);
	IIC_SCL = 1;
	DelayUs(1);
	while (READ_SDA)		// 读取SDA，从机应答0 退出
	{
		cnt ++;
		if (cnt>250)
		{
			IIC_Stop();		// 超时退出
			return 1;
		}
	}
	IIC_SCL = 0;			   
	return 0;  
} 

/* 
 * 产生ACK应答			
 * 将SDA拉低表示应答		
 */

void IIC_Ack(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 0;		// SDA为0 -- 应答
	DelayUs(IIC_DELAY_2);
	IIC_SCL = 1;
	DelayUs(IIC_DELAY_2);
	IIC_SCL = 0;
}

/* 
 * 产生ACK应答			
 * 将SDA拉高表示非应答	
 */
		    
void IIC_NAck(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 1;		// SDA为1 -- 非应答
	DelayUs(IIC_DELAY_2);
	IIC_SCL = 1;
	DelayUs(IIC_DELAY_2);
	IIC_SCL = 0;
}			

/* 
 * IIC发送一个字节（没有等待应答）
 */
	  
void IIC_SendByte(uint8_t txd)
{                        
	uint8_t t;   
	SDA_OUT(); 	    
	IIC_SCL=0;										// 先拉低时钟再开始数据传输
	for (t=0; t<8; t++)
	{              
		IIC_SDA = (txd&0x80)>>7; 		// 传输最高位
		txd <<= 1; 	  							// 左移一位
		DelayUs(IIC_DELAY_2);   		// 对TEA5767这三个延时都是必须的
		IIC_SCL = 1;								// 拉高SCL以便读取数据
		DelayUs(IIC_DELAY_2); 
		IIC_SCL = 0;								// 拉低时钟准备下一位传输
		DelayUs(IIC_DELAY_2);
	}	
} 	   

/* 
 * 接收1个字节				
 * 返回参数：接收到的字节		
 */

uint8_t IIC_RecvByte(void)
{
	uint8_t i, recv=0;
	SDA_IN();									// SDA设置为输入
	for(i=0; i<8; i++ )
	{
		IIC_SCL = 0; 
		DelayUs(IIC_DELAY_2);
		IIC_SCL = 1;
		if(READ_SDA)						// 读取到1，recv左移1位并加1
		{
			recv <<= 1;   
			recv += 0x01;
		}
		else										// 读取到0，recv左移1位并加0
		{
			recv <<= 1;
			recv += 0x00;
		}
		DelayUs(IIC_DELAY_3); 
	}					 
    return recv;
}






/****************************** IIC组合操作方法 ***********************************/

/* 
 * IIC发送一个字节											
 * 返回从机有无应答，相当于 IIC_SendByte() + IIC_WaitAck()	
 * ack=0表示应答，ack=1 表示无应答！！！						
 */

uint8_t IIC_WriteByte(uint8_t txd)
{                        
  uint8_t ack;   
	IIC_SendByte(txd);
	ack = IIC_WaitAck();			// 等待从机应答，t=0表示应答，t=1 表示无应答！！！
	return ack;
} 


/* 
 * IIC在指定器件指定寄存器发送一个字节					
 * 返回从机有无应答										
 * 0 表示应答（发送成功），1 表示无应答（发送失败）！！！	
 */

uint8_t IIC_WriteAddrRegByte(uint8_t addr, uint8_t reg, uint8_t dat)
{          
	IIC_Start();
	if (IIC_WriteByte(addr))
	{
		IIC_Stop();
		//printf("IIC write addr fail\r\n");
		return 1;
	}
	if (IIC_WriteByte(reg))
	{
		IIC_Stop();
		printf("IIC write reg fail\r\n");
		return 1;
	}
	if (IIC_WriteByte(dat))
	{
		IIC_Stop();
		printf("IIC write data fail\r\n");
		return 1;
	}
	IIC_Stop();
	return 0;
}

/* 
 * IIC在指定器件指定寄存器发送一个字节					
 * 返回从机有无应答										
 * 0 表示应答（发送成功），1 表示无应答（发送失败）！！！	
 */

uint8_t IIC_WriteAddrRegBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	IIC_Start();
	if (IIC_WriteByte(addr))
	{
		IIC_Stop();
		return 1;
	}
	if (IIC_WriteByte(reg))
	{
		IIC_Stop();
		return 1;
	}
	for (i=0; i<len; i++)
	{
		if (IIC_WriteByte(buf[i]))
		{
			IIC_Stop();
			return 1;
		}	
	}
	IIC_Stop();
	return 0;
}


/* 
 * 读1个字节，相当于 IIC_RecvByte() + IIC_(N)Ack()				    
 * ack=0时，发送ACK；ack=1，发送NACK（不是故意反逻辑，原因见开头注释2） 
 * 返回读取的字节													'
 */

uint8_t IIC_ReadByte(uint8_t ack)
{
	uint8_t recv = 0;
	recv = IIC_RecvByte();
    if (ack)
        IIC_NAck();						// ack=1 发送nACK
    else
        IIC_Ack(); 						// ack=0 发送ACK   
    return recv;
}

/* 
 * 在给定器件给定寄存器读出一个数据	
 */
uint8_t IIC_ReadAddrRegByte(uint8_t addr, uint8_t reg)
{
	uint8_t recv;
	
	IIC_Start();
	IIC_WriteByte(addr);
	IIC_WriteByte(reg);
	
	IIC_Start();
	IIC_WriteByte(addr | 0x01);
	recv = IIC_ReadByte(1);				// NAck
	IIC_Stop();
	return recv;
}

/* 
 * 在给定器件给定寄存器读出多个数据	
 */
uint8_t IIC_ReadAddrRegBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	IIC_Start();
	if (IIC_WriteByte(addr))
	{
		IIC_Stop();
		return 1;
	}
	IIC_WriteByte(reg);
	
	IIC_Start();
	IIC_WriteByte(addr | 0x01);
	for (i=0; i<len; i++)
	{
		if (i == len-1)
			buf[i] = IIC_ReadByte(1);	// 最后一字节 NAck
		else
			buf[i] = IIC_ReadByte(0);	// 不是最后一字节时 Ack
	}
	IIC_Stop();
	return 0;
}
	
	


