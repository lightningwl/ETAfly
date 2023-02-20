#ifndef MYIIC_H
#define MYIIC_H
#include "sys.h"


/*	
 * IIC基本操作方法	
 */

void IIC_Init(void);                		// 初始化IIC的IO口	

void IIC_Start(void);										// 发送IIC开始信号
void IIC_Stop(void);	  								// 发送IIC停止信号
void IIC_Ack(void);											// IIC发送ACK信号
void IIC_NAck(void);										// IIC发送NACK信号
void IIC_SendByte(uint8_t txd);					// IIC发送一个字节（不带等待应答）
uint8_t IIC_RecvByte(void);							// IIC接收一个字节（不带应答）
uint8_t IIC_WaitAck(void); 							// IIC等待ACK信号

/*
 * IIC组合操作方法	
 */

uint8_t IIC_WriteByte(uint8_t txd);																										// IIC发送一个字节（带等待应答）
uint8_t IIC_WriteAddrRegByte(uint8_t addr, uint8_t reg, uint8_t dat);									// 在给定器件给定寄存器写入一个给定数据
uint8_t IIC_WriteAddrRegBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf); 	// 在给定器件给定寄存器写入多个给定数据

uint8_t IIC_ReadByte(uint8_t ack);																										// IIC读取一个字节
uint8_t IIC_ReadAddrRegByte(uint8_t addr, uint8_t reg);																// 在给定器件给定寄存器读出一个数据
uint8_t IIC_ReadAddrRegBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);		// 在给定器件给定寄存器读出多个数据


#endif

