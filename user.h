/******************************************************************************
;  *  @芯片             : MC30F6920
;  *  @工程             : rf_relay_6920  (RF 接收转发, 33us 采样)
;  *  @功能             : PT14 接收 WF550RA 的 DO 数据
;  *                      PT13 转发(跟随 DO)
;  *                      PT15 有效指示(收到=低, 没收到=高)
;  *  @公司             : SINOMCU
;  *---------------------- 说明 ----------------------------------------------
;  *  头文件: 类型定义 / 全局变量声明 / 函数声明
;  *  注意: 本文件与 main.c 配套, 编译时一起加入工程
******************************************************************************/
#ifndef USER
#define USER

#include "mc30-common.h"
#include "mc30F6920.h"

#define u8  unsigned char
#define u16 unsigned int
#define u32 unsigned long int

#define uint8_t  unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long int

#define DEF_SET_BIT0   0x01
#define DEF_SET_BIT1   0x02
#define DEF_SET_BIT2   0x04
#define DEF_SET_BIT3   0x08
#define DEF_SET_BIT4   0x10
#define DEF_SET_BIT5   0x20
#define DEF_SET_BIT6   0x40
#define DEF_SET_BIT7   0x80

#define DEF_CLR_BIT0   0xFE
#define DEF_CLR_BIT1   0xFD
#define DEF_CLR_BIT2   0xFB
#define DEF_CLR_BIT3   0xF7
#define DEF_CLR_BIT4   0xEF
#define DEF_CLR_BIT5   0xDF
#define DEF_CLR_BIT6   0xBF
#define DEF_CLR_BIT7   0x7F

#define ENABLE   1
#define DISABLE  0

/* 中断上下文保存用的暂存变量(不可删) */
u8 abuf;
u8 statusbuf;

/* 函数声明 */
void CLR_RAM(void);
void IO_Init(void);
void TIMER1_INT_Init(void);
void Sys_Init(void);
void Delay(u16 n);
void IR_Rx_fun(void);

#endif
/**************************** end of file ************************************/
