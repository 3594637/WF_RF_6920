/******************************************************************************
;  *   	@型号	   	   	   	  : MC30F6920
;  *   	@创建日期             : 2021.08.04
;  *   	@公司/作者 	   	   	  : SINOMCU-FAE
;  *   	@晟矽微技术支持       : 2048615934
;  *   	@晟矽微官网           : http://www.sinomcu.com/
;  *   	@版权                 : 2021 SINOMCU公司版权所有.
;  *----------------------摘要描述---------------------------------
;  *   	   	   	   	TIME  定时器使用
;  *   	   	   	   	T0 	1mS中断  P15翻转
;  *   	   	   	   	T1 	100uS中断 P14翻转
******************************************************************************/
#include    "user.h"

#define io_rfdata   P14D        /* 输入: WF550RA DO      */
#define io_OUT      P13D        /* 输出: 转发            */
#define io_RxEnable P15D        /* 输出: (低有效)   	   	*/
#define io_dbg      P10D        /* 调试脚                                 */

/* ======================== 解码阈值(33us tick) =========================== */
#define LEVEL_TOUT   64         /* 单电平超时 2.1ms */
#define BIT_MIN      10         /* 1bit 总宽下限 330us */
#define BIT_MAX      80         /* 1bit 总宽上限 2.64ms */
#define VALID_BITS   4          /* 连续合法 bit 数 */

/* ======================== 系统状态 ====================== */
#define ST_LISTEN    0          /* 监听 10ms */
#define ST_FORWARD   1          /* 转发 */
#define ST_SLEEP     2          /* 休眠 200ms */


/* ====================== 片选等级(PT11/PT12) ====================== */
#define WIN_L0        200      /* 6.06ms */
#define WIN_L1        333      /* 11.0ms */
#define WIN_L2        485      /* 16.0ms */
#define EXT_L0        300      /* 9.9ms  */
#define EXT_L1        620      /* 20.5ms */
#define EXT_L2        850      /* 28.0ms */
#define BIT_TK_L0     16       /* 位宽中值 10k=12, 5k=19   */
#define BIT_TK_L1     35       /* 4k=30, 2.86k=42          */
#define BIT_TK_L2     54       /* 2.5k=48, 2k=61           */
#define WIN_EXT_MAX   788      /* 动态延长上限 26ms        */
#define SYNC_MIN      90       /* 同步总宽下限 90 tick(3ms)   */
#define SYNC_MAX      520      /* 同步总宽上限 520 tick(17ms) */
#define BIT_TICK_MIN  12
#define BIT_TICK_MAX  61

#define WIN_EXT_OF(lv)  ((lv) == 0 ? EXT_L0 : ((lv) == 1 ? EXT_L1 : EXT_L2))

u8  level      = 0;            /* 0/1/2 当前档 */
u16 win_ticks  = WIN_L0;       /* 当前窗口 */
u8  bit_ticks  = BIT_TK_L0;    /* 当前位宽基准 */

//============Define  Flag=================
typedef union {
   	unsigned char byte;
   	struct
   	{
   	   	u8 bit0 : 1;
   	   	u8 bit1 : 1;
   	   	u8 bit2 : 1;
   	   	u8 bit3 : 1;
   	   	u8 bit4 : 1;
   	   	u8 bit5 : 1;
   	   	u8 bit6 : 1;
   	   	u8 bit7 : 1;
   	} bits;
}bit_flag;

bit_flag flag1;

#define f_hl                flag1.bits.bit3     
#define f_getRfDataBits     flag1.bits.bit4     
#define f_int_rx            flag1.bits.bit5    
#define f_int_TMR0          flag1.bits.bit6  
#define f_synced            flag1.bits.bit7   

      
u8  RX_PCL 	   	= 0;          /* 0=空闲 1=解码中 */
u8  r_h_delay  	= 0;
u8  bit_count  	= 0;
u8  r_tol  	   	= 0;          /* 位宽一致容差(相对) */
u8  f_dataout  	= 0; 
u8  sys_state  	= ST_LISTEN;
u16 activity   	= 0;
u16 r_Timer1   	= 0; 
u16 r_l_delay  	= 0;   	   	  /* 同步低最长 470 tick */
u16 r_HLbuf    	= 0;



/************************************************
;  *    @函数名          : CLR_RAM
;  *    @说明            :
;  *    @输入参数        :
;  *    @返回参数        :
;  ***********************************************/
void CLR_RAM(void)
{
    for (FSR = 0xBF; FSR > 0x8F; FSR--)
    {
        INDF = 0;
    }
}

/************************************************
;  *    @函数名            : IO_Init
;  *    @说明              : 
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
/*  PIN1 VDD   PIN8 VSS
    PIN2 PT15  PIN7 PT10
    PIN3 PT14  PIN6 PT11
    PIN4 PT13  PIN5 PT12   */
void IO_Init(void)
{
    P1     = 0x2F;      /* P13=1 P15=1, P10/11/12=1 */
    DDR1   = 0x10;      /* 1:输入 0:输出 -> 只有 P14 是输入 */
    PUCON1 = 0xFF;     
    PDCON1 = 0xFF;      /* 下拉全禁 */
    ODCON1 = 0x00;      /* 推挽 */
}


/************************************************
;  *    @函数名            : Level_Init
;  *    @说明              : 片选读等级, PT11/PT12 内部上拉
;  *                        00=L0 10k~5k / 01=L1 4k ~ 2.86k / 10或11=L2  2.8k ~ 2k
;  ***********************************************/
void Level_Init(void)
{
    DDR1   |= (DEF_SET_BIT1 | DEF_SET_BIT2);      /* PT11/PT12 输入 */
    PUCON1 &= (u8)~(DEF_SET_BIT1 | DEF_SET_BIT2); /* 上拉有效 */
    Nop();
    Nop();
    Nop();

    if (P11D)            level = 2;
    else if (P12D)       level = 1;
    else                 level = 0;

   	level = 0; //测试
 
    if (level == 0)      { win_ticks = WIN_L0; bit_ticks = BIT_TK_L0; }
    else if (level == 1) { win_ticks = WIN_L1; bit_ticks = BIT_TK_L1; }
    else                 { win_ticks = WIN_L2; bit_ticks = BIT_TK_L2; }
}


/************************************************
;  *    @函数名            : TIMER0_LP_Init
;  *    @说明              : 定时器T0
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void TIMER0_LP_Init(void)
{
    TMRCR |= 0x40;           /* T0 时钟源 = 低频 FLIRC 32kHz */
    T0CR  = 0x05;           /* ÷64 预分频 + T0PTA=0(预分频给T0, WDT 1:1) */
    T0CNT = 256 - 100;       /* 100 计数 x 2ms = 200ms */
    T0IE  = 1;
}



/************************************************
;  *    @函数名            : TIMER1_INT_Init
;  *    @说明              : 定时器T1
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void TIMER1_INT_Init(void)
{
    T1CR   = 0x02;      /* CPU 时钟, /8  (8MHz/8 = 1MHz) */
    T1CNT  = 32;        /* 33 计数 -> 33us */
    T1LOAD = 32;
    T1EN   = 1;
    T1IE   = 1;
}


/************************************************
;  *    @函数名            : Clk_High_Init
;  *    @说明              : 唤醒后恢复高频:STOP 会停掉 16MHz 高频时钟, 
;  *   	   	   	   	   	   	 T0(FLIRC)唤醒后 CPU还在低频，必须把高频开回来、再重配 T1。
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void Clk_High_Init(void)
{
    u16 t = 0;

    HFEN = 1;               /* 使能 16MHz 高频 RC */

    while (!STBH)
    {
        ClrWdt();
        if (++t > 2000)     /* 起振超时: 停止喂狗, 由WDT复位, 避免死等 */
        {
            while (1) { }
        }
    }
    CLKS = 0;               /* 系统时钟切回高频 FCPU=8MHz */
}

/************************************************
;  *    @函数名            : Sys_Init
;  *    @说明              : 系统初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void Sys_Init(void)
{
    GIE    	   	= 0;

    RX_PCL 	   	= 0;
    r_h_delay  	= 0;
    r_l_delay  	= 0;
    bit_count  	= 0;
    r_HLbuf    	= 0;
    r_tol  	   	= 0;
    r_Timer1   	= 0;
    flag1.byte 	= 0;
    f_dataout  	= 0;
   	activity   	= 0;

    CLR_RAM();
    IO_Init();
   	Level_Init(); 
   	TIMER0_LP_Init();
    TIMER1_INT_Init();

    GIE = 1;
}

/* ======================== RFRX================ */
static void RX_START(void)
{
    if (io_rfdata)                  /* 高: 开始解码 */
    {
        r_l_delay = 0;
        r_h_delay = 1;
        f_hl = 0;
        f_getRfDataBits = 0;
        bit_count = 0;
        RX_PCL++;
    }
    else                            /* 低: 空闲时也累计, 供延长判断 */
    {
        if (r_l_delay < 600) r_l_delay++;
    }
}

/************************************************
;  *    @函数名            : RX_RESET
;  *    @说明              : 每个监听窗口开始前复位解码器
;  ***********************************************/
static void RX_RESET(void)
{
    RX_PCL = 0;
    r_h_delay = 0;
    r_l_delay = 0;
    f_hl = 0;
    f_getRfDataBits = 0;
    f_synced = 0;
    bit_count = 0;
}

/********************************************************************
* Function Name  : RX_DATA_HL.
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
********************************************************************/
static void RX_DATA_HL(void)
{
    u16 hl, t, d;

    if (io_rfdata)                  /* 高电平 */
    {
        if (f_hl)                   /* 低->高: 结算上一个电平对 */
        {
            f_hl = 0;
            f_getRfDataBits = 1;
        }
        else
        {
            r_h_delay++;
            if (r_h_delay > LEVEL_TOUT) { RX_PCL = 0; return; }
        }
    }
    else                            /* 低电平: 只计数, 不结算 */
    {
        f_hl = 1;
        if (r_l_delay < 600) r_l_delay++;
        return;
    }

    if (!f_getRfDataBits) return;
    f_getRfDataBits = 0;

    hl = (u16)r_h_delay + r_l_delay;

    /* ---------- 1. 同步头: 长低 + 短高 ---------- */
    if (r_l_delay > LEVEL_TOUT)
    {
        /* 低至少是高的 8 倍(数据位最大 3:1), 总宽 90~520 tick */
        if ((r_l_delay >= ((u16)r_h_delay << 3)) && (hl >= SYNC_MIN) && (hl <= SYNC_MAX))
        {
            bit_ticks = (u8)(hl >> 3);      /* 128α/8 = 16α */
            if (bit_ticks < BIT_TICK_MIN) bit_ticks = BIT_TICK_MIN;
            if (bit_ticks > BIT_TICK_MAX) bit_ticks = BIT_TICK_MAX;
            f_synced = 1;
        }
        bit_count = 0;
        r_l_delay = 0;
        r_h_delay = 1;
        return;
    }

    /* ---------- 2. 数据位: 先看总宽 ---------- */
    t = (u16)bit_ticks;
    if (f_synced)
    {
        if (hl < (t - (t >> 2)) || hl > (t + (t >> 2))) { RX_PCL = 0; return; }
    }
    else    /* 没抓到同步: 用等级基准放宽到 ±37.5% */
    {
        if (hl < (t - (t >> 2) - (t >> 3))) { RX_PCL = 0; return; }
        if (hl > (t + (t >> 2) + (t >> 3))) { RX_PCL = 0; return; }
    }

    /* ---------- 3. 判 0/1: 比例 + 幅值 ---------- */
    if (r_h_delay > r_l_delay)                  /* 高>低 -> 1 */
    {
        if (r_h_delay < (r_l_delay << 1)) { RX_PCL = 0; return; }
    }
    else if (r_h_delay < r_l_delay)             /* 高<低 -> 0 */
    {
        if (r_l_delay < ((u16)r_h_delay << 1)) { RX_PCL = 0; return; }
    }
    else { RX_PCL = 0; return; }

    /* ---------- 4. 帧内一致性 ---------- */
    if (bit_count == 0)
    {
        r_HLbuf = hl;
    }
    else
    {
        r_tol = (u8)(r_HLbuf >> 2);
        if (r_tol < 2) r_tol = 2;
        if (hl > r_HLbuf) d = hl - r_HLbuf;
        else              d = r_HLbuf - hl;
        if (d > r_tol) { RX_PCL = 0; return; }
    }

    r_l_delay = 0;
    r_h_delay = 1;
    bit_count++;

    if (bit_count >= VALID_BITS)
    {
        f_dataout = 1;
        activity  = (u16)r_HLbuf * 32;
    }
}

/********************************************************************
* Function Name  : IR_Rx_fun.
* Description    : RF 接收
* Input          : None.
* Output         : None.
* Return         : None.
********************************************************************/

void IR_Rx_fun(void)
{
    if (f_int_rx == 0)
    {
        return;
    }

    f_int_rx = 0;

    switch (RX_PCL)
    {
        case 0: RX_START();     break;
        case 1: RX_DATA_HL();   break;
        default: RX_PCL = 0;    break;
    }
}

/************************************************
;  *    @函数名            : main
;  *    @说明              : 主程序
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void main(void)
{
    Sys_Init();

     while (1)
    {
        switch (sys_state)
        {
        /* ---- 监听: 开接收, 等 1ms 启动, 听 10ms ---- */
        case ST_LISTEN:
            io_RxEnable = 0;              /* 接收开 */
            RX_RESET();
            r_Timer1 = 0;
            while (r_Timer1 < 30)         /* 30×33us ≈ 1ms 启动 */
            {
                ClrWdt();
            }

            r_Timer1 = 0;
            while (r_Timer1 < win_ticks)  /* 基础窗口 5/12/18ms */
            {
                ClrWdt();
                IR_Rx_fun();
                if (f_dataout)
                {
                    sys_state = ST_FORWARD;
                    break;
                }
            }

             /* 动态延长: 卡在同步里, 或已收到几位但不够 4 位 */
            if ((sys_state != ST_FORWARD) &&
                ((r_l_delay > LEVEL_TOUT) ||
                 ((bit_count > 0) && (bit_count < VALID_BITS))))
            {
                while (r_Timer1 < WIN_EXT_OF(level))
                {
                    ClrWdt();
                    IR_Rx_fun();
                    if (f_dataout)
                    {
                        sys_state = ST_FORWARD;
                        break;
                    }
                }
            }

            if (sys_state != ST_FORWARD)
                sys_state = ST_SLEEP;
            break;

        /* ---- 转发: 持续解码, 转发在中断里做 ---- */
        case ST_FORWARD:
            ClrWdt();
            IR_Rx_fun();

             if (!f_dataout)
            {
                io_OUT = 0;          
                sys_state = ST_SLEEP;
            }
            break;

        /* ---- 休眠 200ms: 关接收, T0 唤醒 ---- */
        case ST_SLEEP:
            io_OUT = 0;                   /* 输出高 */
            io_RxEnable = 1;              /* 接收关 */

            T0CNT = 256 - 125;            /* 重装 200ms (100 x 2ms) */

            T0IF  = 0;
            T0IE  = 1;
            HFEN  = 0;

            Stop();                       /* 睡 200ms -> T0 中断唤醒 */
            Nop();
            Nop();

            ClrWdt();                     /* 醒来先清狗*/
            T0IE  = 0;

            /* 唤醒后: 恢复高频 + 重配 T1*/
            Clk_High_Init();
            TIMER1_INT_Init();

            sys_state = ST_LISTEN;
            break;
        }
    }
}

/************************************************
;  *    @函数名            : interrupt
;  *    @说明              : 中断函数
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void int_isr(void) __interrupt
{
    __asm
    movra   _abuf
    swapar  _STATUS
    movra   _statusbuf
__endasm;

   	if ((T0IF) && (T0IE))
    {
        T0IF = 0;    
    }
    if ((T1IF) && (T1IE))
    {
        T1IF = 0;
   	   	T1IF = 0;
   	   	f_int_rx = 1;

   	   	r_Timer1++;   
        //io_dbg = !io_dbg;  

   	   	if (f_dataout)// 转发: PT13 跟随 PT14 
        {
            io_OUT = io_rfdata;     
        }

   	   	/* 超时清零：信号消失约一帧后关掉转发 */
   	   	if (f_dataout)
        {
            if (activity)
            {
                activity--;
                if (activity == 0)
                {
                    f_dataout = 0;
                }
            }
        }
        
    }

    __asm
    swapar  _statusbuf
    movra   _STATUS
    swapr   _abuf
    swapar  _abuf
__endasm;
}

/**************************** end of file ************************************/
