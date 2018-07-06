
/*---------------------------------------------------------------------*/
/* --- STC MCU International Limited ----------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: 13922805190 --------------------------------------------*/
/* --- Fax: 0513-55012956,55012947,55012969 ---------------------------*/
/* --- Tel: 0513-55012928,55012929,55012966 ---------------------------*/
/* --- Web: www.GXWMCU.com   www.stcmcu.com ---------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了宏晶科技的资料及程序   */
/*---------------------------------------------------------------------*/


/*************	功能说明	**************

本程序试验使用STC15W401AS-35I-SOP16<RMB1.6>来驱动航模用的无传感器无刷三相直流马达.

本程序参考自网上的代码(作者: 瑞生), 改良而来.

电路图见文件 "BLDC-V10-实验电路.pdf".

控制信号由P3.2输入正脉冲信号, 间隔5~20ms, 脉冲宽度1.000~1.610ms.

1.160ms开始启动, 1.610ms为最高速度, 分辨率为2us.

本程序仅仅是简单控制, 软件没有处理 过0延时30度切换 过流检测.

由于过0检测部分有RC滤波, 所以改变电容值可以大约的对应在最高速时延时30度的时间.

有意者可自行完善电路和程序.

******************************************/




#define MAIN_Fosc		24000000L	//定义主时钟

#include "STC15Fxxxx.H"

#define		MCU_PIN		16	/* 选择MCU引脚数, 只支持16或20脚(28脚或32脚跟20脚一样) */

// #include "reg51.h"
#include "intrins.h"

typedef unsigned char BYTE;
typedef unsigned int WORD;
void SendData(BYTE dat);
void SendString(char *s);
// #define FOSC 11059200L          //系统频率
// #define BAUD 115200             //串口波特率

#define NONE_PARITY     0       //无校验
#define ODD_PARITY      1       //奇校验
#define EVEN_PARITY     2       //偶校验
#define MARK_PARITY     3       //标记校验
#define SPACE_PARITY    4       //空白校验

#define PARITYBIT NONE_PARITY   //定义校验位



//CMPCR1
#define	CMPEN	0x80	//1: 允许比较器, 0: 禁止,关闭比较器电源
#define	CMPIF	0x40	//比较器中断标志, 包括上升沿或下降沿中断, 软件清0
#define	PIE		0x20	//1: 比较结果由0变1, 产生上升沿中断
#define	NIE		0x10	//1: 比较结果由1变0, 产生下降沿中断
#define	PIS		0x08	//输入正极性选择, 0: 选择外部P5.5做正输入,           1: 由ADCIS[2:0]所选择的ADC输入端做正输入.
#define	NIS		0x04	//输入负极性选择, 0: 选择内部BandGap电压BGv做负输入, 1: 选择外部P5.4做输入.
#define	CMPOE	0x02	//1: 允许比较结果输出到P1.2, 0: 禁止.
#define	CMPRES	0x01	//比较结果, 1: CMP+电平高于CMP-,  0: CMP+电平低于CMP-,  只读

//CMPCR2
#define	INVCMPO	0x80	//1: 比较器输出取反,  0: 不取反
#define	DISFLT	0x40	//1: 关闭0.1uF滤波,   0: 允许
#define	LCDTY	0x00	//0~63, 比较结果变化延时周期数

#if	(MCU_PIN == 20)
	sbit PWM2_L = P3^4;
	sbit PWM1_L = P3^5;
	sbit PWM0_L = P3^6;
#endif

#if	(MCU_PIN == 16)
	sbit PWM2_L = P5^5;		// C-
	sbit PWM1_L = P3^3;		// B-
	sbit PWM0_L = P3^6;		// A-
#endif

u8	Step;
u8	PWM_Value; // 决定PWM占空比的值
u16	RxPulseWide;
bit	B_RxOk;
bit	B_RUN;
u8	PWW_Set;
u8	cnt10ms;
u8	Rx_cnt;
u8	TimeOut;	//堵转超时

#define DISABLE_CMP_INT CMPCR1 &= ~0X40		// 关闭比较器中断
#define ENABLE_CMP_INT  CMPCR1 |= 0X40		// 打开比较器中断

/*************************/

void	Delay_n_ms(u8 dly)
{
	u16	j;
	do
	{
		j = MAIN_Fosc / 13000;	//延时1ms, 主程序在此节拍下运行
		while(--j)	;
	}while(--dly);
}


void delay_us(u8 us)
{
	do
	{
		NOP(20);	//@24MHz
	}
	while(--us);
}

void StepXL(void) // 换相序列函数
{
 switch(Step)
  {
   case 0:  // AB
			PWM0_L=0;	PWM2_L=0;
			PWM0_OUT_1();
			PWM1_OUT_1();
			PWM2_OUT_1();
			PWM0_NORMAL();
			CCAP0H = PWM_Value;		// 打开A相的高端
			PWM1_L = 1; 		// 打开B相的低端
			ADC_CONTR = 0XED;	// 选择P1.5作为ADC输入 即c相电压
			CMPCR1 = 0x9C;		//bit7=1 允许比较器, bit4=1 比较结果由1变0, 产生下降沿中断 (不能响应下降沿中断?)
		break;
   case 1:  // AC
			PWM0_L=0;	PWM1_L=0;
			PWM0_OUT_1();
			PWM1_OUT_1();
			PWM2_OUT_1();
			PWM0_NORMAL();
			CCAP0H = PWM_Value;		// 打开A相的高端
			PWM2_L = 1;			// 打开C相的低端
			ADC_CONTR = 0XEC;	// 选择P1.4作为ADC输入 即B相电压
			CMPCR1 = 0xAC;		//上升沿中断
	 
      	break;
   case 2:  // BC
			PWM0_L=0;	PWM1_L=0;
			PWM0_OUT_1();
			PWM1_OUT_1();
			PWM2_OUT_1();
			PWM1_NORMAL();
			CCAP1H = PWM_Value; // 打开B相的高端
			PWM2_L = 1;			// 打开C相的低端
			ADC_CONTR = 0XEB;	// 选择P1.3作为ADC输入 即a相电压
			CMPCR1 = 0x9C;		//下降沿中断
      	break;
   case 3:  // BA
			PWM1_L=0;	PWM2_L=0;
			PWM0_OUT_1();
			PWM1_OUT_1();
			PWM2_OUT_1();
			PWM1_NORMAL();
			CCAP1H = PWM_Value; // 打开B相的高端
			PWM0_L = 1;			// 打开A相的低端
			ADC_CONTR = 0XED;	// 选择P1.5作为ADC输入 即c相电压 
			CMPCR1 = 0xAC;		//上升沿中断
			
      	break;
   case 4: // CA
			PWM1_L=0;	PWM2_L=0;
			PWM0_OUT_1();
			PWM1_OUT_1();
			PWM2_OUT_1();
			PWM2_NORMAL();
			CCAP2H = PWM_Value; // 打开C相的高端
			PWM0_L = 1;			// 打开A相的低端
			ADC_CONTR = 0XEC;	// 选择P1.4作为ADC输入 即B相电压
			CMPCR1 = 0x9C;		//下降沿中断
     	break;
   case 5: // CB
      		PWM0_L=0;	PWM2_L=0;
			PWM0_OUT_1();
			PWM1_OUT_1();
			PWM2_OUT_1();
			PWM2_NORMAL();
			CCAP2H = PWM_Value; // 打开C相的高端
      		PWM1_L = 1;			// 打开B相的低端
			ADC_CONTR = 0XEB;	// 选择P1.3作为ADC输入 即a相电压
			CMPCR1 = 0xAC;		//上升沿中断
	 
		break;
	 
	default:
		break;
  }	
}



void PWM_Init(void)
{
	PWM0_L = 0;
	PWM1_L = 0;
	PWM2_L = 0;
	
	#if	(MCU_PIN == 20)
		P3n_push_pull(0x70);
	#endif
	#if	(MCU_PIN == 16)
		P3n_push_pull(0x48);
		P5n_push_pull(0x20);
	#endif

	// CMOD = 1 << 1; //选择系统时钟/2为时钟源，即PWM频率=24M/2/256=46.9K
	CMOD = 5 << 1; //选择系统时钟/4为时钟源，即PWM频率=24M/4/256=23.4K
	// CMOD = 6 << 1; //选择系统时钟/6为时钟源，即PWM频率=24M/6/256=15.6K
	CL=0;			// PCA计数器清零
	CH=0;
	
	PCA_PWM0 = 0X00;
	CCAP0H=0;    // 初始化占空比为0% H的值装载到L中
	CCAP0L=0;
	CCAPM0=0x42;	// 设置为PWM模式, 8位，无中断。

	PCA_PWM1 = 0X00;
	CCAP1H=0;    // 初始化占空比为0%
	CCAP1L=0;
	CCAPM1=0x42;	// 设置为PWM模式
	
	PCA_PWM2 = 0X00;
	CCAP2H=0;    // 初始化占空比为0%
	CCAP2L=0;
	CCAPM2=0x42;	// 设置为PWM模式
	
	PWM0_OUT_1();
	PWM1_OUT_1();
	PWM2_OUT_1();

	CR = 1;
}

void ADC_Init(void)
{
	P1n_pure_input(0x38);
	P1ASF = 0X38; // 开通P1.3 P1.4 P1.5的AD输入口
}

void CMP_INT(void) interrupt 21
{
	CMPCR1 &= ~0X40; // 需软件清除中断标志位
	if(Step<5)	Step++;
	else		Step = 0;
	StepXL();
	TimeOut = 10;	//10ms超时
}

void CMP_Init(void)
{
	CMPCR1 = 0X8C;	// 1000 1100 打开比较器，P5.4作为比较器的反相输入端，ADC引脚作为正输入端 
	CMPCR2 = 60;	// 60个时钟滤波
	P5n_pure_input(0x10);
}

u8 StartMotor(void)
{
	u16 timer,i;
	DISABLE_CMP_INT;	// 禁止比较器中断
	PWM_Value = 30;		// 初始占空比=16/256=6%
	Step = 0;
	StepXL();			// 初始位置
	Delay_n_ms(5);//delay_ms(5);
	timer = 300;

	while(1)
	{
		for(i=0; i<timer; i++)	delay_us(50);  // 第一次是15毫秒，加上上面的5毫米延时，总计大约20毫秒
		timer -= timer /15 + 1;
		if(timer < 25)	{
			CMPCR1 &= ~0X40; 	// 需软件清除中断标志位
			B_RUN = 1;			// 马达运行标志位
			PWM_Value = 0;		// 启动完成。停转，等串口的PWW_SET值
			TimeOut = 10;		// 10毫米超时
			ENABLE_CMP_INT; 	// 打开比较器中断
			return(1);
		}
		if( Step < 5)	Step++;
		else			Step = 0;
		StepXL();
		// SendString("Start Motor!\r\n");
	}
}

// void T0_Iint(void)
// {
// 	Timer0_AsTimer();	/* 时器0用做定时器	*/
// 	Timer0_12T();		/* Timer0 clodk = fo/12	12分频,	default	*/
// 	Timer0_16bit();
// 	Timer0_Gate_INT0_P32();	/* 时器0由外部INT0高电平允许定时计数 */
// 	TH0 = 0;
// 	TL0 = 0;
// 	TR0 = 1; // 打开定时器0
// 	ET0 = 1;// 允许ET0中断
// }

// void T0_Interrupt(void) interrupt 1
// {
// 	Rx_cnt = 0;			//一旦出现溢出, 则开始的n个脉冲无效
// 	RxPulseWide = 1000;	//停止
// 	B_RxOk = 1;			//虚拟收到一个脉冲
// }

/********************* INT0中断函数 *************************/
// void INT0_int (void) interrupt INT0_VECTOR
// {
// 	u16	j;
	
// 	TR0 = 0;
// 	j = ((u16)TH0 << 8) + TL0;
// 	TH0 = 0;
// 	TL0 = 0;
// 	TR0 = 1;

// 	if(++Rx_cnt >= 5)	Rx_cnt = 5;
// 	j >>= 1;	//为了好处理, 转成单位为us
// 	if((j >= 800) && (j <= 2000) && (Rx_cnt == 5))
// 	{
// 		RxPulseWide = j;
// 		B_RxOk = 1;		//标志收到一个脉冲
// 	}

// }

/********************* 串口初始化函数 *************************/
bit busy;


void UartInit(void)		//9600bps@24.000MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0x8F;		//设定定时初值
	T2H = 0xFD;		//设定定时初值
	AUXR |= 0x10;		//启动定时器2
    ES = 1;                     //使能串口1中断
}


/*----------------------------
UART 中断服务程序
-----------------------------*/
void Uart() interrupt 4 using 1
{
    ES = 0;         //关闭串口中断
	if (RI)
    {
		PWW_Set = SBUF; // 把收到的数据给了PWW_Set
		RI=0; // 串口接收标志清0
		B_RxOk = 1;
    }
    if (TI)
    {
        TI = 0;                 //清除TI位
        busy = 0;               //清忙标志
    }
	ES = 1;         //使能串口中断
}


/*----------------------------
发送串口数据
----------------------------*/
void SendData(BYTE dat)
{
    while (busy);               //等待前面的数据发送完成
    ACC = dat;                  //获取校验位P (PSW.0)
    if (P)                      //根据P来设置校验位
    {
		#if (PARITYBIT == ODD_PARITY)
				TB8 = 0;                //设置校验位为0
		#elif (PARITYBIT == EVEN_PARITY)
				TB8 = 1;                //设置校验位为1
		#endif
    }
    else
    {
		#if (PARITYBIT == ODD_PARITY)
				TB8 = 1;                //设置校验位为1
		#elif (PARITYBIT == EVEN_PARITY)
				TB8 = 0;                //设置校验位为0
		#endif
    }
    busy = 1;
    SBUF = ACC;                 //写数据到UART数据寄存器
}

/*----------------------------
发送字符串
----------------------------*/
void SendString(char *s)
{
    while (*s)                  //检测字符串结束标志
    {
        SendData(*s++);         //发送当前字符
    }
}


/**********************************************/

void main(void)
{
	PWM_Init();
	ADC_Init();
	CMP_Init();
	UartInit();
	// T0_Iint();

	IE0 = 0;	// 清除外中断0标志位
	// EX0 = 1;	// INT0 Enable
	// IT0 = 1;	//INT0 下降沿中断
	
	// RxPulseWide = 1000;
	PWW_Set = 0;
	cnt10ms = 0;
	Rx_cnt  = 0;
	TimeOut = 0;

	EA = 1; // 打开总中断
	// Delay_n_ms(1);	//延时
	SendString("BLDC Test !\r\n");
	while (1)
	{
		Delay_n_ms(1);	//延时1ms, 主程序在此节拍下运行

		if (TimeOut > 0)
		{
			if(--TimeOut == 0)	//堵转超时
			{
				DISABLE_CMP_INT; 	// 关比较器中断
				CCAP0H=0;	CCAP1H=0;	CCAP2H=0;  // 占空比为0
				PWM0_L=0;	PWM1_L=0;	PWM2_L=0;
				// RxPulseWide = 1000;
				PWW_Set   = 0;
				PWM_Value = 0;
				B_RxOk = 0;
				B_RUN  = 0;
				Rx_cnt = 0;
				TimeOut = 0;
				SendString("Time Out!\r\n");

				Delay_n_ms(250);	//堵转时,延时1秒再启动
				Delay_n_ms(250);
				Delay_n_ms(250);
				Delay_n_ms(250);

			}
		}
		
		// if(B_RxOk)	//收到一个脉冲
		// {
		// 	B_RxOk = 0;
		// 	j = RxPulseWide;
		// 	if(j >= 1100)				// 1100~1610对应PWM占空比值0~255
		// 	{
		// 		j = (j - 1100) >> 1;	//2us对应PWM一个步进
		// 		if(j > 256)	j = 255;
		// 	}
		// 	else	j = 0;
		// 	PWW_Set = (u8)j;
		// }
		// SendData(PWW_Set);
		if(B_RxOk) // 如果串口收到数据
		{
			// SendData(PWW_Set);
			B_RxOk = 0;
			// if(rec==0x22)// 加速命令
			// {
			// 	if(PWM_Value<250)
			// 	{
			// 		PWM_Value++; // 增加占空比
			// 	}
			// }
			// else if(rec==0x33)// 减速命令
			// {
			// 	if(PWM_Value>30)
			// 	{
			// 		PWM_Value--; // 减小占空比
			// 	}
			// }
			// else if(rec==0x11) // 启动命令
			// {
			// 	StartMotor();	// 启动马达
			// 	CMPCR1 &= ~0X40; // 需软件清除中断标志位
			// 	ENABLE_CMP_INT; // 打开比较器中断
			// 	B_RUN = 1;
			// 	TimeOut = 0;
			// }
			// else if(rec==0x44) // 停止命令
			// {
			// 	CCAP0H=0;CCAP1H=0;CCAP2H=0;  // 占空比都置0
			// 	EA = 0; // 关闭全局中断
			// 	DISABLE_CMP_INT; // 关闭比较器中断
			// }
		}
		if(!B_RUN && (PWW_Set >= 30))		// PWM_Set >= 30, 并且马达未运行, 则启动马达
		{
			StartMotor();	// 启动马达
			// TimeOut = 0;
		}
		
		
		if(++cnt10ms >= 10)		// 10ms时隙
		{
			// SendData(cnt10ms);
			cnt10ms = 0;
			if(B_RUN)
			{
				if(PWM_Value < PWW_Set)	PWM_Value++;
				if(PWM_Value > PWW_Set)	PWM_Value--;
				if(PWM_Value < 20)	// 停转
				{
					PWM_Value = 0;
					B_RUN = 0;
					CCAP0H=0;	CCAP1H=0;	CCAP2H=0;  // 占空比为0
					PWM0_L=0;	PWM1_L=0;	PWM2_L=0;
					DISABLE_CMP_INT; // 关比较器中断
					SendString("Stop Motor!\r\n");
				}
			}
		}
	}
}




