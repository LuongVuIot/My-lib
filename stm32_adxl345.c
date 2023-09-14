#include "main.h"
#include <math.h>
#include <stdio.h>


#define PORTA	(PORTx_*)0x40010800U
#define PORTB (PORTx_*)0x40010C00U
#define PORTC (PORTx_*)0x40011000U
#define PORTD (PORTx_*)0x40011400U
#define PORTE (PORTx_*)0x40011800U
#define PORTF (PORTx_*)0x40011C00U
#define PORTG (PORTx_*)0x40012000U

#define SYSTICK_ID  							(Sys_tick*)0xE000E010U
#define Set_Handler_pri_ID				(Set_Handler_pri*)0xE000ED18U
#define ISR_Systick_ID						0x0840003CU
#define RCC_ID 										(RCC_*)0x40021000U

#define ICTR_ID				(ICTR_*)0xE000E004U
#define NVIC_ISER_ID	(NVIC_ISER*)0xE000E100U
#define NVIC_ICER_ID	(NVIC_ICER*)0xE000E180U
#define NVIC_ISPR_ID	(NVIC_ISPR*)0xE000E200U
#define NVIC_ICPR_ID	(NVIC_ICPR*)0xE000E280U
#define NVIC_IABR_ID	(NVIC_IABR*)0xE000E300U
#define	NVIC_IPR_ID		(NVIC_IPR*)0xE000E400U
#define STIR_ID				(STIR_*)0xE000EF00U

#define EXTI_ID	(EXTI_*)0x40010400U
#define AFIO_ID (AFIO_*)0x40010000U

#define ADC1_ID	(ADCx_*)0x40012400U
#define ADC2_ID (ADCx_*)0x40012800U
#define ADC3_ID (ADCx_*)0x40013C00U

#define I2C1_ID (I2Cx_*)0x40005400U
#define I2C2_ID (I2Cx_*)0x40005800U

volatile unsigned int count = 0;
uint16_t adc_value;
volatile short	t = 0;
volatile short x, x0;
volatile short y, y0;
volatile short	z, z0;
volatile float k;
volatile float KG;
volatile float	EstError = 4;		// for kalman filter
volatile float Est	= 255;
volatile short EstC;
float R_measure ;

volatile float K = 0 , a = 0, P = 65025, j = 0, s = 0, r = 100, Q = 0.01;		// K:kalman gain - a:Estimate accel value - P: Error in Estimate accel \
																																							 j:Estimate jerk value - s:Estimate snap value - r:Covarience measurment \
																																							 q:Covarient process

unsigned int time, ms;		// for calculate time of excution

unsigned short	ok = 0, up = 0, down = 0, m = 0, degree = 50, ac = 10;		// ok-up-down button; m - Acel_devia_detect or Degree_detect

uint8_t adxl[2] = {0x2D,0x08};				// into measure mode
// uint8_t adxl0[2] = {0x2D,0x00};		// reset register first ?
uint8_t arr[4];
uint8_t data[] = {0x32};							// begin read data gravity from 0x32 address.
uint8_t buf[6];

short AccelInitValue[3];			// Acceleration Initial Value - Gia tri gia toc ban dau tren 3 truc
short AngleInitValue[3];			// Angle Initial Value - Gia tri goc ban dau tren 3 truc
char data_gravity[11] = {' ',0,'.',0,0,0,0,0,0,0,0};				// khoang trong ' ' danh cho signed // g unit
char data_degree[11] = {' ',0,0,0xDF,0,0,0x27,0,0,0x27,0x27}; // degrees unit
char data_accel[14] = {' ',0,0,'.',0,0,0,0,0,' ','m','/','s',0};		// m/s2 unit; 0 at last is 0x00 address of CGRAM.
const char mu2[] = {0x0c, 0x12, 0x04, 0x08, 0x1e,0x00, 0x00, 0x00};	// ky tu mu 2
const char ar[] = {0x06, 0x02, 0x0c, 0x12, 0x12, 0x12, 0x0f, 0x00}; // // 0x01 address CGRAM
const char as[] = {0x02,0x04,0x00,0x0C,0x12,0x12,0x0F,0x00};		// add CGRAM 2
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_init (void);
void lcd_send_string (char *str);

typedef enum { PLLOFF = 0U, PLLON } pll_st;

typedef enum { HSI = 0U, HSE = 16U } osc_source;

typedef enum pin_st { ON = 0U, OFF } pin_st;								// Reset/Set Pinx

typedef enum clk_st { DIS = 0u, EN } clk_st;								// Enable/disable Portx clock

typedef enum port_mode { INPUT = 0u, OUTPUT} port_mode;			// OUTPUT max speed = 2Mhz

typedef enum io_mode { ANALOG = 0u, FLOAT, PULLUP = 10u, PULLDOWN = 12u,
											OUTPUSH = 0u, OUTDRAIN, AFPUSH = 10u, AFDRAIN } io_mode;
typedef enum pinx { PORT0 = 0u, PORT1, PORT2, PORT3, PORT4, PORT5, PORT6, PORT7,
										PORT8, PORT9, PORT10, PORT11, PORT12, PORT13, PORT14, PORT15, ALL_PORT } pinx;
typedef enum handler { MemMage = 0u, BusFaults, UsageFaults, SVCall, PendSV, SysTick_exception } handler;

typedef enum EXTI_line { EXTI0 = 0u, EXTI1, EXTI2, EXTI3, EXTI4, EXTI5, EXTI6, EXTI7, EXTI8, EXTI9,
												 EXTI_10, EXTI_11, EXTI_12, EXTI_13, EXTI14, EXTI15} EXTI_line;
typedef enum EXTI_egle { RISING = 1u, FALLING = 2u, BOTH = 3u} EXTI_egle;		// BOTH will set RISING and FALLING simutaneous
typedef enum adc_mode { SINGLE, CONTINUOUS } adc_mode;

void set_devia();

typedef struct{
		unsigned int RCC_RC;
		unsigned int RCC_CFGR;
		unsigned int RCC_CIR;
		unsigned int RCC_APB2RSTR;
		unsigned int RCC_APB1RSTR;
		unsigned int RCC_AHBENR;
		unsigned int RCC_APB2ENR;
		unsigned int RCC_APB1ENR;
		unsigned int RCC_BDCRS;
		unsigned int RCC_CSR;
}RCC_;		RCC_* RCC_reg = RCC_ID;

typedef struct{
	unsigned int PORTx_CRL;
	unsigned int PORTx_CRH;
	unsigned int PORTx_IDR;
	unsigned int PORTx_ODR;
	unsigned int PORTx_BSRR;
	unsigned int PORTx_BRR;
	unsigned int PORTx_LCKR;
}PORTx_;

typedef struct{
	volatile unsigned int STCSR;			// Systick control and status register
	volatile unsigned int STRVR;			// Systick reload value register
	volatile unsigned int STCVR;			// Systick current value register
	volatile unsigned int STCR;			 // Systick cabibration value register
}Sys_tick;		Sys_tick* Systimer = SYSTICK_ID;

typedef struct{
	unsigned int SHPR1;
	unsigned int SHPR2;
	unsigned int SHPR3;
}Set_Handler_pri;		Set_Handler_pri* HandlerPri = Set_Handler_pri_ID;

typedef struct{	unsigned int ICTR; }	ICTR_;					// interrupt controller type register 
typedef struct{ unsigned int ISER[8]; }NVIC_ISER;			// interrupt set-enable register
typedef struct{	unsigned int ICER[8];	}NVIC_ICER;			// interrupt clear enable register
typedef struct{	unsigned int ISPR[8];	}NVIC_ISPR;			// interrupt set pending register
typedef struct{ unsigned int ICPR[8]; }NVIC_ICPR;			// interrupt clear pending register
typedef struct{	unsigned int IABR[8];	}NVIC_IABR;			// interrupt active bit register
typedef struct{	unsigned int IPR[60];	}NVIC_IPR;			// interrupt priority register
typedef struct{	unsigned int STIR; }	STIR_;					// software trigger interrupt register 

ICTR_* 		 ICTR_REG = ICTR_ID;
NVIC_ISER* ISER_REG = NVIC_ISER_ID;
NVIC_ICER* ICER_REG = NVIC_ICER_ID;
NVIC_ISPR* ISPR_REG = NVIC_ISPR_ID;
NVIC_ICPR* ICPR_REG = NVIC_ICPR_ID;
NVIC_IABR* IABPR_REG = NVIC_IABR_ID;
NVIC_IPR*	 IPR_REG  = NVIC_IPR_ID;
STIR_* 		 STIR_REG = STIR_ID;

typedef struct{
	unsigned int EVCR;
	unsigned int MAPR;
	unsigned int EXTICR[4];
}AFIO_;		AFIO_* AFIO_REG = AFIO_ID;

typedef struct{
	unsigned int IMR;
	unsigned int EMR;
	unsigned int RTSR;
	unsigned int FTSR;
	unsigned int SWIER;
	unsigned int PR;
}EXTI_;		EXTI_* EXTI_REG = EXTI_ID;

typedef struct{
	unsigned int SR;
	unsigned int CR1;
	unsigned int CR2;
	unsigned int SMPR1;
	unsigned int SMPR2;
	unsigned int SMPR3;
	unsigned int JOFR1;
	unsigned int JOFR2;
	unsigned int JOFR3;
	unsigned int JOFR4;
	unsigned int HTR;
	unsigned int LTR;
	unsigned int SQR1;
	unsigned int SQR2;
	unsigned int SQR3;
	unsigned int JSQR;
	unsigned int JDR1;
	unsigned int JDR2;
	unsigned int JDR3;
	unsigned int JDR4;
	unsigned int DR;
}ADCx_;

ADCx_* ADC_1 = ADC1_ID;
ADCx_* ADC_2 = ADC2_ID;
ADCx_* ADC_3 = ADC3_ID;

typedef struct{
	unsigned int CR1;
	unsigned int CR2;
	unsigned int OAR1;
	unsigned int OAR2;
	unsigned int DR;
	unsigned int SR1;
	unsigned int SR2;
	unsigned int CCR;
	unsigned int TRISE;
}I2Cx_;

I2Cx_* I2C1_REG = I2C1_ID;
I2Cx_* I2C2_REG = I2C2_ID;

void write_bit( unsigned int reg, unsigned char bit, unsigned char value)
{
	*(unsigned int*)(0x42000000 + ((unsigned int)&reg - 0x40000000)*32 + bit*4) = value;
}
unsigned int read_bit(unsigned int reg, unsigned char bit)
{
	return *(unsigned int*)(0x42000000 + ((unsigned int)&reg - 0x40000000)*32 + bit*4);
}
void delay_systick(unsigned int n)
{
	unsigned int i;
	Systimer->STCSR = 5;		// enable counter, choose system clock for systick
	for(i=0; i<n; i++)
		while((Systimer->STCSR & 0x10000) == 0);
	Systimer->STCSR = 0;
}

void Delay_ms(unsigned int n)
{
	//__disable_irq();
	count = 0;
	//Systimer->STCVR = 0;
	//Systimer->STCSR = 7;		// enable counter, tickinit and choose system clock for systick
	//__enable_irq();
	while( n!=count );		
	//count = 0;
	//Systimer->STCSR = 0;
}
unsigned int time_distance_ns()		// return ns value
{
	unsigned int t;
	t = time*1000000 + (8000 - Systimer->STCVR)*125;
	time = 0;
	return t;
}

unsigned int time_distance_ms()		// return ms value
{
	unsigned int t;
	t = time;
	time = 0;
	return t;
}

/*uint32_t HAL_GetTick(void)
{
  return count;
}*/

void Set_PriorityGroup(unsigned int x)	// x: 0 - 4
{
	*(unsigned int*)0xE000ED0C = (0x5FA00000 | ((7-x)<<8));			// xem stm-note
}
void Set_Handler_Priority( handler Acronym, unsigned int PriorityLevel )		// PHAI VIET LAI HAM NAY CHO TOI UU HON, refer EXTI_config
{
	switch (Acronym)
	{	
		case MemMage:
			HandlerPri->SHPR1 &= 0xffffff00;					// clear 8 firts bit
			HandlerPri->SHPR1 |= PriorityLevel<<4;		// 4 hight bit are used
			break;
		case BusFaults:
			HandlerPri->SHPR1 &= 0xffff00ff;
			HandlerPri->SHPR1 |= PriorityLevel<<12;
			break;
		case UsageFaults:
			HandlerPri->SHPR1 &= 0xff00ffff;
			HandlerPri->SHPR1 |= PriorityLevel<<20;
			break;
		case SVCall:
			HandlerPri->SHPR2 &= 0x00ffffff;
			HandlerPri->SHPR2 |= PriorityLevel<<28;
			break;
		case PendSV:
			HandlerPri->SHPR3 &= 0xff00ffff;
			HandlerPri->SHPR3 |= PriorityLevel<<20;
			break;
		case SysTick_exception:
			HandlerPri->SHPR3 &= 0x00ffffff;
			HandlerPri->SHPR3 = PriorityLevel<<28;
			break;
	}
}

/* INT dung cho ngat noi chung, EXTI dung cho ngat ben ngoai */
void Set_INT_Priority( unsigned int IRQ_number, unsigned int PriorityLevel )
{
	IPR_REG->IPR[IRQ_number/4] = PriorityLevel<<(8*(IRQ_number%4)+4);
}
void EXTI_Enable( int IRQ_number, unsigned int line)		// enable interrup
{
	ISER_REG->ISER[IRQ_number/32] = 1<<IRQ_number;		// ko lo bi ghi de vi: write 0 no effect. 
	EXTI_REG->IMR = 1<<line;
}
void INT_Disable( int IRQ_number)		// enable interrup
{
	ICER_REG->ICER[IRQ_number/32] = 1<<(IRQ_number & 0x1Fu);		// ko lo bi ghi de vi: write 0 no effect.
}
void EXTI_config(EXTI_line LINEx, PORTx_* PORT, EXTI_egle EGLE, io_mode UP_DOWN)		// chon canh cho ngat.
{
	*(unsigned int*)(0x42000000 + 0x21018*32) = 1;		// RCC_APB2ENR.bit0 enable AFIO clock
	*(unsigned int*)(0x42000000 + ((unsigned int)&EXTI_REG->RTSR - 0x40000000)*32 + LINEx*4) = EGLE;
	*(unsigned int*)(0x42000000 + ((unsigned int)&EXTI_REG->FTSR - 0x40000000)*32 + LINEx*4) = EGLE>>1;
	AFIO_REG->EXTICR[LINEx/4] = (unsigned int)((((unsigned int)&PORT->PORTx_CRL - 0x40010800U)>>10)<<((LINEx%4)*4));	// Chon nguon ngat
	if( UP_DOWN == PULLUP )					*(unsigned int*)(0x42000000 + ((unsigned int)&PORT->PORTx_ODR - 0x40000000)*32 + LINEx*4) = 1;		// choose pullup
	else if( UP_DOWN == PULLDOWN )	*(unsigned int*)(0x42000000 + ((unsigned int)&PORT->PORTx_ODR - 0x40000000)*32 + LINEx*4) = 0;		// choose pulldown
}

void sysClk_setup(pll_st PLL_ST, osc_source OSC)
{	
	Systimer->STRVR = 7999;	// write to reload value reg: delay 1 ms with System clock = 8Mhz
	Systimer->STCSR = 7;		// enable counter, tickinit and choose system clock for systick
	Systimer->STCVR = 0;
	*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_RC - 0x40000000)*32 + OSC*4) = 1;		            // Enable HSI/HSE
	
	if(PLL_ST == PLLON)	
	{
		if(OSC == HSE)	*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_CFGR - 0x40000000)*32 + 17*4) = 1;
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_CFGR - 0x40000000)*32 + 16*4) = OSC/16U;  			// HSI/HSE is Source for PLL
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_CFGR - 0x40000000)*32) = 0; 									// PLL is system clk
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_CFGR - 0x40000000)*32 + 4) = 1; 								// ___
	}
	else if(PLL_ST == PLLOFF)
	{
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_CFGR - 0x40000000)*32 ) = OSC/16U; 						// HSI/HSE is system clk
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_CFGR - 0x40000000)*32 + 4) = 0;								// ___
		// while(*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_RC - 0x40000000)*32 + (OSC+1)*4) == 0);		// wait for OSC is stable
	} 
			*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_RC - 0x40000000)*32 + 24*4) = PLL_ST;		        // ON/OFF PLL
}
// APB2ENR - enable/disable PORTx (x = A...E)
void clk_gpio_setup(PORTx_* PORTx, clk_st ST)											
{
		unsigned int bit_number = (unsigned int)PORTx >> 8 & 0XFF;				// tinh ra bit_number*4 ung voi thanh ghi RCC_APB2ENR
		*(unsigned int*)(0x42000000 + 0x21018*32 + bit_number) = ST;		// bit_number chia 4 nhan 4 lai bang chinh no !
}

void gpio_init(PORTx_* PORTx, pinx PIN, port_mode MODE, io_mode MODE2)		// Max speed = 2Mhz in output mode
{
	unsigned int i = 0;
	if(PIN <= 7u)
	{
		*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRL - 0x40000000)*32 + PIN*16) = MODE;
		*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRL - 0x40000000)*32 + PIN*16+4) = 0;
		*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRL - 0x40000000)*32 + PIN*16+8) = MODE2%10;
		*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRL - 0x40000000)*32 + PIN*16+12) = MODE2/10;
		if( MODE2 == PULLUP )					*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_ODR - 0x40000000)*32 + PIN*4) = 1;		// choose pullup
		else if( MODE2 == PULLDOWN )	*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_ODR - 0x40000000)*32 + PIN*4) = 0;		// choose pulldown
		
	}
	else if(PIN == ALL_PORT)
	{
		while(i < 32)
		{	// bit 0-7
			*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRL - 0x40000000)*32 + i*16) = MODE;
			*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRL - 0x40000000)*32 + i*16+4) = 0;
			*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRL - 0x40000000)*32 + i*16+8) = MODE2%10;
			*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRL - 0x40000000)*32 + i*16+12) = MODE2/10;
			// bit 8-15
			*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRH - 0x40000000)*32 + i*16) = MODE;
			*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRH - 0x40000000)*32 + i*16+4) = 0;
			*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRH - 0x40000000)*32 + i*16+8) = MODE2%10;
			*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRH - 0x40000000)*32 + i*16+12) = MODE2/10;
			i += 4;
		}
		i = 0;
	}
	else if(PIN > 7u)
	{
		*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRH - 0x40000000)*32 + (PIN-8)*16) = MODE;
		*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRH - 0x40000000)*32 + (PIN-8)*16+4) = 0;
		*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRH - 0x40000000)*32 + (PIN-8)*16+8) = MODE2%10;
		*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_CRH - 0x40000000)*32 + (PIN-8)*16+12) = MODE2/10;
	}
}
void write_pin(PORTx_* PORTx, pinx PIN, pin_st ST)
{
	*(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_ODR - 0x40000000)*32 + PIN*4) = ST;
}

pin_st read_pin(PORTx_* PORTx, pinx PIN)
{
	return *(unsigned int*)(0x42000000 + ((unsigned int)&PORTx->PORTx_IDR - 0x40000000)*32 + PIN*4);
}

void ADC_setting(ADCx_* ADCx, adc_mode MODE)			// ignor disable clock ADC because it doesn't affect
{
	if(ADCx == ADC1_ID)						*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_APB2ENR - 0x40000000)*32 + 9*4) = 1;			// ENABLE clock ADC1
	else if(ADCx == ADC2_ID)			*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_APB2ENR - 0x40000000)*32 + 10*4) = 1;		// ENABLE clock ADC2
	else													*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_APB2ENR - 0x40000000)*32 + 15*4) = 1;		// ENABLE clock ADC3
	*(unsigned int*)(0x42000000 + ((unsigned int)&ADCx->CR2 - 0x40000000)*32 + 4) = MODE;		// single or continuous mode
}

unsigned char RegularGroup[] = {0};

void ADC_RegularGroup_Setup(ADCx_* ADCx, unsigned int NumOfChannels)	// trung t global
{
	int n = 0;
	int t;
	NumOfChannels -= 1;		// vi ADC_SQR1.bit23:20 = 0000 <-> 1 kenh
	
	// cau hinh so luong kenh regular (ADC_SQR1.bit23:20)
	while(n<4){
		*(unsigned int*)(0x42000000 + ((unsigned int)&ADCx->SQR1 - 0x40000000)*32 + (20+n)*4) = NumOfChannels%2;
		NumOfChannels /= 2;
		n++;}
	
	// setup conversion sequence
	for(n=0; n<=NumOfChannels; n++)
	{
		if(n<6)					
			for( t=0; t<5; t++)
			{
				*(unsigned int*)(0x42000000 + ((unsigned int)&ADCx->SQR3 - 0x40000000)*32 + (n*5 + t)*4) = RegularGroup[n]%2;		// n*5 vi moi kenh chua trong 5bits
					RegularGroup[n] /= 2;
			}
		else if(n>=6 && n<12)
			for( t=0; t<5; t++)
			{
				*(unsigned int*)(0x42000000 + ((unsigned int)&ADCx->SQR2 - 0x40000000)*32 + ((n-6)*5 + t)*4) = RegularGroup[n]%2;		// n=6 -> n-6=0
					RegularGroup[n] /= 2;
			}
		else
			for( t=0; t<5; t++)
			{
				*(unsigned int*)(0x42000000 + ((unsigned int)&ADCx->SQR1 - 0x40000000)*32 + ((n-12)*5 + t)*4) = RegularGroup[n]%2;	// n=12 -> n-12=0
					RegularGroup[n] /= 2;
			}
		}
	// sample time is fixed at 13.5 cycles
	//*(unsigned int*)(0x42000000 + ((unsigned int)&ADCx->SMP - 0x40000000)*32 + ((n-12)*5 + t)*4) =
}

void i2c1_init(I2Cx_* I2Cx_REG)
{
	I2Cx_REG->CR2 |= 0x02;												// FREQ = 2Mhz
	if(I2Cx_REG == I2C1_ID) write_bit(RCC_reg->RCC_APB1ENR, 21, 1);				// I2C1 clock enable
	if(I2Cx_REG == I2C2_ID) write_bit(RCC_reg->RCC_APB1ENR, 22, 1);				// I2C1 clock enable
	clk_gpio_setup(PORTB, EN);
	write_bit(RCC_reg->RCC_APB2ENR, 0, 1);				// AFIO clock enable
	gpio_init(PORTB, PORT6, OUTPUT, AFDRAIN);
	gpio_init(PORTB, PORT7, OUTPUT, AFDRAIN);
	I2Cx_REG->CCR |= 0x10;												// CCR calcular
	write_bit(I2Cx_REG->CCR, 15, 1);							// choose standard mode
	I2Cx_REG->TRISE |= 0x02;											// set rise time
	write_bit(I2Cx_REG->CR1, 0, 1);								// Enable i2c
}
void i2c_master_transmit(I2Cx_* I2Cx_REG, unsigned char SlaveAdd, unsigned char MemoryAdd, unsigned char* data, unsigned int size_data)
{
	unsigned int buff;
	write_bit(I2Cx_REG->CR1, 8, 1);								// START bit
	while(read_bit(I2Cx_REG->SR1, 0) == 0);				// wait START condition complete - SB bit = 1	V5
	buff = I2Cx_REG->SR1;													// clear SB	
	I2Cx_REG->DR = SlaveAdd;
	while(read_bit(I2Cx_REG->SR1, 1) == 0);				// wait slave address is sent - ADDR bit set V6
	buff = I2Cx_REG->SR1;													// clear ADDR
	buff = I2Cx_REG->SR2;
	while(read_bit(I2Cx_REG->SR1, 7) == 0);				// check DR empty - TxE = 1	V8_1
	I2Cx_REG->DR = MemoryAdd;											// send Memory address + clear TxE
	while(read_bit(I2Cx_REG->SR1, 7) == 0);				// check DR empty - TxE = 1	V8
	while( *data != NULL )
	{
		I2Cx_REG->DR = *data;												// send data + clear TxE
		*data = *(data + 1);				
		while(read_bit(I2Cx_REG->SR1, 7) == 0);			// check DR empty - TxE = 1	V8
	}
	while(read_bit(I2Cx_REG->SR1, 7) == 0);				// check DR empty - TxE = 1	V8_2
	while(read_bit(I2Cx_REG->SR1, 2) == 0);				// thanh ghi DR da khong duoc ghi after last byte transmitted
	write_bit(I2Cx_REG->CR1, 9, 1);								// send STOP condition
}
void i2c_master_receive(I2Cx_* I2Cx_REG, unsigned char SlaveAdd, unsigned char MemoryAdd, unsigned char* data, unsigned char size_data )
{
	unsigned char buff;
	unsigned int i;
	// Send Memory address first
	write_bit(I2Cx_REG->CR1, 8, 1);								// START bit
	while(read_bit(I2Cx_REG->SR1, 0) == 0);				// wait START condition complete - SB bit = 1	V5
	buff = I2Cx_REG->SR1;													// clear SB	
	I2Cx_REG->DR = SlaveAdd;
	while(read_bit(I2Cx_REG->SR1, 1) == 0);				// wait slave address is sent - ADDR bit set V6
	buff = I2Cx_REG->SR1;													// clear ADDR
	buff = I2Cx_REG->SR2;
	while(read_bit(I2Cx_REG->SR1, 7) == 0);				// check DR empty - TxE = 1	V8_1
	I2Cx_REG->DR = MemoryAdd;											// send Memory address + clear TxE
	while(read_bit(I2Cx_REG->SR1, 7) == 0);				// check DR empty - TxE = 1	V8
	
	// Begin read from Memory address
	write_bit(I2Cx_REG->CR1, 0, 1);								// Enable ACK
	write_bit(I2Cx_REG->CR1, 8, 1);								// START bit is used as a Restart bit	- page 758
	while(read_bit(I2Cx_REG->SR1, 0) == 0);				// wait START condition complete - SB bit = 1	V5
	buff = I2Cx_REG->SR1;													// clear SB	
	I2Cx_REG->DR = SlaveAdd;
	while(read_bit(I2Cx_REG->SR1, 1) == 0);				// wait slave address is sent - ADDR bit set V6
	buff = I2Cx_REG->SR1;													// clear ADDR
	buff = I2Cx_REG->SR2;
	for( i=0; i<size_data; i++ )
	{
		while(read_bit(I2Cx_REG->SR1, 6) == 6);				// check DR not empty - RxNE = 1 EV7
		data[i] = I2Cx_REG->DR;												// write data received to string(data[]) and clear RxNE
	}
	write_bit(I2Cx_REG->CR1, 0, 0);								// Disable ACK
	write_bit(I2Cx_REG->CR1, 9, 1);								// send STOP condition
}


/* ===================================================	HANDLER, INTERRUPT EXCUTION FUNCTION ================================================  */
void HAL_IncTick(void)
{
	count++;
	time++;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)		// maintaining
{
	if(GPIO_Pin == GPIO_PIN_0)		// = 0 is SET
	{
		t = 3;
	}
}

	int mu(int a, int b)
	{
		int i;
		int c = 1;
		for(i=0;i<b;i++)
		{
			c *= a;
		}
		return c;
	}
	short my_abs( short a )
	{
		if(a<0) a = ~a + 1;
		return a;
	}
	void data_convert_gravity(int d)
	{
		int i;
		for(i=0; i<8; i++) 			data_gravity[10-i] = (char)(((d%mu(10,i+1))/mu(10,i)) + 48);		// lcd_data[1] is MBS
	}
	void data_convert_degree(int d)
	{
		data_degree[8] = (char)(((d%mu(10,1))/mu(10,0)) + 48);		
    data_degree[7] = (char)(((d%mu(10,2))/mu(10,1))*.66 + 48);		// *.66 vi chi co 60" not 100"
    data_degree[5] = (char)(((d%mu(10,3))/mu(10,2)) + 48);	
    data_degree[4] = (char)(((d%mu(10,4))/mu(10,3))*.66 + 48);	// *.66 vi chi co 60' not 100'
    data_degree[2] = (char)(((d%mu(10,5))/mu(10,4)) + 48);	
    data_degree[1] = (char)(((d%mu(10,6))/mu(10,5)) + 48);	// * - lcd_data[0] is MBS
	}
	void data_convert_accel(int d)
	{
		int i;
		data_accel[2] = (char)(((d%mu(10,6))/mu(10,5)) + 48);
		data_accel[1] = (char)(((d%mu(10,7))/mu(10,6)) + 48);
		for(i=0; i<5; i++) 			data_accel[8-i] = (char)(((d%mu(10,i+1))/mu(10,i)) + 48);		// lcd_data[1] is MBS
	}
	void lcd_acceleration(int gravity_value)		// hien thi gia tri m/s2 cua 3 truc len lcd, 1 m/s2 = 2.61 gravity_value
	{
		int i;
		if(gravity_value < 0)	// ktra bit signed
		{
			data_accel[0] = '-';
			gravity_value = ~gravity_value + 1;		// doi tu (-) sang (+) phai cong them 1
		}
		else data_accel[0] = ' ';
		gravity_value *= 3832;								// 9.8/255.75 = 0.03832 ( 1 gravity_value = 0.03832 m/s2
		data_convert_accel(gravity_value);
		for(i=0;i<14;++i)	lcd_send_data(data_accel[i]);
	}
	void lcd_gravity(int gravity_value)		// hien thi gia tri g cua 3 truc len lcd, 1 g = 
	{
		int i;
		if(gravity_value < 0)	// ktra bit signed
		{
			data_gravity[0] = '-';
			gravity_value = ~gravity_value + 1;		// doi tu (-) sang (+) phai cong them 1
		}
		else data_gravity[0] = ' ';
		gravity_value *= 390625;
		data_convert_gravity(gravity_value);
		data_gravity[1] = (gravity_value/100000000 + 48);
		for(i=0;i<11;++i)	lcd_send_data(data_gravity[i]);
	}
	void lcd_degree(int gravity_value)		// hien thi gia tri goc cua 3 truc len lcd 
	{
		int i;
		if(gravity_value < 0)	// ktra bit signed
		{
			data_degree[0] = '-';
			gravity_value = ~gravity_value + 1;		// doi tu (-) sang (+) phai cong them 1
		}
		else data_degree[0] = ' ';
		gravity_value *= 3519;									// 1 gravity_value = 0.3519 degree
		data_convert_degree(gravity_value);
		for(i=0;i<11;++i)	lcd_send_data(data_degree[i]);
	}
	void lcd_make_exp2_ar_as()		// tao ky tu mu 2
	{
		int i;
		lcd_send_cmd(64);		// CGRAM 0 address
		for(i=0; i<7; ++i)	lcd_send_data(mu2[i]);
		lcd_send_cmd(72);		// CGRAM 0 address
		for(i=0; i<7; ++i)	lcd_send_data(ar[i]);
		lcd_send_cmd(80);		// CGRAM 0 address
		for(i=0; i<7; ++i)	lcd_send_data(as[i]);
	}
	void accelDevia_error_detect(float AccelDevia_value)		// Gia tri offset cua gia toc ban dau va cac thoi diem sau. Neu G0 - G1 > AcelDevia_value -> warning
	{
		//AccelDevia_value *= 255.75;
		AccelDevia_value *= 26.1;				// 26.1 ung voi 1 m/s2
		if((my_abs(x - AccelInitValue[0]) > AccelDevia_value)	| (my_abs(y - AccelInitValue[1]) > AccelDevia_value) | (my_abs(z - AccelInitValue[2]) > AccelDevia_value))
		{
			write_pin(PORTC, PORT13, ON);  // bao hieu do nga
			lcd_send_cmd(0xd0);
			lcd_send_data('C');
			lcd_send_data(1);
			lcd_send_data('n');
			lcd_send_data('h');
			lcd_send_data(' ');
			lcd_send_data('B');
			lcd_send_data(2);
			lcd_send_data('o');
		}
		if((read_pin(PORTA, PORT5) == 0) && (read_pin(PORTC, PORT13) == 0))
		{
			Delay_ms(5);
			while(read_pin(PORTA, PORT5) == 0);
			lcd_send_cmd(1);
			write_pin(PORTC, PORT13, OFF);
			set_devia();
		}
	}
void angleDevia_error_detect(float angleDevia_value)
	{
		angleDevia_value *= 2.842;
		if((my_abs(x - AngleInitValue[0]) > angleDevia_value)	| (my_abs(y - AngleInitValue[1]) > angleDevia_value) | (my_abs(z - AngleInitValue[2]) > angleDevia_value))
		{
			write_pin(PORTC, PORT13, ON);  // bao hieu do nga
			write_pin(PORTB, PORT13, OFF);  // bao hieu do nga
			lcd_send_cmd(0xd0);
			lcd_send_data('C');
			lcd_send_data(1);
			lcd_send_data('n');
			lcd_send_data('h');
			lcd_send_data(' ');
			lcd_send_data('B');
			lcd_send_data(2);
			lcd_send_data('o');
		}
		if((read_pin(PORTA, PORT5) == 0) && (read_pin(PORTC, PORT13) == 0))
		{
			Delay_ms(5);
			while(read_pin(PORTA, PORT5) == 0);
			lcd_send_cmd(1);
			write_pin(PORTC, PORT13, OFF);
			write_pin(PORTB, PORT13, ON);
			Delay_ms(1000);
			set_devia();
		}
}

void change()
{
	
}
uint8_t ADXL345_filter()
{
	if(my_abs(x-x0)>3 | my_abs(y-y0)>3 | my_abs(z-z0)>3)
	{
		x0 = x;
		y0 = y;
		z0 = z;
		return 1;   // != 0
	}
	else
	{
		x = x0;
		y = y0;
		z = z0;
		return 0;
	}
}

/*short kalman_filter(float Mea, float MeaError, float q)
{
	KG = EstError/(MeaError + EstError);
	R_measure = (KG*Mea - KG*Est);
	EstC += R_measure;
	EstError = (1-KG)*EstError + fabs(EstC - Est)*q;
	Est = EstC;
	return EstC;
}*/
short kalman_filter(short Mea, float MeaError, float q)
{
	KG = EstError/(MeaError + EstError);
	R_measure = (KG*Mea - KG*Est);
	EstC += R_measure;
	EstError = (1-KG)*EstError + fabs(EstC - Est)*q;
	Est = EstC;
	return EstC;
}
	float KalmanFilter( float z, float t)
{
    // calculate kalman gain
    K = P/(P + r);

    // update state
    s += (2*(z - a))/(t*t);     // snap
    j += (z - a)/t;             // jerk
    a += K*(z - a);            // accel

    // update covariance
    P *= (1-K);

    // extrapolate state
    a += j*t + s*t*t/2;
    j += s*t;
    s = s;

    // extrapolate covariance
    P += Q;

    return a;
}
void press_ok()
{
	if(read_pin(PORTA, PORT6)==0)		// ok button
	{
			Delay_ms(5);
			while(read_pin(PORTA, PORT6)==0);
			ok += 1;
	}
}
void press_up()
{
	if(read_pin(PORTA, PORT7)==0)		// up button
	{
		Delay_ms(5);
		while(read_pin(PORTA, PORT7)==0);
		if(m==0)  degree ++;
		else			ac++ ;
	}
}
void press_down()
{
	if(read_pin(PORTA, PORT5)==0)		// down button
	{
		Delay_ms(5);
		while(read_pin(PORTA, PORT5)==0);
		if(m==0)  degree --;
		else			ac-- ;
	}
}
void display()
{
	if(read_pin(PORTA, PORT6)==0)
	{
		Delay_ms(5);
		while(read_pin(PORTA, PORT6)==0);
		t += 1;
		lcd_send_cmd(1);
		if(t>2)		t=0;
	}
	switch(t)
	{
		case 0:
		{
			//lcd_send_cmd(1);
			lcd_send_cmd(0x80);
			lcd_send_string("X:");
			lcd_degree(x);
			lcd_send_cmd(0xc0);
			lcd_send_string("Y:");
			lcd_degree(y);
			lcd_send_cmd(0x90);
			lcd_send_string("Z:");
			lcd_degree(z);
			break;
		}
		case 1:
		{
			//lcd_send_cmd(1);
			lcd_send_cmd(0x80);
			lcd_send_string("X:");
			lcd_acceleration(x);
			lcd_send_cmd(0xc0);
			lcd_send_string("Y:");
			lcd_acceleration(y);
			lcd_send_cmd(0x90);
			lcd_send_string("Z:");
			lcd_acceleration(z);
			break;
		}
		case 2:
		{
			//lcd_send_cmd(1);
			lcd_send_cmd(0x80);
			lcd_send_string("X:");
			lcd_gravity(x);
			lcd_send_cmd(0xc0);
			lcd_send_string("Y:");
			lcd_gravity(y);
			lcd_send_cmd(0x90);
			lcd_send_string("Z:");
			lcd_gravity(z);
			break;
		}
		case 3:
		{
			lcd_send_cmd(1);
			while( ok != 2)
			{
				lcd_send_cmd(0x80);
				lcd_send_string("Change Devia:");
				lcd_send_cmd(0xc2);
				lcd_send_string("Angle");
				lcd_send_cmd(0x92);
				lcd_send_string("Accelebration");
				if(read_pin(PORTA, PORT7)==0)		// up button
				{
					Delay_ms(5);
					while(read_pin(PORTA, PORT7)==0);
					lcd_send_cmd(0xc0);
					lcd_send_data('>');
					up = 0;
					lcd_send_cmd(0x90);
					lcd_send_data(' ');
				}
				if(read_pin(PORTA, PORT6)==0)		// ok button
				{
					Delay_ms(5);
					while(read_pin(PORTA, PORT6)==0);
					ok += 1;
					if(0 == up)									// change degree deviation
					{
						lcd_send_cmd(1);
						m = 0;
						while(ok != 2)
						{
							lcd_send_cmd(0x80);
							lcd_send_data((degree/10) + 48);
							lcd_send_data((degree%10) + 48);
							lcd_send_data(0xdf);
							press_ok();
							press_up();
							press_down();
						}
					}
					if(0 == down)									// change accel deviation
					{
						m = 1;
						lcd_send_cmd(1);
						while(ok != 2)
						{
							lcd_send_cmd(0x80);
							lcd_send_data((ac/10) + 48);
							lcd_send_data((ac%10) + 48);
							lcd_send_string("m/s2");
							press_ok();
							press_up();
							press_down();
						}
					}
				}
				if(read_pin(PORTA, PORT5)==0)		// down button
				{
					Delay_ms(5);
					while(read_pin(PORTA, PORT5)==0);
					lcd_send_cmd(0x90);
					lcd_send_data('>');
					up = 3;
					lcd_send_cmd(0xc0);
					lcd_send_data(' ');
				}
			}
			ok = 0;
			t = 0;
			lcd_send_cmd(1);
			break;
		}
	}
}

I2C_HandleTypeDef hi2c1;
void read_adxl345()
	{
		HAL_I2C_Master_Transmit(&hi2c1, 0xA6, data, 1, 1000);		// data = 0x32 truy cap thanh ghi gia tri
		HAL_I2C_Mem_Read(&hi2c1, 0xA6, 0x32, 1, buf, 6, 1000);
	}

void set_devia()	// lay gia tri ban dau cho error_detect
{
	read_adxl345();
	AccelInitValue[0] = (buf[0] | (buf[1]<<8)) - 5;
	AccelInitValue[1] = (buf[2] | (buf[3]<<8)) + 3;
	AccelInitValue[2] = (buf[4] | (buf[5]<<8)) + 29;
	
	AngleInitValue[0] = AccelInitValue[0];
	AngleInitValue[1] = AccelInitValue[1];
	AngleInitValue[2] = AccelInitValue[2];
	lcd_send_cmd(0x01);
}
static void MX_I2C1_Init(void);


int main(void)
{
	int n;
	//FILE* prt;
	Set_PriorityGroup(4);
	sysClk_setup(PLLOFF, HSE);
	Set_Handler_Priority(SysTick_exception, 2);
	Set_INT_Priority(6,5);
	
	EXTI_Enable(6,0);  
	EXTI_config( EXTI0, PORTA, FALLING, PULLUP);
	clk_gpio_setup(PORTB, EN);
	gpio_init(PORTB, PORT13, OUTPUT, OUTPUSH);
	clk_gpio_setup(PORTC, EN);
	gpio_init(PORTC, PORT13, OUTPUT, OUTPUSH);
	clk_gpio_setup(PORTA, EN);
	gpio_init(PORTA, PORT0, INPUT, PULLUP); // can trong int
	gpio_init(PORTA, PORT6, INPUT, PULLUP);	// ok
	gpio_init(PORTA, PORT7, INPUT, PULLUP);	// up
	gpio_init(PORTA, PORT5, INPUT, PULLUP);	// down
	MX_I2C1_Init();
	//i2c1_init(I2C11_REG);
	lcd_init();
	lcd_make_exp2_ar_as();
	HAL_I2C_Master_Transmit(&hi2c1, 0xA6, adxl, 2, 1000);		// into measure mode
	
	set_devia();
	
	write_pin(PORTC, PORT13, OFF);	
	write_pin(PORTB, PORT13, ON);
  // lay gia tri ban dau cho filter
	HAL_I2C_Master_Transmit(&hi2c1, 0xA6, data, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, 0xA6, 0x32, 1, buf, 6, 1000);
	x0 = (buf[0] | (buf[1]<<8)) - 5;
	y0 = (buf[2] | (buf[3]<<8)) + 3;
	z0 = (buf[4] | (buf[5]<<8)) + 29;
  while (1)
  {
		// line 1-4: 0x80, 0xc0, 0x90, 0xd0
	  HAL_I2C_Master_Transmit(&hi2c1, 0xA6, data, 1, 1000);
		HAL_I2C_Mem_Read(&hi2c1, 0xA6, 0x32, 1, buf, 6, 1000);
		x = (buf[0] | (buf[1]<<8)) - 5;
		y = (buf[2] | (buf[3]<<8)) + 3;
		z = (buf[4] | (buf[5]<<8)) + 29;
		ADXL345_filter();
		//Kalman(z,10); 
		//k =  KalmanFilter(z,(time_distance_ns()/1000000000));
		k = kalman_filter(z,10,0.1);
		angleDevia_error_detect(degree);
		//accelDevia_error_detect(ac);
		//prt = fopen("C:\\Users\\ngoca\\OneDrive\\Desktop\\kalman.txt", "a");
		//fprintf(prt, "%f\n", z);
    //fclose(prt);
		//Delay_ms(100);
		display();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, 0x4E,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, 0x4E,(uint8_t *) data_t, 4, 100);
}
void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
	//while(*str != NULL) lcd_send_data(*str++);
}
void lcd_init (void)
{
	lcd_send_cmd (0x33); /* set 4-bits interface */
	lcd_send_cmd (0x32);
	Delay_ms(50);
	//HAL_Delay(50);
	lcd_send_cmd (0x28); /* start to set LCD function */
	Delay_ms(50);
	//HAL_Delay(50);
	lcd_send_cmd (0x01); /* clear display */
	Delay_ms(50);
	//HAL_Delay(50);
	lcd_send_cmd (0x06); /* set entry mode */
	Delay_ms(50);
	//HAL_Delay(50);
	lcd_send_cmd (0x0c); /* set display to on */	
	Delay_ms(50);
	//HAL_Delay(50);
	lcd_send_cmd (0x02); /* move cursor to home and set data address to 0 */
	Delay_ms(50);
	//HAL_Delay(50);
	lcd_send_cmd (0x80);
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
