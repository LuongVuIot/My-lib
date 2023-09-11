/**
	* @file 	stm32f103c8t6.h
	*	@author LuongVuIot
	*	@brief 	CLOCK, GPIO, ADC, PWM, INTERRUPT, HANDLER, SYSTICK
	*/

#include<stdint.h>

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

#define TIM1_ID (TIMx_*)0x40012C00U
#define TIM2_ID (TIMx_*)0x40000000U
#define TIM3_ID (TIMx_*)0x40000400U
#define TIM4_ID (TIMx_*)0x40000800U
#define TIM5_ID (TIMx_*)0x40000C00U
#define TIM6_ID (TIMx_*)0x40001000U
#define TIM7_ID (TIMx_*)0x40001400U
#define TIM8_ID (TIMx_*)0x40013400U
#define TIM9_ID (TIMx_*)0x40014C00U
#define TIM10_ID (TIMx_*)0x40015000U
#define TIM11_ID (TIMx_*)0x40015400U
#define TIM12_ID (TIMx_*)0x40013400U
#define TIM13_ID (TIMx_*)0x40001C00U
#define TIM14_ID (TIMx_*)0x40002000U

typedef enum { 
	PLLOFF = 0U, 
	PLLON 
} pll_st;

typedef enum { 
	HSI = 0U, 
	HSE = 16U 
} osc_source;

typedef enum pin_st { 
	ON = 0U, 
	OFF 
} pin_st;								// Reset/Set Pinx

typedef enum clk_st { 
	DIS = 0u, 
	EN 
} clk_st;								// Enable/disable Portx clock

typedef enum port_mode { 
	INPUT = 0u, 
	OUTPUT
} port_mode;			// OUTPUT max speed = 2Mhz

typedef enum io_mode { 
	ANALOG = 0u, 
	FLOAT, 
	PULLUP = 10u, 
	PULLDOWN = 12u,
	OUTPUSH = 0u, 
	OUTDRAIN, 
	AFPUSH = 10u, 
	AFDRAIN 
} io_mode;

typedef enum pinx { 
	PORT0 = 0u, 
	PORT1, 
	PORT2, 
	PORT3, 
	PORT4, 
	PORT5, 
	PORT6, 
	PORT7,
	PORT8, 
	PORT9, 
	PORT10, 
	PORT11, 
	PORT12, 
	PORT13, 
	PORT14, 
	PORT15, 
	ALL_PORT 
} pinx;

typedef enum handler { 
	MemMage = 0u, 
	BusFaults, 
	UsageFaults, 
	SVCall, 
	PendSV, 
	SysTick_exception 
} handler;

typedef enum EXTI_line { 
	EXTI0 = 0u, 
	EXTI1, 
	EXTI2, 
	EXTI3, 
	EXTI4, 
	EXTI5, 
	EXTI6, 
	EXTI7, 
	EXTI8, 
	EXTI9,
	EXTI_10, 
	EXTI_11, 
	EXTI_12,
	EXTI_13, 
	EXTI14, 
	EXTI15
} EXTI_line;

typedef enum EXTI_egle { 
	RISING = 1u, 
	FALLING = 2u,
	BOTH = 3u
} EXTI_egle;		// BOTH will set RISING and FALLING simutaneous

typedef enum adc_mode { 
	SINGLE, 
	CONTINUOUS 
} adc_mode;

// for Timer
typedef enum { 
	UpCounter = 0, 
	DowCounter 
} direction;	// direction of counter.

typedef enum { 
	Edge=0, 
	Center1, 
	Center2, 
	Center3 
} align;	// center-align mode selection

typedef enum { 
	APB_TIM1 = 1U, 
	APB_TIM2, 
	APB_TIM3, 
	APB_TIM4, 
	APB_TIM5, 
	APB_TIM6, 
	APB_TIM7, 
	APB_TIM8, 
	APB_TIM9, 
	APB_TIM10,
	APB_TIM11, 
	APB_TIM12, 
	APB_TIM13, 
	APB_TIM14 
} RCC_APBENR_TIMx;

// for PWM
typedef enum { Mode1 = 6, Mode2 } OC_mode; // output compare mode.
typedef enum { High = 0, Low } OC_active; 

volatile unsigned int count = 0;

typedef struct{
	uint32_t TIMx_CR1;		// 00
	uint32_t TIMx_CR2;		// 04
	uint32_t TIMx_SMCR;		// 08
	uint32_t TIMx_DIER;		// 0C
	uint32_t TIMx_SR;			// 10
	uint32_t TIMx_EGR;		// 14
	uint32_t TIMx_CCMR1;	// 18
	uint32_t TIMx_CCMR2;	// 1C
	uint32_t TIMx_CCER;		// 20
	uint32_t TIMx_CNT;		// 24
	uint32_t TIMx_PSC;		// 28
	uint32_t TIMx_ARR;		// 2C
	uint32_t TIMx_RCR;		// 30
	uint32_t TIMx_CCR1;		// 34
	uint32_t TIMx_CCR2;		// 38
	uint32_t TIMx_CCR3;		// 3C
	uint32_t TIMx_CCR4;		// 40
	uint32_t unused1;			// 44
	uint32_t TIMx_DCR;		// 48
	uint32_t TIMx_DMAR;		// 4C
}TIMx_;

TIMx_* TIM_2 = TIM2_ID;
TIMx_* TIM_3 = TIM3_ID;
TIMx_* TIM_4 = TIM4_ID;

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

// ADC register struct defination
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

// I2C register struct defination
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

// Common function
void write_bit( unsigned int reg, unsigned char bit, unsigned char value);
unsigned int read_bit(unsigned int reg, unsigned char bit);
void delay_systick(unsigned int n);

// System clock prototype funcion
void sysClk_setup(pll_st PLL_ST, osc_source OSC);

// GPIO prototype functions
void clk_gpio_setup(PORTx_* PORTx, clk_st ST);
void gpio_init(PORTx_* PORTx, pinx PIN, port_mode MODE, io_mode MODE2);
void write_pin(PORTx_* PORTx, pinx PIN, pin_st ST);
pin_st read_pin(PORTx_* PORTx, pinx PIN);

// Timer prototype functions
void My_TIM_Clk(RCC_APBENR_TIMx TIM, clk_st ST);
void My_TIM_Init( TIMx_* mtim, uint16_t PSC, uint16_t ARR, direction DIR, align CMS);
void My_TIM_Start(TIMx_* mtim);
void My_TIM_Stop(TIMx_* mtim);

// PWM
void My_PWM_Init(TIMx_* mtim, uint8_t channel, OC_mode mode, OC_active polarity, uint16_t duty_cycle);
void My_PWM_CCR(TIMx_* mtim, uint8_t channel, uint16_t duty_cycle);
void My_PWM_Start(TIMx_* mtim, uint8_t channel);
void My_PWM_Stop(TIMx_* mtim, uint8_t channel);

// ADC - Haven't commpleted yet.
void ADC_setting(ADCx_* ADCx, adc_mode MODE);
void ADC_RegularGroup_Setup(ADCx_* ADCx, unsigned int NumOfChannels);

// Interrupt, Handler
void Set_INT_Priority( unsigned int IRQ_number, unsigned int PriorityLevel );
void Set_PriorityGroup(unsigned int x);
void Set_Handler_Priority( handler Acronym, unsigned int PriorityLevel );
void EXTI_config(EXTI_line LINEx, PORTx_* PORT, EXTI_egle EGLE, io_mode UP_DOWN);
void EXTI_Enable( int IRQ_number, unsigned int line);
void INT_Disable( int IRQ_number);



