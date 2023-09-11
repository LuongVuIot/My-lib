/**
	* @file 	stm32f103c8t6.c
	*	@author LuongVuIot
	*	@brief 	CLOCK, GPIO, INTERRUPT, HANDLER, SYSTICK, ADC, PWM	
	*/

#include"stm32f103c8t6.h"

void My_TIM_Clk(RCC_APBENR_TIMx TIM, clk_st ST)	// Enable clock for timer
{
	if (TIM == 1)  
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_APB2ENR - 0x40000000)*32 + 11*4) = ST;
	else if( TIM == 8 )
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_APB2ENR - 0x40000000)*32 + 13*4) = ST;
	else if( TIM == 9 || TIM == 10 || TIM == 11)
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_APB2ENR - 0x40000000)*32 + (TIM+10)*4) = ST;
	else if( TIM == 12 || TIM == 13 || TIM == 14)
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_APB1ENR - 0x40000000)*32 + (TIM-6)*4) = ST;
	else	// 2-7
		*(unsigned int*)(0x42000000 + ((unsigned int)&RCC_reg->RCC_APB1ENR - 0x40000000)*32 + (TIM-2)*4) = ST;		
}

void My_TIM_Init( TIMx_* mtim, uint16_t PSC, uint16_t ARR, direction DIR, align CMS)
{
	mtim->TIMx_PSC = PSC;
	mtim->TIMx_ARR = ARR;
	mtim->TIMx_CR1 |= DIR<<4;
	mtim->TIMx_CR1 |= CMS<<5;
}
void My_TIM_Start(TIMx_* mtim)		// enable counter
{
	mtim->TIMx_CR1 |= 1;
}
void My_TIM_Stop(TIMx_* mtim)		// disable counter
{
	*(unsigned int*)(0x42000000 + ((unsigned int)&mtim->TIMx_CCER - 0x40000000)*32) = 0;
}

void My_PWM_Init(TIMx_* mtim, uint8_t channel, OC_mode mode, OC_active polarity, uint16_t duty_cycle)
{
	if(channel == 1 || channel == 2)
		mtim->TIMx_CCMR1 |= mode<<(8*channel - 4);		// PWM mode
	else	// channel 3,4
		mtim->TIMx_CCMR1 |= mode<<(8*channel - 20);		// PWM mode
	
	mtim->TIMx_CCER |= polarity<<(4*channel - 3);		
	
	switch(channel)
	{
		case 1:
			mtim->TIMx_CCR1 = duty_cycle;
			break;
		case 2:
			mtim->TIMx_CCR2 = duty_cycle;
			break;
		case 3:
			mtim->TIMx_CCR3 = duty_cycle;
			break;
		case 4:
			mtim->TIMx_CCR4 = duty_cycle;
			break;
	}
}
void My_PWM_CCR(TIMx_* mtim, uint8_t channel, uint16_t duty_cycle)
{
	switch(channel)
	{
		case 1:
			mtim->TIMx_CCR1 = duty_cycle;
			break;
		case 2:
			mtim->TIMx_CCR2 = duty_cycle;
			break;
		case 3:
			mtim->TIMx_CCR3 = duty_cycle;
			break;
		case 4:
			mtim->TIMx_CCR4 = duty_cycle;
			break;
	}
}
void My_PWM_Start(TIMx_* mtim, uint8_t channel)
{
	mtim->TIMx_CCER |= 1<<(4*channel - 4);
}
void My_PWM_Stop(TIMx_* mtim, uint8_t channel)
{
	*(unsigned int*)(0x42000000 + ((unsigned int)&mtim->TIMx_CCER - 0x40000000)*32 + (channel-1)*16) = 0;
}
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

void write_bit( unsigned int reg, unsigned char bit, unsigned char value)
{
	*(unsigned int*)(0x42000000 + ((unsigned int)&reg - 0x40000000)*32 + bit*4) = value;
}
unsigned int read_bit(unsigned int reg, unsigned char bit)
{
	return *(unsigned int*)(0x42000000 + ((unsigned int)&reg - 0x40000000)*32 + bit*4);
}
void delay_systick(unsigned int n)	// n = ms
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

/*unsigned int time_distance_ns()		// return ns value
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
}*/

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
			HandlerPri->SHPR1 |= PriorityLevel<<4;		// 4 high bits are used
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

