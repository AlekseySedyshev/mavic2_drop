#include "stm8l10x.h"

#define SW_SHORT					GPIOC->ODR 	&=(~0x8);
#define SW_RELEASE				GPIOC->ODR |=0x8;

#define SLEEP_CYCLE				30		// ¬рем€ сна между замерами в минутах 	
#define LIGHT (GPIOD->IDR & 1)
//----------------------------------

uint16_t TimingDelay,sec_tic;
uint8_t pwm_tic;

void TimingDelayDec(void) 															{//Delay Main function
 if (TimingDelay!=0x00) {TimingDelay--;}
 if (!sec_tic) {sec_tic=1000;GPIOC->ODR ^=8;} 
 sec_tic--;
 }
void delay_ms (uint16_t DelTime) 												{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}

void initial(void)																			{//TIM4 INIT
GPIOA->DDR=0; GPIOA->CR1=0xff; GPIOA->CR2=0;
GPIOB->DDR=0; GPIOB->CR1=0xff; GPIOB->CR2=0;
GPIOC->DDR=0; GPIOC->CR1=0xff; GPIOC->CR2=0;
GPIOD->DDR=0; GPIOD->CR1=0xff; GPIOD->CR2=0;

GPIOC->DDR|=8; GPIOC->CR1|=8; GPIOC->CR2|=8;

CLK->CKDIVR = 3;						//2MHz
CLK->PCKENR |=CLK_PCKENR_AWU;

//----------TIM4------------------
CLK->PCKENR |= CLK_PCKENR_TIM4;
TIM4->PSCR = 4;
TIM4->ARR = 124; 																	// 2^4 = 16, 16*125 = 2000 
TIM4->SR1 &=~TIM4_SR1_UIF; 												//сброс флага прерывани€ TIM4_ClearFlag(TIM4_FLAG_UPDATE); 	
TIM4->IER	|= TIM4_IER_UIE; 												//прерывание включено
TIM4->CR1 |= TIM4_CR1_CEN; 												// запустить таймер
_asm("rim");

//=============TIM2_PWM================
CLK->PCKENR |= CLK_PCKENR_TIM2;

GPIOB->DDR |=0x1;GPIOB->CR1 |=0x1; GPIOB->CR2 |=0x1;		//PB0 - Out Tim2Ch1

TIM2->PSCR = 0b111; // prescaler 
TIM2->ARRH = 0;TIM2->ARRL = 200;
TIM2->SMCR 	=0;
TIM2->CCMR1	|= 0b110<<4 | TIM_CCMR_OCxPE; 										//PWM1 mode
TIM2->CCER1 |= TIM_CCER1_CC1E;																//Active pulse High 
TIM2->CCR1H=0;	TIM2->CCR1L=0; 																//Reset count

TIM2->CNTRH=0;	TIM2->CNTRL=0;
TIM2->BKR |=TIM_BKR_MOE;
TIM2->EGR |= TIM_EGR_UG;
TIM2->CR1 |=TIM_CR1_CEN;
}

void sleep_mode(void) {// уход в сп€чку

   AWU->APR &=(~0x3F) | 0x3D;    //AWU->APR &=(~0x3F) | 0x3E; // сброс предделител€ и установка его на 62
   AWU->TBR |=0xD;                   //AWU->TBR |=0xE;  5 сек, //AWU->TBR |=0xF; //-----30 сек
   AWU->CSR |=AWU_CSR_AWUEN ;  //разрешить прерывание по окончанию.
      
	 TIM3->CR1 &=~TIM_CR1_CEN;
   TIM4->CR1 &=~TIM4_CR1_CEN;
   CLK->PCKENR &=(~CLK_PCKENR_TIM3) &(~CLK_PCKENR_TIM4);
     _asm("halt");  //уснуть
   CLK->PCKENR |=CLK_PCKENR_TIM3 | CLK_PCKENR_TIM4;
   TIM4->CR1 |=TIM4_CR1_CEN;
	 TIM3->CR1 |=TIM_CR1_CEN;
}
void sleep_min(uint16_t time) {
 uint16_t tmp;
	 AWU->APR = 0x23;    						//	0x23 + 2 = 37
   AWU->TBR = 0xF;                //	0xF,  30*2^11 = 61440;  61440 * 37 / 38000 = 59,8 сек
   AWU->CSR |=AWU_CSR_AWUEN ;  	 	// разрешить прерывание по окончанию.
      
	 TIM3->CR1 &=~TIM_CR1_CEN;
   TIM4->CR1 &=~TIM4_CR1_CEN;
   CLK->PCKENR &=(~CLK_PCKENR_TIM3) &(~CLK_PCKENR_TIM4);
	 for (tmp=0; tmp<time;tmp++){
			_asm("halt");  //уснуть
		}
   CLK->PCKENR |=CLK_PCKENR_TIM3 | CLK_PCKENR_TIM4;
   TIM4->CR1 |=TIM4_CR1_CEN;
	 TIM3->CR1 |=TIM_CR1_CEN;
}

int main(void) {

initial();
//----------------Main Cycle------------------------
while (1)		 {

if(LIGHT) {

TIM2->CCR1L=10;// 0,6mS
}
else 
{
TIM2->CCR1L=30;// 1,9mS
}


} // END while
} // End Main

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
