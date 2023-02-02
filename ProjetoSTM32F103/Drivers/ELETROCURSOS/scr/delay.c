#include "delay.h"

__IO uint32_t Time_Cnt1;
__IO uint32_t Time_Cnt2;

void Delayus(uint32_t usdelay)
{
	uint32_t timer = SysTick->VAL;
	do
	{
		while ((timer - SysTick->VAL) < 50); //Valor ajustado com analisador lógico;

		timer = SysTick->VAL;

	} while (--usdelay);

}

void Delayms(uint32_t msdelay)
{
	volatile uint32_t timer = Time_Cnt1;

	while ((Time_Cnt1 - timer) < msdelay);
}


void DelayIncCnt()
{
	Time_Cnt1++;
	Time_Cnt2++;
}
