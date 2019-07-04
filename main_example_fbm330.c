
#include <stdio.h>
#include <fbm330.h>

/* Private variable */
int32_t real_p, real_t, altitude;
volatile uint32_t TMR0_Ticks = 0; //one tick per millisecond(ms)
volatile uint32_t fbm330_update_rdy = 0;

/**
 * @brief      A timer generate an interrupt every millisecond
 */
void TMR0_IRQHandler(void)
{
	if (TIMER_GetIntFlag(TIMER0) == 1)
	{
		/* Clear Timer0 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER0);

		TMR0_Ticks++;
	}
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

	fbm330_init();//fbm330 initiation

	while (1)
	{
		fbm330_update_data();//Updating fbm330 data
		if (fbm330_update_rdy) {
			// real_p = fbm330_read_pressure();//If you need the pressure value read is in uint of Pa, use this function.
			// real_t = fbm330_read_temperature();//If you need the temperature value read is in unit of degree Celsius, use this function.

			/* This function read pressure and temperature values. Pressure uint:Pa, Temperature unit:0.01 degree Celsius */
			fbm330_read_data(&real_p, &real_t);
			fbm330_update_rdy = 0;
		}
	}
}
