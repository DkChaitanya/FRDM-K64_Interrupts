#include "fsl_device_registers.h"
#include"MK64F12.h"
void DelayFunction (void);
int main(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; //ENABLE CLOCK TO PORT B//
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //ENABLE CLOCK TO PORT C//
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //ENABLE CLOCK TO PORT A//

	PORTB_PCR21 = 0x100; //BLUE LED SET AS GPIO//
	PORTB_PCR22 = 0x100; //RED LED SET AS GPIO//
	PORTC_PCR6 = 0x90100; //SW2 - PORTC_PCR6: ISF=0,IRQC=9,MUX=1//
	PORTA_PCR4 = 0x100; //SW3 IS SET AS GPIO//

	GPIOB_PDDR |= (1<<21); //SETTING PTB21 AS OUTPUT//
	GPIOB_PDDR |= (1<<22); //SETTING PTB21 AS OUTPUT//
	GPIOC_PDDR |= (0<<6); //SETTING PTB21 AS INPUT//

	GPIOB_PDOR |= (1<<22); //TURN OFF RED LED//
	GPIOB_PDOR |= (1<<21); //TURN OFF BLUE LED//

	PORTC_ISFR = PORT_ISFR_ISF(0x40); //CLEAR INTERRUPT STATUS FLAG//
	NVIC_EnableIRQ(PORTC_IRQn); //ENABLE PORTC INTERRUPT//
    /* This for loop should be replaced. By default this loop allows a single stepping. */
    for (;;) {
       GPIOB_PTOR |= (1<<22); //RED BLINK//
       DelayFunction();
    }
    return 0;
}

void PORTC_IRQHandler (void)
{
	DelayFunction();
	GPIOB_PSOR |= (1<<22); //Turn off Red LED//
	GPIOB_PCOR |= (1<<21); //Turn on Blue LED//
	DelayFunction();
	GPIOB_PSOR |= (1<<21); //Turn off Blue LED//
	DelayFunction();

	PORTC_ISFR = PORT_ISFR_ISF(0x40); //Clear Interrupt Flag//
}

void DelayFunction (void)
{
	int cnt;
	for(cnt=0;cnt<1000000;cnt++)
	{
	}
}
/* The following code is written by Krishna Chaitanya Devulapalli, 
 * however the FreeScale libraries were used */
/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */ 

