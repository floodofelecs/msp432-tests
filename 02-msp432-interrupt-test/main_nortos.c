/*
 * Copyright (c) 2017-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,

 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti/devices/msp432p4xx/inc/msp.h"

/*
 *  ======== main_nortos.c ========
 */
#include <stdint.h>
#include <stddef.h>

#include <NoRTOS.h>

/* Driver configuration */
#include <ti/drivers/Board.h>

extern void *mainThread(void *arg0);

/**
 * Simple init routine that flashes the D1 LED and waits for GPIO0 to be pressed.
 * Written at the register level to ensure this code is not likely to crash CPU.
 */
void init_wait() {
    volatile uint32_t i;

    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
                 WDT_A_CTL_HOLD;

    P9->DIR |= BIT1;                        // P9.1 set as output (Connected to D2)
    P4->DIR &= ~BIT5;                       // P4.5 set as input (connected to GPIO0)
    // Enable P4 pulldown resistor
    P4->REN |= BIT5;
    P4->OUT &= ~BIT5;
    // Disable alternate functions for P4.5
    P4->SEL0 &= ~BIT5;
    P4->SEL1 &= ~BIT5;

    /*
     * While P4.5 is low, wait for it to be pulled high
     */
    while ((P4->IN & BIT5) == 0)
    {
        P9->OUT ^= BIT1;                    // Blink P9.1 LED
        for (i = 20000; i > 0; i--);        // Delay

    }
}

/*
 * This test starts by flashing LED D1, and after GPIO0 is pressed it will
 * turn both LEDs off. When GPIO0 is pressed again, it will toggle both LEDs
 */
/*
 *  ======== main ========
 */
int main(void)
{
    // Wait for GPIO to be pressed
    init_wait();
    /*
     * The below code uses nortos library.
     */
    /* Call driver init functions */
    Board_init();

    /* Start NoRTOS */
    NoRTOS_start();

    /* Call mainThread function */
    mainThread(NULL);

    while (1) {}
}
