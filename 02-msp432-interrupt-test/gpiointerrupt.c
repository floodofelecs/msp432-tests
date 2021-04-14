/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// Just toggles D1 and D2 LEDs
void gpioButtonFxn0(uint8_t index) {
    GPIO_toggle(CONFIG_GPIO_D1_LED);
    GPIO_toggle(CONFIG_GPIO_D2_LED);
}

/*
 *  ======== mainThread ========
 *  This code will disable both D1 and D2 LEDs, and then will wait for an interrupt on GPIO0.
 *  Tests no-rtos library, as well as GPIO interrupts.
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    // Make both LEDs low
    GPIO_setConfig(CONFIG_GPIO_D1_LED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_D2_LED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    // Set GPIO0 to pulldown and to interrupt on rising edge
    GPIO_setConfig(CONFIG_GPIO0_BUTTON, GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_RISING);

    GPIO_write(CONFIG_GPIO_D1_LED, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_D2_LED, CONFIG_GPIO_LED_OFF);
    /* install Button callback */
    GPIO_setCallback(CONFIG_GPIO0_BUTTON, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO0_BUTTON);


    return (NULL);
}
