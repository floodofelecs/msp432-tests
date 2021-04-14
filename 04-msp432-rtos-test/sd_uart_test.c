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
 *  ======== sd_uart_test.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <file.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* POSIX Header files */
#include <pthread.h>

#include <third_party/fatfs/ffcio.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDFatFS.h>
#include <ti/drivers/UART.h>

/* xdc module headers */
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* Driver configuration */
#include "ti_drivers_config.h"


/* String conversion macro */
#define STR_(n)             #n
#define STR(n)              STR_(n)

/* Drive number used for FatFs */
#define DRIVE_NUM           0

const char outputfile[] = "fat:"STR(DRIVE_NUM)":out.txt";

const char textarray[] = \
"***********************************************************************\n"
"0         1         2         3         4         5         6         7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"This is some text to be inserted into the output file if there isn't\n"
"already an existing file located on the media.\n"
"***********************************************************************\n";

/* Set this to the current UNIX time in seconds */
const struct timespec ts = {
    .tv_sec = 1469647026,
    .tv_nsec = 0
};

/* File name prefix for this filesystem for use with TI C RTS */
char fatfsPrefix[] = "fat";

/*
 *  ======== uartThread ========
 *  This code will echo UART characters, and attempt to write a file to the SD card.
 *  It will also illuminate the D2 led
 */
void *uartThread(void *arg0)
{
    char        input;
    const char  echoPrompt[] = "Echoing characters:\r\n";
    UART_Handle uart;
    UART_Params uartParams;
    /* Call driver init functions */
    GPIO_init();
    UART_init();

    System_printf("Starting RTOS test\n");
    System_flush();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    UART_write(uart, echoPrompt, sizeof(echoPrompt));

    /* Loop forever echoing */
    while (1) {
        UART_read(uart, &input, 1);
        UART_write(uart, &input, 1);
    }
}

/**
 * This function will access the SD card and write a file to it.
 */
void *sdThread(void *arg0)
{
    SDFatFS_Handle sdfatfsHandle;
    FILE *dst;

    SDFatFS_init();
    /* add_device() should be called once and is used for all media types */
    add_device(fatfsPrefix, _MSA, ffcio_open, ffcio_close, ffcio_read,
        ffcio_write, ffcio_lseek, ffcio_unlink, ffcio_rename);

    /* Initialize real-time clock */
    clock_settime(CLOCK_REALTIME, &ts);

    sdfatfsHandle = SDFatFS_open(CONFIG_SDFatFS_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        System_printf("Error starting the SD card\n");
        System_flush();
        return NULL;
    }
    dst = fopen(outputfile, "w+");
    if (!dst) {
        System_printf("Could not open destination file\n");
        System_flush();
    }
    // Write test string to file
    if (fwrite(textarray, strlen(textarray), 1, dst) != 1) {
        System_abort("Could not write to destination file\n");
    }
    fflush(dst);
    fclose(dst);
    System_printf("Successfully wrote to SD card file\n");
    System_flush();
    return NULL;
}

/*
 *  ======== fatfs_getFatTime ========
 *  Has to be provided or code won't compile
 */
int32_t fatfs_getFatTime(void)
{
    time_t seconds;
    uint32_t fatTime;
    struct tm *pTime;

    /*
     *  TI time() returns seconds elapsed since 1900, while other tools
     *  return seconds from 1970.  However, both TI and GNU localtime()
     *  sets tm tm_year to number of years since 1900.
     */
    seconds = time(NULL);

    pTime = localtime(&seconds);

    /*
     *  localtime() sets pTime->tm_year to number of years
     *  since 1900, so subtract 80 from tm_year to get FAT time
     *  offset from 1980.
     */
    fatTime = ((uint32_t)(pTime->tm_year - 80) << 25) |
        ((uint32_t)(pTime->tm_mon) << 21) |
        ((uint32_t)(pTime->tm_mday) << 16) |
        ((uint32_t)(pTime->tm_hour) << 11) |
        ((uint32_t)(pTime->tm_min) << 5) |
        ((uint32_t)(pTime->tm_sec) >> 1);

    return ((int32_t)fatTime);
}
