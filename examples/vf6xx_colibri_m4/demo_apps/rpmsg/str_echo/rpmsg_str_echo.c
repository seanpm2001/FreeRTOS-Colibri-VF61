/*
 * Copyright (c) 2016, Toradex AG
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

#include <stdio.h>
#include <errno.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "string.h"
#include "assert.h"
#include "rpmsg/rpmsg.h"
#include "plat_porting.h"
#include <ccm_vf6xx.h>
#include "debug_console_vf6xx.h"
#include "pin_mux.h"
#include "rpmsg/rpmsg_rtos.h"

/*
 * function decalaration for platform provided facility
 */
extern void platform_interrupt_enable(void);
extern void platform_interrupt_disable(void);

/*
 * APP decided interrupt priority
 */
#define APP_MSCM_IRQ_PRIORITY  3

/* Globals */
static char app_buf[512]; /* Each RPMSG buffer can carry less than 512 payload */

/*!
 * @brief A basic RPMSG task
 */
static void StrEchoTask(void *pvParameters)
{
    int result;
    struct remote_device *rdev = NULL;
    struct rpmsg_channel *app_chnl = NULL;
    void *rx_buf;
    int len;
    unsigned long src;
    void *tx_buf;
    unsigned long size;

    /* Print the initial banner */
    PRINTF("\r\nRPMSG String Echo FreeRTOS RTOS API Demo...\r\n");

    /* RPMSG Init as REMOTE */
    PRINTF("RPMSG Init as Remote\r\n");
    result = rpmsg_rtos_init(0 /*REMOTE_CPU_ID*/, &rdev, RPMSG_MASTER, &app_chnl);
    assert(result == 0);

    PRINTF("Name service handshake is done, M4 has setup a rpmsg channel [%d ---> %d]\r\n", app_chnl->src, app_chnl->dst);

    /*
     * str_echo demo loop
     */
    for (;;)
    {
        /* Get RPMsg rx buffer with message */
        result = rpmsg_rtos_recv_nocopy(app_chnl->rp_ept, &rx_buf, &len, &src, 0xFFFFFFFF);
        assert(result == 0);

        /* Copy string from RPMsg rx buffer */
        assert(len < sizeof(app_buf));
        memcpy(app_buf, rx_buf, len);
        app_buf[len] = 0; /* End string by '\0' */

        if ((len == 2) && (app_buf[0] == 0xd) && (app_buf[1] == 0xa))
            PRINTF("Get New Line From Master Side\r\n");
        else
            PRINTF("Get Message From Master Side : \"%s\" [len : %d]\r\n", app_buf, len);

        /* Get tx buffer from RPMsg */
        tx_buf = rpmsg_rtos_alloc_tx_buffer(app_chnl->rp_ept, &size);
        assert(tx_buf);
        /* Copy string to RPMsg tx buffer */
        memcpy(tx_buf, app_buf, len);
        /* Echo back received message with nocopy send */
        result = rpmsg_rtos_send_nocopy(app_chnl->rp_ept, tx_buf, len, src);
        assert(result == 0);

        /* Release held RPMsg rx buffer */
        result = rpmsg_rtos_recv_nocopy_free(app_chnl->rp_ept, rx_buf);
        assert(result == 0);
    }
}

int main(void)
{
    /* Init Clock Control and UART */
    CCM_GetClocks();
    CCM_ControlGate(ccmCcgrGateUart2, ccmClockNeededAll);

    configure_uart_pins(UART2);
    vf6xx_DbgConsole_Init(UART2, ccmIpgBusClk, 115200);

    printf("Starting RPMSG String Echo Demo...\r\n");

    /*
    * Prepare for the MSCM Interrupt
    * MSCM must be initialized before rpmsg init is called
    */
    platform_interrupt_enable();
    NVIC_SetPriority(CPU2CPU_INT0_IRQ, APP_MSCM_IRQ_PRIORITY);
    NVIC_SetPriority(CPU2CPU_INT1_IRQ, APP_MSCM_IRQ_PRIORITY);


    // Create a demo task which will print Hello world and echo user's input.
    xTaskCreate(StrEchoTask, "String Echo Task", configMINIMAL_STACK_SIZE,
		NULL, tskIDLE_PRIORITY+1, NULL);

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    // Should never reach this point.
    while (true);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
