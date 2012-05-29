/*
 * Copyright (c) 2011, Texas Instruments Incorporated
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
 *  ======== Interrupt.c ========
 *  OMAPL138/DSP Interrupt Manger
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/family/c64p/Hwi.h>

#include <ti/ipc/MultiProc.h>

#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>

#include "InterruptDsp.h"

#include "package/internal/InterruptDsp.xdc.h"

/* Register access method. */
#define REG16(A)   (*(volatile UInt16 *) (A))
#define REG32(A)   (*(volatile UInt32 *) (A))

#define DSPEVENTID              5

#define DSPINT                  5

#define SYSCFG_BASEADDR (0x01c14000)
#define SYSCFG_CHIPSIG (SYSCFG_BASEADDR + 0x174)
#define SYSCFG_CHIPSIG_CLR (SYSCFG_BASEADDR + 0x178)
#define SYSCFG_CHIPINT0 (1 << 0)
#define SYSCFG_CHIPINT1 (1 << 1)
#define SYSCFG_CHIPINT2 (1 << 2)
#define SYSCFG_CHIPINT3 (1 << 3)

Fxn userFxn = NULL;

Void InterruptDsp_isr(UArg arg);

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*!
 *  ======== InterruptDsp_intEnable ========
 *  Enable remote processor interrupt
 */
Void InterruptDsp_intEnable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
//    REG32(MAILBOX_IRQENABLE_SET_DSP) = MAILBOX_REG_VAL(DSP_MBX);
    Hwi_enableInterrupt(DSPINT);
}

/*!
 *  ======== InterruptDsp_intDisable ========
 *  Disables remote processor interrupt
 */
Void InterruptDsp_intDisable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
//    REG32(MAILBOX_IRQENABLE_CLR_DSP) = MAILBOX_REG_VAL(DSP_MBX);
    Hwi_disableInterrupt(DSPINT);
}

/*!
 *  ======== InterruptDsp_intRegister ========
 */
Void InterruptDsp_intRegister(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo,
                              Fxn fxn, UArg arg)
{
    Hwi_Params  hwiAttrs;
    UInt        key;

    while (InterruptDsp_intClear(remoteProcId, intInfo) != InterruptDsp_INVALIDPAYLOAD);

    userFxn = fxn;

    /* Disable global interrupts */
    key = Hwi_disable();

    InterruptDsp_intClear(remoteProcId, intInfo);

    Hwi_Params_init(&hwiAttrs);

    hwiAttrs.eventId = DSPEVENTID;
    hwiAttrs.enableInt = FALSE;
    Hwi_create(DSPINT,
               (Hwi_FuncPtr)InterruptDsp_isr,
               &hwiAttrs,
               NULL);

    /* Enable the interrupt */
    Hwi_enableInterrupt(DSPINT);

    /* Enable the mailbox interrupt to the M3 core */
    InterruptDsp_intEnable(remoteProcId, intInfo);

    /* Restore global interrupts */
    Hwi_restore(key);
}

Void InterruptDsp_intUnregister(UInt16 remoteProcId,
                                IInterrupt_IntInfo *intInfo)
{
    Hwi_Handle hwiHandle;

    InterruptDsp_intDisable(remoteProcId, intInfo);

    hwiHandle = Hwi_getHandle(DSPINT);
    Hwi_delete(&hwiHandle);

    userFxn = NULL;
}

/*!
 *  ======== InterruptDsp_intSend ========
 *  Send interrupt to the remote processor
 */
Void InterruptDsp_intSend(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo,
                          UArg arg)
{
    if (remoteProcId == MultiProc_getId("HOST")) {
        REG32(SYSCFG_CHIPSIG) = SYSCFG_CHIPINT0;
    }
}

/*!
 *  ======== InterruptDsp_intClear ========
 *  Clear interrupt and return payload
 */
UInt InterruptDsp_intClear(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    UInt arg = InterruptDsp_INVALIDPAYLOAD;

    if ((REG32(SYSCFG_CHIPSIG) & SYSCFG_CHIPINT2) == SYSCFG_CHIPINT2) {
        /* dummy up a message payload */
        arg = 0;
        REG32(SYSCFG_CHIPSIG_CLR) = SYSCFG_CHIPINT2;
    }

    return (arg);
}

/*!
 *  ======== InterruptDsp_isr ========
 */
Void InterruptDsp_isr(UArg arg)
{
    UArg payload;

System_printf("InterruptDsp_isr:\n");

    payload = InterruptDsp_intClear(arg, NULL);
    if (payload != InterruptDsp_INVALIDPAYLOAD) {
System_printf("   calling userFxn: 0x%x\n", userFxn);
        userFxn(payload);
    }
}
