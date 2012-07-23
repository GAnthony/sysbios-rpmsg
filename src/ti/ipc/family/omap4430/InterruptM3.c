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
 *  ======== InterruptM3.c ========
 *  OMAP4430/Ducati Interrupt Manger
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/ducati/Core.h>

#include <ti/ipc/MultiProc.h>

#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>

#include "InterruptM3.h"

#include "package/internal/InterruptM3.xdc.h"

/* Register access method. */
#define REG16(A)   (*(volatile UInt16 *) (A))
#define REG32(A)   (*(volatile UInt32 *) (A))

#define HOSTINT                 26
#define DSPINT                  55
#define M3INT_MBX               50
#define M3INT                   19

/* Assigned mailboxes */
#define SYSM3_MBX               0
#define HOST_MBX                1
#define APPM3_MBX               2
#define DSP_MBX                 3

#define MAILBOX_BASEADDR        (0xAA0F4000)

#define MAILBOX_MESSAGE(M)      (MAILBOX_BASEADDR + 0x040 + (0x4 * M))
#define MAILBOX_FIFOSTATUS(M)   (MAILBOX_BASEADDR + 0x080 + (0x4 * M))
#define MAILBOX_STATUS(M)       (MAILBOX_BASEADDR + 0x0C0 + (0x4 * M))
#define MAILBOX_REG_VAL(M)      (0x1 << (2 * M))

#define MAILBOX_IRQSTATUS_CLR_M3    (MAILBOX_BASEADDR + 0x124)
#define MAILBOX_IRQENABLE_SET_M3    (MAILBOX_BASEADDR + 0x128)
#define MAILBOX_IRQENABLE_CLR_M3    (MAILBOX_BASEADDR + 0x12C)

/*
 *  Ducati control register that maintains inter-core interrupt bits.
 *
 *  Using separate core0 and core1 values to do 16-bit reads/writes
 *  because we do not want to overwrite the other cores value.
 */
#define INTERRUPT_CORE_0       (0x40001000)
#define INTERRUPT_CORE_1       (0x40001000 + 2)

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*!
 *  ======== InterruptM3_intEnable ========
 *  Enable remote processor interrupt
 */
Void InterruptM3_intEnable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    /*
     *  If the remote processor communicates via mailboxes, we should enable
     *  the Mailbox IRQ instead of enabling the Hwi because multiple mailboxes
     *  share the same Hwi
     */
    if (Core_getId() == 0) {
        REG32(MAILBOX_IRQENABLE_SET_M3) = MAILBOX_REG_VAL(SYSM3_MBX);
    }
    else {
        Hwi_enableInterrupt(M3INT);
    }
}

/*!
 *  ======== InterruptM3_intDisable ========
 *  Disables remote processor interrupt
 */
Void InterruptM3_intDisable(UInt16 remoteProcId, 
                                IInterrupt_IntInfo *intInfo)
{
    /*
     *  If the remote processor communicates via mailboxes, we should disable
     *  the Mailbox IRQ instead of disabling the Hwi because multiple mailboxes
     *  share the same Hwi
     */
    if (Core_getId() == 0) {
        REG32(MAILBOX_IRQENABLE_CLR_M3) = MAILBOX_REG_VAL(SYSM3_MBX);
    }
    else {
        Hwi_disableInterrupt(M3INT);
    }
}

/*!
 *  ======== InterruptM3_intRegister ========
 */
Void InterruptM3_intRegister(UInt16 remoteProcId,
                                 IInterrupt_IntInfo *intInfo,
                                 Fxn func, UArg arg)
{
    Hwi_Params  hwiAttrs;
    UInt        key;
    Int         index;
    InterruptM3_FxnTable *table;

    /* Ensure that remoteProcId is valid */
    Assert_isTrue(remoteProcId < MultiProc_getNumProcessors(), 
            NULL);

    if (remoteProcId == InterruptM3_dspProcId) {
        index = 0;
    }
    else if (remoteProcId == InterruptM3_hostProcId) {
        index = 1;
    }
    else {
        /* Going to the other M3 */
        index = 2;
    }

    Log_print1(Diags_USER1,
            "InterruptM3_intRegister: table index = %d", (IArg)index);

    /* Disable global interrupts */
    key = Hwi_disable();

    table = &(InterruptM3_module->fxnTable[index]);
    table->func = func;
    table->arg  = arg;

    Hwi_Params_init(&hwiAttrs);
    hwiAttrs.maskSetting = Hwi_MaskingOption_LOWER;
    hwiAttrs.arg = remoteProcId;

    if (Core_getId() == 0) {
        Hwi_create(M3INT_MBX,
                   (Hwi_FuncPtr) InterruptM3_intShmStub,
                   &hwiAttrs,
                   NULL);
        /* InterruptM3_intEnable won't enable the Hwi */
        Hwi_enableInterrupt(M3INT_MBX);
    }
    else {
        InterruptM3_module->numPlugged++;
        if (InterruptM3_module->numPlugged == 1) {
        Hwi_create(M3INT,
                       (Hwi_FuncPtr) InterruptM3_intShmStub,
                   &hwiAttrs,
                   NULL);
        }
    }

    /* Enable the mailbox interrupt to the M3 core */
    InterruptM3_intEnable(remoteProcId, intInfo);

    /* Restore global interrupts */
    Hwi_restore(key);
}

/*!
 *  ======== InterruptM3_intUnregister ========
 */
Void InterruptM3_intUnregister(UInt16 remoteProcId,
                                   IInterrupt_IntInfo *intInfo)
{
    Hwi_Handle hwiHandle;
    Int index;
    InterruptM3_FxnTable *table;

    if (remoteProcId == InterruptM3_dspProcId) {
        index = 0;
    }
    else if (remoteProcId == InterruptM3_hostProcId) {
        index = 1;
    }
    else {
        /* Going to the other M3 */
        index = 2;
    }

    /* Disable the mailbox interrupt source */
    InterruptM3_intDisable(remoteProcId, intInfo);

    /* Delete/disable the Hwi */
    if (Core_getId() == 0) {
        hwiHandle = Hwi_getHandle(M3INT);
        Hwi_delete(&hwiHandle);
    }
    else {
        InterruptM3_module->numPlugged--;
        if (InterruptM3_module->numPlugged == 0) {
            hwiHandle = Hwi_getHandle(M3INT_MBX);
            Hwi_delete(&hwiHandle);
        }
    }

    /* Clear the FxnTable entry for the remote processor */
    table = &(InterruptM3_module->fxnTable[index]);
    table->func = NULL;
    table->arg  = 0;
}
/*!
 *  ======== InterruptM3_intSend ========
 *  Send interrupt to the remote processor
 */
Void InterruptM3_intSend(UInt16 remoteProcId,
                             IInterrupt_IntInfo *intInfo,
                             UArg arg)
{

    Log_print2(Diags_USER1,
        "InterruptM3_intSend: Sending interrupt with payload 0x%x to proc #%d",
        (IArg)arg, (IArg)remoteProcId);
    if (remoteProcId == InterruptM3_sysm3ProcId) {
        while(REG32(MAILBOX_FIFOSTATUS(SYSM3_MBX)));
        REG32(MAILBOX_MESSAGE(SYSM3_MBX)) = arg;
    }
    else if (remoteProcId == InterruptM3_appm3ProcId) {
        while(REG32(MAILBOX_FIFOSTATUS(APPM3_MBX)));
        /* Write to the mailbox, but this won't trigger an interrupt */
        REG32(MAILBOX_MESSAGE(APPM3_MBX)) = arg;
        /* Actually trigger the interrupt */
        REG16(INTERRUPT_CORE_1) |= 0x1;
    }
    else if (remoteProcId == InterruptM3_dspProcId) {
        while(REG32(MAILBOX_FIFOSTATUS(DSP_MBX)));
        REG32(MAILBOX_MESSAGE(DSP_MBX)) = arg;
    }
    else if (remoteProcId == InterruptM3_hostProcId) {
        while(REG32(MAILBOX_FIFOSTATUS(HOST_MBX)));
        REG32(MAILBOX_MESSAGE(HOST_MBX)) = arg;
    }
    else {
        /* Should never get here */
        Assert_isTrue(FALSE, NULL);
    }
}

/*!
 *  ======== InterruptM3_intClear ========
 *  Clear interrupt and return payload
 */
UInt InterruptM3_intClear(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    UInt arg = InterruptM3_INVALIDPAYLOAD;

    /* First check whether incoming mailbox has a message */
    if (Core_getId() == 0) {
        /* If FIFO is empty, return InterruptM3_INVALIDPAYLOAD */
        if (REG32(MAILBOX_STATUS(SYSM3_MBX)) == 0) {
            return (arg);
        }
        else {
            /* If there is a message, return the argument to the caller */
            arg = REG32(MAILBOX_MESSAGE(SYSM3_MBX));
            REG32(MAILBOX_IRQSTATUS_CLR_M3) = MAILBOX_REG_VAL(SYSM3_MBX);
        }
    }
    else {
        /* Clear the inter-M3 interrupt if necessary */
        if ((REG16(INTERRUPT_CORE_1) & 0x1) == 0x1) {
            REG16(INTERRUPT_CORE_1) &= ~(0x1);
        }

        /* If FIFO is empty, return InterruptM3_INVALIDPAYLOAD */
        if (REG32(MAILBOX_STATUS(APPM3_MBX)) == 0) {
            return (arg);
        }
        else {
            /* If there is a message, return the argument to the caller */
            arg = REG32(MAILBOX_MESSAGE(APPM3_MBX));
            REG32(MAILBOX_IRQSTATUS_CLR_M3) = MAILBOX_REG_VAL(APPM3_MBX);

            if (REG32(MAILBOX_STATUS(APPM3_MBX)) != 0) {
                /* Trigger our own interrupt since another interrupt pending */
                REG16(INTERRUPT_CORE_1) |= 0x1;
            }
        }
    }

    return (arg);
}

/*
 *************************************************************************
 *                      Internal functions
 *************************************************************************
 */

/*!
 *  ======== InterruptM3_intShmStub ========
 *  Calls the function supplied by the user in intRegister
 */
Void InterruptM3_intShmStub(UArg arg)
{
    UArg payload;
    InterruptM3_FxnTable *table;

    payload = InterruptM3_intClear(arg, NULL);
    if (payload != InterruptM3_INVALIDPAYLOAD) {
        Log_print1(Diags_USER1,
            "InterruptM3_isr: Interrupt received, payload = 0x%x",
            (IArg)payload);
        /*
         * We don't know from which processor we got the interrupt, but
         * they all have the same ISR fxn.  So, use index=1 arbitrarily.
         */
        table = &(InterruptM3_module->fxnTable[1]);
        (table->func)(payload);
    }
}
