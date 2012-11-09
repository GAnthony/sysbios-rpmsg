/*
 * Copyright (c) 2012, Texas Instruments Incorporated
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
 *  C647x based interrupt manager.
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Startup.h>

#include <ti/sysbios/family/c64p/Hwi.h>

#include <ti/ipc/MultiProc.h>
#include <ti/sdo/ipc/family/c647x/MultiProcSetup.h>
#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>

#include "Interrupt.h"

#include "package/internal/Interrupt.xdc.h"


Fxn userFxn = NULL;
Void Interrupt_isr(UArg arg);

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*
 *  ======== Interrupt_Module_startup ========
 */
Int Interrupt_Module_startup(Int phase)
{
    volatile UInt32 *kick0 = (volatile UInt32 *)Interrupt_KICK0;
    volatile UInt32 *kick1 = (volatile UInt32 *)Interrupt_KICK1;
    UInt16 procId = MultiProc_self();
    extern volatile cregister Uns DNUM;

    /*
     *  Wait for Startup to be done (if MultiProc id not yet set) because a
     *  user fxn should set it
     */
    if (!Startup_Module_startupDone() && procId == MultiProc_INVALIDID) {
        return (Startup_NOTDONE);
    }

    if (!Interrupt_enableKick) {
        /* Do not unlock the kick registers */
        return (Startup_DONE);
    }

    /*
     * Only write to the KICK registers if:
     * - This core is the SR0 owner
     * - There is no SR0 and this core has procId '1' (IPC 3.x: this case).
     */
    /* TODO: What if CORE0 is not started, but the others are? */
    if (DNUM == 0) {
        if (Interrupt_KICK0 && Interrupt_KICK1){
            /* unlock the KICK mechanism in the Bootcfg MMRs if defined */
            kick0[0] = 0x83e70b13;      /* must be written with this value */
            kick1[0] = 0x95a4f1e0;      /* must be written with this value */
        }
    }

    return (Startup_DONE);
}

/*!
 *  ======== Interrupt_intEnable ========
 *  Enable interrupt
 */
Void Interrupt_intEnable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    Hwi_enableInterrupt(intInfo->intVectorId);
}

/*!
 *  ======== Interrupt_intDisable ========
 *  Disables interrupts
 */
Void Interrupt_intDisable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    Hwi_disableInterrupt(intInfo->intVectorId);
}

/*
 *  ======== Interrupt_intRegister ========
 *  Register ISR for remote processor interrupt
 */
Void Interrupt_intRegister(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo,
                           Fxn func, UArg arg)
{
    UInt i;
    Hwi_Params hwiAttrs;
    Interrupt_FxnTable *table;

    /* setup the function table using the same Hwi int */
    table = &(Interrupt_module->fxnTable[remoteProcId]);
    table->func = func;
    if (remoteProcId != MultiProc_getId("HOST"))
    {
      /* arg not used for interrupts from Host.  The value in the IPCAR register
        * will be used instead */
      table->arg  = arg;
    }

    /* Make sure the interrupt only gets plugged once */
    Interrupt_module->numPlugged++;
    if (Interrupt_module->numPlugged == 1) {
        /* Clear any pending interrupt */
        for (i = 0; i < MultiProc_getNumProcessors(); i++) {
            Interrupt_intClear(i, NULL);
        }

        /* Register interrupt to remote processor */
        Hwi_Params_init(&hwiAttrs);
        hwiAttrs.maskSetting = Hwi_MaskingOption_SELF;
        hwiAttrs.arg         = arg;
        hwiAttrs.eventId     = Interrupt_INTERDSPINT;

        Hwi_create(intInfo->intVectorId,
            (Hwi_FuncPtr)Interrupt_isr, &hwiAttrs, NULL);

        Hwi_enableInterrupt(intInfo->intVectorId);
    }
}

Void Interrupt_intUnregister(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    Hwi_Handle hwiHandle;
    Interrupt_FxnTable *table;

    Interrupt_module->numPlugged--;
    if (Interrupt_module->numPlugged == 0) {
        /* No need to disable interrupt: Hwi_delete takes care of this */
        hwiHandle = Hwi_getHandle(intInfo->intVectorId);
        Hwi_delete(&hwiHandle);
    }

    /* Unset the function table */
    table = &(Interrupt_module->fxnTable[remoteProcId]);
    table->func = NULL;
    table->arg  = 0;
}

/*!
 *  ======== Interrupt_intSend ========
 *  Send interrupt to the remote processor
 */
Void Interrupt_intSend(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo,
                          UArg arg)
{
    UInt32 val;
    extern volatile cregister Uns DNUM;
    volatile UInt32 *ipcgr = (volatile UInt32 *)Interrupt_IPCGR0;
    volatile UInt32 *ipcgrh = (volatile UInt32 *)Interrupt_IPCGRH;

    Log_print2(Diags_USER1,
        "Interrupt_intSend: Sending payload 0x%x to proc #%d",
        (IArg)arg, (IArg)remoteProcId);

    /*
     *  bit 0 is set to generate interrupt.
     *  bits 4-7 is set to specify the interrupt generation source.
     *  The convention is that bit 4 (SRCS0) is used for core 0,
     *  bit 5 (SRCS1) for core 1, etc... .
     */
    val = (1 << (DNUM + Interrupt_SRCSx_SHIFT)) | 1;

    if (remoteProcId == MultiProc_getId("HOST"))
    {
      /* Interrupt is to be generated on the Host processor.  Go through
       * IPCGRH register
       */
      /* TODO: Only most significant 4 bits of arg have control bits,
       * must find good way to mask rest of bits before ORing with val
       */
      *ipcgrh = (val | arg);
    }
    else
    {
      /* Interrupt is to be generated on another DSP. */
      /* OR: ? ipcgr[DNUM] =  val; */
      ipcgr[MultiProcSetup_procMap[remoteProcId]] =  val;
    }
}

#define IPCAR_INT_SRCS27     (0x80000000U)
/*
 *  ======== Interrupt_intClear ========
 *  Acknowledge interrupt by clearing the corresponding source bit.
 *
 *  For interrupt from HOST, clear SRCC27 (bit 32), and return 1.
 *  Otherwise, clear SRCC0-3 depending on MultiProc ID.
 */
UInt Interrupt_intClear(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    extern volatile cregister Uns DNUM;
    volatile UInt32 *ipcgr = (volatile UInt32 *)Interrupt_IPCGR0;
    volatile UInt32 *ipcar = (volatile UInt32 *)Interrupt_IPCAR0;
    UInt arg = Interrupt_INVALIDPAYLOAD;
    UInt val = ipcgr[DNUM];

    /* If interrupt originated at "HOST" clear signalling bits too */
    if (remoteProcId == MultiProc_getId("HOST"))
    {
        if (val & IPCAR_INT_SRCS27) {
            Log_print1(Diags_USER1, "Interrupt_intClear: ipcgr: 0x%x\n", val);
            arg = 1;  /* Dummy return value, not used. */
            /* Clear bit: Must match keystone_remoteproc kick. */
            ipcar[DNUM] = IPCAR_INT_SRCS27;
        }
    }
    else {
        ipcar[DNUM] =  (1 << (MultiProcSetup_procMap[remoteProcId] +
            Interrupt_SRCSx_SHIFT));
    }

    return (arg);
}

/*
 *  ======== Interrupt_isr ========
 */
Void Interrupt_isr(UArg arg)
{
    Int i;
    Interrupt_FxnTable *table;
    extern volatile cregister Uns DNUM;
    volatile UInt32 *ipcar = (volatile UInt32 *)Interrupt_IPCAR0;
    UArg payload;


    payload = ipcar[DNUM];
    Log_print1(Diags_USER1,
        "InterruptDsp_isr: Interrupt received, payload = 0x%x",
        (IArg)payload);

    for (i = 0; i < MultiProc_getNumProcessors(); i++) {
        if ((ipcar[DNUM]) & (1 << (MultiProcSetup_procMap[i]
            + Interrupt_SRCSx_SHIFT))) {
            table = &(Interrupt_module->fxnTable[i]);

            if (table->func != NULL)
            {
                if (i == MultiProc_getId("HOST"))
                {
                  /* If interrupt source is the Host provide the value of the
                   * IPCARx register to the service routine
                   */
                  (table->func)(ipcar[DNUM]);
                }
                else
                {
                  (table->func)(table->arg);
                }
            }
        }
    }
}
