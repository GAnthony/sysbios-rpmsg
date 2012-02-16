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
 * */
/*
 *  ======== Interrupt.c ========
 *  OMAP4430/Ducati Interrupt Manger
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>

#include <ti/sysbios/family/c64p/Hwi.h>

#include <ti/sdo/ipc/_Ipc.h>
#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>
#include <ti/sdo/utils/_MultiProc.h>
#include <ti/sdo/ipc/_SharedRegion.h>

#include <ti/sdo/ipc/family/c647x/MultiProcSetup.h>

#include "package/internal/Interrupt.xdc.h"


/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*
 *  ======== Interrupt_intEnable ========
 *  Enable interrupt
 */     
Void Interrupt_intEnable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    Hwi_enableInterrupt(intInfo->intVectorId);
}
        
/*
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
    UInt key, i;
    Hwi_Params hwiAttrs;
    Interrupt_FxnTable *table;

    /* Disable global interrupts */
    key = Hwi_disable();

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
        for (i = 0; i < ti_sdo_utils_MultiProc_numProcessors; i++) {
            Interrupt_intClear(i, NULL);
        }

        /* Register interrupt to remote processor */
        Hwi_Params_init(&hwiAttrs);
        hwiAttrs.maskSetting = Hwi_MaskingOption_SELF;
        hwiAttrs.arg         = arg;
        hwiAttrs.eventId     = Interrupt_INTERDSPINT;

        Hwi_create(intInfo->intVectorId,
            (Hwi_FuncPtr)Interrupt_intShmStub, &hwiAttrs, NULL);

        Hwi_enableInterrupt(intInfo->intVectorId);
    }

    /* Restore global interrupts */
    Hwi_restore(key);
}

/*
 *  ======== Interrupt_intUnregister ========
 */
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

/*
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
        * IPCGRH register */
      *ipcgrh = (val | arg); /* Only most significant 4 bits of arg have control bits, must
                                         * find good way to mask rest of bits before ORing with val */
    }
    else
    {
      /* Interrupt is to be generated on another DSP. */
      ipcgr[MultiProcSetup_procMap[remoteProcId]] =  val;
    }
}

/*
 *  ======== Interrupt_intPost ========
 *  Send interrupt to the remote processor
 */
Void Interrupt_intPost(UInt16 srcProcId, IInterrupt_IntInfo *intInfo,
                       UArg arg)
{
    UInt32 val;
    extern volatile cregister Uns DNUM;
    volatile UInt32 *ipcgr = (volatile UInt32 *)Interrupt_IPCGR0;

    /*
     *  bit 0 is set to generate interrupt.
     *  bits 4-7 is set to specify the interrupt generation source.
     *  The convention is that bit 4 (SRCS0) is used for core 0,
     *  bit 5 (SRCS1) for core 1, etc... .
     */
    val = (1 << (MultiProcSetup_procMap[srcProcId] + Interrupt_SRCSx_SHIFT)) | 1;

    ipcgr[DNUM] =  val;
}

/*
 *  ======== Interrupt_intClear ========
 *  Acknowledge interrupt by clearing the corresponding source bit.
 *  Does not clear the IFR bit by way of ICR write because that should
 *  only be done during init time.
 */
UInt Interrupt_intClear(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    extern volatile cregister Uns DNUM;
    volatile UInt32 *ipcar = (volatile UInt32 *)Interrupt_IPCAR0;

    ipcar[DNUM] =  (1 << (MultiProcSetup_procMap[remoteProcId] + 
            Interrupt_SRCSx_SHIFT));

    /* If interrupt originated at "HOST" clear signalling bits too */
    if (remoteProcId == MultiProc_getId("HOST"))
    {
      ipcar[DNUM] = 0xF0000000;  /* Hardcoded for now */
    }

    return (0);
}

/*
 *************************************************************************
 *                      Internals functions
 *************************************************************************
 */

/*
 *  ======== Interrupt_intShmStub ========
 */
Void Interrupt_intShmStub(UArg arg)
{
    Int i;
    Interrupt_FxnTable *table;
    extern volatile cregister Uns DNUM;
    volatile UInt32 *ipcar = (volatile UInt32 *)Interrupt_IPCAR0;

    for (i = 0; i < ti_sdo_utils_MultiProc_numProcessors; i++) {
        if ((ipcar[DNUM]) & (1 << (MultiProcSetup_procMap[i] 
            + Interrupt_SRCSx_SHIFT))) {
            table = &(Interrupt_module->fxnTable[i]);
            
            if (table->func != NULL) 
            {
                if (i == MultiProc_getId("HOST"))
                {
                  /* If interrupt source is the Host provide the value of the IPCARx
                    * register to the service routine */
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

/*
 *  @(#) ti.sdo.ipc.family.c647x; 1, 0, 0, 0,4; 10-19-2011 10:45:46; /db/vtree/library/trees/ipc/ipc.git/src/ null
 */

