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
 * 
 */
/*
 *  ======== InterruptM3.xdc ========
 */

import ti.sdo.utils.MultiProc;

/*!
 *  ======== InterruptM3 ======== 
 *  OMAP4430/Ducati IPC interrupt manager
 */

module InterruptM3 inherits ti.sdo.ipc.notifyDrivers.IInterrupt
{

internal:

    /*! Function table */
    struct FxnTable {
        Fxn    func;
        UArg   arg;
    }
    
    /*!
     *  ======== intShmStub ========
     *  Stub function plugged as interrupt handler
     */
    Void intShmStub(UArg arg);

    struct Module_State {
        FxnTable   fxnTable[];  /* One entry for each core */
        UInt       numPlugged;  /* # of times the interrupt was registered */
    };

    /*! Statically retrieve procIds to avoid doing this at runtime */
    config UInt hostProcId  = MultiProc.INVALIDID;
    config UInt sysm3ProcId = MultiProc.INVALIDID;
    config UInt appm3ProcId = MultiProc.INVALIDID;
    config UInt dspProcId   = MultiProc.INVALIDID;
}

