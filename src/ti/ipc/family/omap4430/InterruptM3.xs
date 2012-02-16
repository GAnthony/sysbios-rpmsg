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
 *  ======== InterruptM3.xs ========
 */

var Hwi         = null;
var InterruptM3 = null;
var Core        = null;
var MultiProc   = null;

/*
 *  ======== module$use ========
 */
function module$use()
{   
    InterruptM3     = this;

    Hwi         = xdc.useModule("ti.sysbios.family.arm.m3.Hwi");
    Core        = xdc.useModule("ti.sysbios.family.arm.ducati.Core");
    MultiProc   = xdc.useModule("ti.sdo.utils.MultiProc");

    this.hostProcId   = MultiProc.getIdMeta("HOST");
    this.sysm3ProcId  = MultiProc.getIdMeta("CORE0");
    this.appm3ProcId  = MultiProc.getIdMeta("CORE1");
    this.dspProcId    = MultiProc.getIdMeta("DSP");
}

/*
 *  ======== module$static$init ========
 */
function module$static$init(mod, params)
{
    var fxnTable = InterruptM3.$object.fxnTable;
    var MultiProc = xdc.module('ti.sdo.utils.MultiProc');
    
    /* The function table length should be the number of processors */
    fxnTable.length = MultiProc.numProcessors;
    for (var i = 0; i < fxnTable.length; i++) {
        fxnTable[i].func = null;
        fxnTable[i].arg = 0;
    }

    mod.numPlugged = 0;
}

