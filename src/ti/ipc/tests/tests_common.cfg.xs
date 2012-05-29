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
 *  The SysMin used here vs StdMin, as trace buffer address is required for
 *  Linux trace debug driver, plus provides better performance.
 */
var System      = xdc.useModule('xdc.runtime.System');
var SysMin      = xdc.useModule('xdc.runtime.SysMin');
System.SupportProxy = SysMin;
SysMin.bufSize  = 0x8000;

var Memory   = xdc.useModule('xdc.runtime.Memory');

/* Modules used in the virtqueue/TransportVirtio/ServiceMgr libraries: */
var Semaphore   = xdc.useModule('ti.sysbios.knl.Semaphore');
var BIOS        = xdc.useModule('ti.sysbios.BIOS');
BIOS.heapSize   = 0x20000;

var SyncSem   = xdc.useModule('ti.sysbios.syncs.SyncSem');
var HeapBuf   = xdc.useModule('ti.sysbios.heaps.HeapBuf');
var List      = xdc.useModule('ti.sdo.utils.List');
var GateSwi   = xdc.useModule('ti.sysbios.gates.GateSwi');
var Task      = xdc.useModule('ti.sysbios.knl.Task');
Task.deleteTerminatedTasks = true;

var Thread = xdc.useModule('xdc.runtime.knl.Thread');

var MessageQ  = xdc.useModule('ti.sdo.ipc.MessageQ');
MessageQ.traceFlag = true;
MessageQ.SetupTransportProxy = xdc.module('ti.ipc.transports.TransportVirtioSetup');

NameServer    = xdc.useModule("ti.sdo.utils.NameServer");
var nsRemote = xdc.useModule("ti.ipc.namesrv.NameServerRemoteRpmsg");
NameServer.SetupProxy = nsRemote;

xdc.loadPackage('ti.ipc.namesrv');
if (Program.platformName.match(/OMAPL138/)) {
    xdc.loadPackage('ti.ipc.family.omapl138');
}
else {
    xdc.loadPackage('ti.ipc.family.omap4430');
}

/* Reduces code size, by only pulling in modules explicitly referenced: */
BIOS.libType    = BIOS.LibType_Custom;

if (!Program.platformName.match(/OMAPL138/)) {
    /* Modules used in Power Management */
    xdc.loadPackage('ti.pm');
    var Power = xdc.useModule('ti.sysbios.family.arm.ducati.omap4430.Power');
    Power.loadSegment = "PM_DATA";

    var Idle = xdc.useModule('ti.sysbios.knl.Idle');
    /* IpcPower idle function must be at the end */
    Idle.addFunc('&IpcPower_idle');
}

var Assert = xdc.useModule('xdc.runtime.Assert');
var Defaults = xdc.useModule('xdc.runtime.Defaults');
var Diags = xdc.useModule('xdc.runtime.Diags');
var LoggerSys = xdc.useModule('xdc.runtime.LoggerSys');
var LoggerSysParams = new LoggerSys.Params();

/* Enable Logger: */
Defaults.common$.logger = LoggerSys.create(LoggerSysParams);

nsRemote.common$.diags_ENTRY = Diags.ALWAYS_OFF;
nsRemote.common$.diags_INFO  = Diags.ALWAYS_OFF;
nsRemote.common$.diags_EXIT  = Diags.ALWAYS_OFF;

/* Enable runtime Diags_setMask() for non-XDC spec'd modules: */
var Text = xdc.useModule('xdc.runtime.Text');
Text.isLoaded = true;
var Registry = xdc.useModule('xdc.runtime.Registry');
Registry.common$.diags_ENTRY = Diags.RUNTIME_ON;
Registry.common$.diags_EXIT  = Diags.RUNTIME_ON;
Registry.common$.diags_INFO  = Diags.RUNTIME_ON;
Registry.common$.diags_STATUS = Diags.RUNTIME_ON;
Registry.common$.diags_USER1 = Diags.RUNTIME_ON;
Diags.setMaskEnabled = true;

MessageQ.common$.diags_USER1= Diags.ALWAYS_OFF;

var TransportVirtio = xdc.useModule('ti.ipc.transports.TransportVirtio');
TransportVirtio.common$.diags_ENTRY = Diags.ALWAYS_OFF;
TransportVirtio.common$.diags_EXIT  = Diags.ALWAYS_OFF;
TransportVirtio.common$.diags_INFO  = Diags.ALWAYS_OFF;
TransportVirtio.common$.diags_STATUS = Diags.ALWAYS_OFF;
TransportVirtio.common$.diags_USER1 = Diags.ALWAYS_OFF;

if (Program.platformName.match(/OMAPL138/)) {
    var VirtQueue = xdc.useModule('ti.ipc.family.omapl138.VirtQueue');
    VirtQueue.common$.diags_ENTRY = Diags.ALWAYS_OFF;
    VirtQueue.common$.diags_EXIT  = Diags.ALWAYS_OFF;
    VirtQueue.common$.diags_USER1 = Diags.ALWAYS_OFF;

    var InterruptDsp = xdc.useModule('ti.ipc.family.omapl138.InterruptDsp');
    InterruptDsp.common$.diags_ENTRY = Diags.ALWAYS_OFF;
    InterruptDsp.common$.diags_EXIT  = Diags.ALWAYS_OFF;
    InterruptDsp.common$.diags_USER1 = Diags.ALWAYS_OFF;
}
else {
    var VirtQueue = xdc.useModule('ti.ipc.family.omap4430.VirtQueue');
    VirtQueue.common$.diags_ENTRY = Diags.ALWAYS_OFF;
    VirtQueue.common$.diags_EXIT  = Diags.ALWAYS_OFF;
    VirtQueue.common$.diags_USER1 = Diags.ALWAYS_OFF;

    var InterruptM3 = xdc.useModule('ti.ipc.family.omap4430.InterruptM3');
    InterruptM3.common$.diags_ENTRY = Diags.ALWAYS_OFF;
    InterruptM3.common$.diags_EXIT  = Diags.ALWAYS_OFF;
    InterruptM3.common$.diags_USER1 = Diags.ALWAYS_OFF;
}

var Main = xdc.useModule('xdc.runtime.Main');
Main.common$.diags_ASSERT = Diags.ALWAYS_OFF;
Main.common$.diags_INTERNAL = Diags.ALWAYS_OFF;
Main.common$.diags_USER1 = Diags.ALWAYS_OFF;

if (Program.platformName.match(/OMAPL138/)) {
    var Hwi = xdc.useModule('ti.sysbios.family.c64p.Hwi');
}
else {
    var Hwi = xdc.useModule('ti.sysbios.family.arm.m3.Hwi');
}
Hwi.enableException = true;

/*
 *  Common constants:
 */
Program.global.SLAVE_MESSAGEQNAME = "SLAVE";
Program.global.HOST_MESSAGEQNAME = "HOST";
Program.global.HEAP_NAME    = "myHeap";
Program.global.HEAP_ALIGN   =     8;
Program.global.HEAP_MSGSIZE =    64;
Program.global.HEAP_NUMMSGS =    256; // Worst case: # recv msgs in vring
Program.global.HEAPID       =     0;

/* Number of times to run the loop */
Program.global.NUMLOOPS = 100;  // was 100

/* Initial task in system.  It is set at high priority for messageq_multi.c*/
var params = new Task.Params;
params.instance.name = "tsk0";
params.priority = 5;
Program.global.tsk1 = Task.create('&tsk1Fxn', params);

