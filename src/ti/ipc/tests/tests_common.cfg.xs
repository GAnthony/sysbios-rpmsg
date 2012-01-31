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

var MessageQ  = xdc.useModule('ti.sdo.ipc.MessageQ');
MessageQ.traceFlag = true;
MessageQ.SetupTransportProxy = xdc.module('ti.ipc.transports.TransportVirtioSetup');

NameServer    = xdc.useModule("ti.sdo.utils.NameServer");
var nsRemote = xdc.useModule("ti.ipc.namesrv.NameServerRemoteRpmsg");
NameServer.SetupProxy = nsRemote;

xdc.loadPackage('ti.ipc.rpmsg');

/* Reduces code size, by only pulling in modules explicitly referenced: */
BIOS.libType    = BIOS.LibType_Custom;

/* Modules used in Power Management */
xdc.loadPackage('ti.pm');
var Power = xdc.useModule('ti.sysbios.family.arm.ducati.omap4430.Power');
Power.loadSegment = "PM_DATA";

var Idle = xdc.useModule('ti.sysbios.knl.Idle');
/* IpcPower idle function must be at the end */
Idle.addFunc('&IpcPower_idle');

var Assert = xdc.useModule('xdc.runtime.Assert');
var Defaults = xdc.useModule('xdc.runtime.Defaults');
var Diags = xdc.useModule('xdc.runtime.Diags');
var LoggerSys = xdc.useModule('xdc.runtime.LoggerSys');
var LoggerSysParams = new LoggerSys.Params();

/* Enable Logger: */
Defaults.common$.logger = LoggerSys.create(LoggerSysParams);

nsRemote.common$.diags_ENTRY = Diags.ALWAYS_ON;
nsRemote.common$.diags_INFO  = Diags.ALWAYS_ON;
nsRemote.common$.diags_EXIT  = Diags.ALWAYS_ON;

/* Enable runtime Diags_setMask() for non-XDC spec'd modules: */
var Text = xdc.useModule('xdc.runtime.Text');
Text.isLoaded = true;
var Registry = xdc.useModule('xdc.runtime.Registry');
Registry.common$.diags_ENTRY = Diags.RUNTIME_OFF;
Registry.common$.diags_EXIT  = Diags.RUNTIME_OFF;
Registry.common$.diags_INFO  = Diags.RUNTIME_OFF;
Registry.common$.diags_STATUS = Diags.RUNTIME_OFF;
Registry.common$.diags_USER1 = Diags.RUNTIME_OFF;
Diags.setMaskEnabled = true;

MessageQ.common$.diags_USER1= Diags.ALWAYS_OFF;

var TransportVirtio = xdc.useModule('ti.ipc.transports.TransportVirtio');
/*
TransportVirtio.common$.diags_ENTRY = Diags.ALWAYS_ON;
TransportVirtio.common$.diags_EXIT  = Diags.ALWAYS_ON;
TransportVirtio.common$.diags_INFO  = Diags.ALWAYS_ON;
TransportVirtio.common$.diags_STATUS = Diags.ALWAYS_ON;
*/

var Main = xdc.useModule('xdc.runtime.Main');
Main.common$.diags_ASSERT = Diags.ALWAYS_ON;
Main.common$.diags_INTERNAL = Diags.ALWAYS_OFF;
Main.common$.diags_USER1 = Diags.ALWAYS_OFF;

var Hwi = xdc.useModule('ti.sysbios.family.arm.m3.Hwi');
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
Program.global.NUMLOOPS = 100;  // was 10

/* Task that does the notify sending */
Program.global.tsk1 = Task.create('&tsk1_func');
Program.global.tsk1.instance.name = "tsk1";

