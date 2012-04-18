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
 *  ======== messageq_multi.c ========
 *
 *  Test for messageq operating in multiple simultaneous threads.
 *
 */

#include <xdc/std.h>
#include <string.h>

/*  -----------------------------------XDC.RUNTIME module Headers    */
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Diags.h>

/*  ----------------------------------- IPC module Headers           */
#include <ti/sdo/utils/_NameServer.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/transports/TransportVirtioSetup.h>
#include <ti/ipc/rpmsg/rpmsg.h>
#include <ti/ipc/transports/_TransportVirtio.h>

/*  ----------------------------------- BIOS6 module Headers         */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>

/*  ----------------------------------- To get globals from .cfg Header */
#include <xdc/cfg/global.h>

typedef unsigned int u32;
#include <ti/resources/rsc_table.h>

static int numTests = 0;

void myNameMap_register(Char * name, UInt32 port)
{
    System_printf("registering %s service on %d with HOST\n", name, port);
    nameService_register(name, port, RPMSG_NS_CREATE);
}

void myNameMap_unregister(Char * name, UInt32 port)
{
    System_printf("unregistering %s service on %d with HOST\n", name, port);
    nameService_register(name, port, RPMSG_NS_DESTROY);
}

/*
 * This to get TransportVirtio_attach() and NameServerRemoteRpmsg_attach()
 * called in lieu of using Ipc_start().
 * Must be done after BIOS_start(), as TransportVirtio startup relies on
 * passing an interrupt handshake.
 */
void myIpcAttach(UInt procId)
{
    Int     status;

    /* call TransportVirtioSetup to attach to remote processor */
    status = TransportVirtioSetup_attach(procId, 0);
    Assert_isTrue(status >= 0, NULL);

    /* call NameServer_attach to remote processor */
    status = ti_sdo_utils_NameServer_SetupProxy_attach(procId, 0);
    Assert_isTrue(status >= 0, NULL);

    /*
     * Tell the Linux host we have a MessageQ service over rpmsg.
     *
     * TBD: This should be in the VirtioTransport initialization, but we need
     * an interrupt handshake after BIOS_start().
     * TBD: Also, NameMap should go over a bare rpmsg API, rather than
     * MessageQCopy, as this clashes with MessageQ.
     */
    myNameMap_register("rpmsg-proto", RPMSG_MESSAGEQ_PORT);
}

/*
 * This to get TransportVirtio_detach() and NameServerRemoteRpmsg_detach()
 * called in lieu of using Ipc_start().
 */
void myIpcDetach(UInt procId)
{
    Int     status;

    /* call TransportVirtioSetup to detach from remote processor */
    status = TransportVirtioSetup_detach(procId);
    Assert_isTrue(status >= 0, NULL);

    /* call NameServer_detach from remote processor */
    status = ti_sdo_utils_NameServer_SetupProxy_detach(procId);
    Assert_isTrue(status >= 0, NULL);

    /*
     * Tell the host MessageQ service over rpmsg is going away.
     *
     * TBD: This should be in the VirtioTransport module.
     * Also, NameMap should go over a bare rpmsg API, rather than
     * MessageQCopy, as this clashes with MessageQ.
     */
    myNameMap_unregister("rpmsg-proto", RPMSG_MESSAGEQ_PORT);
}

/*
 *  ======== tsk1_func ========
 *  Do the IpcAttach/IpcDetach once at end of test 
 */
Void tsk1_func(UArg arg0, UArg arg1)
{
    UInt             procId = MultiProc_getId("HOST");

    System_printf("tsk1_func: In tsk1_func.\n");

    /* Get our Transport loaded in absence of Ipc module: */
    myIpcAttach(procId);

    /* Wait for end of test: */
    while (numTests < NUMLOOPS * NUMTHREADS) {
        Task_sleep(10000);
    }

    myIpcDetach(procId);

    System_printf("Multi Thread Test complete!\n");
    System_exit(0);
}

/*
 *  ======== loopback_fxn========
 *  Receive and return messages.
 *  Run at priority lower than tsk1_func above.
 *  Inputs:
 *     - arg0: number of the thread, appended to MessageQ host and slave names.
 */
Void loopback_fxn (UArg arg0, UArg arg1)
{
    MessageQ_Msg     getMsg;
    MessageQ_Handle  messageQ;
    MessageQ_QueueId remoteQueueId;
    Int              status;
    UInt16           msgId = 0;
    UInt             procId = MultiProc_getId("HOST");
    Char             localQueueName[64];
    Char             hostQueueName[64];

    System_printf("Thread loopback_fxn: %d\n", arg0);

    System_sprintf(localQueueName, "%s_%d", SLAVE_MESSAGEQNAME, arg0);
    System_sprintf(hostQueueName,  "%s_%d", HOST_MESSAGEQNAME,  arg0);

    /* Create a message queue. */
    messageQ = MessageQ_create(localQueueName, NULL);
    if (messageQ == NULL) {
        System_abort("MessageQ_create failed\n" );
    }

    remoteQueueId = MessageQ_getQueueId(messageQ);
    System_printf("loopback_fxn: created MessageQ: %s; QueueID: 0x%x\n",
	localQueueName, MessageQ_getQueueId(messageQ));

    /* Open the remote message queue. Spin until it is ready. */
    System_printf("loopback_fxn: Calling MessageQ_open...\n");
    do {
        status = MessageQ_open(hostQueueName, &remoteQueueId);
        /* 1 second sleep: */
        Task_sleep(1000);
    }
    while (status != MessageQ_S_SUCCESS);

    System_printf("loopback_fxn: Remote MessageQ %s; QueueID: 0x%x\n",
	hostQueueName, remoteQueueId);

    System_printf("Start the main loop: %d\n", arg0);
    while (msgId < NUMLOOPS) {
        /* Get a message */
        status = MessageQ_get(messageQ, &getMsg, MessageQ_FOREVER);
        if (status != MessageQ_S_SUCCESS) {
           System_abort("This should not happen since timeout is forever\n");
        }

#ifndef BENCHMARK
        System_printf("%d: Received message #%d from core %d\n",
                     arg0, MessageQ_getMsgId(getMsg), procId);
#endif
        /* test id of message received */
        if (MessageQ_getMsgId(getMsg) != msgId) {
            System_abort("The id received is incorrect!\n");
        }

#ifndef BENCHMARK
        /* Send it back */
        System_printf("%d: Sending message Id #%d to core %d\n", 
                      arg0, msgId, procId);
#endif
        status = MessageQ_put(remoteQueueId, getMsg);
        if (status != MessageQ_S_SUCCESS) {
           System_abort("MessageQ_put had a failure/error\n");
        }
        msgId++;
    }
    
    numTests += NUMLOOPS;

    System_printf("Test thread %d complete!\n", arg0);
}

/*
 *  ======== main ========
 */
Int main(Int argc, Char* argv[])
{
    Error_Block            eb;
    Ptr                    buf;
    HeapBuf_Handle         heapHandle;
    HeapBuf_Params         heapBufParams;
    Int                    i;
    Task_Params            params;

    System_printf("%d resources at 0x%x\n",
                  sizeof(resources) / sizeof(struct resource), resources);

    /* Initialize the Error_Block. This is required before using it */
    Error_init(&eb);

    System_printf("main: MultiProc id = %d\n", MultiProc_self());

    buf = Memory_alloc(0, (HEAP_NUMMSGS * HEAP_MSGSIZE) + HEAP_ALIGN, 8, &eb);

    /*
     *  Create the heap that will be used to allocate messages.
     */
    HeapBuf_Params_init(&heapBufParams);
    heapBufParams.align          = 8;
    heapBufParams.numBlocks      = HEAP_NUMMSGS;
    heapBufParams.blockSize      = HEAP_MSGSIZE;
    heapBufParams.bufSize        = HEAP_NUMMSGS * HEAP_MSGSIZE;
    heapBufParams.buf            = buf;
    heapHandle = HeapBuf_create(&heapBufParams, &eb);
    if (heapHandle == NULL) {
        System_abort("HeapBuf_create failed\n" );
    }

    /* Register this heap with MessageQ */
    MessageQ_registerHeap((IHeap_Handle)(heapHandle), HEAPID);

    /* Create N threads to correspond with host side N thread test app: */
    Task_Params_init(&params);
    params.priority = 3;
    for (i = 0; i < NUMTHREADS; i++) {
        params.arg0 = i;
        Task_create(loopback_fxn, &params, NULL);
    }

    BIOS_start();
    return (0);
 }
