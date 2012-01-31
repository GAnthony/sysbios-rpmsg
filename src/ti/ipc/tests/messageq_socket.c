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
 *  ======== messageq_socket.c ========
 *
 *  Test for messageq over sockets.
 *
 *  Requires:
 *      tools/messageq_socket in rpmsg_3.2_rc4 branch of upstream-rpmsg.
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
#include <ti/ipc/transports/_TransportVirtio.h>

/*  ----------------------------------- BIOS6 module Headers         */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/heaps/HeapBuf.h>

/*  ----------------------------------- To get globals from .cfg Header */
#include <xdc/cfg/global.h>

typedef unsigned int u32;
#include <ti/resources/rsc_table.h>

#define RPMSG_MESSAGEQ_PORT         61

void myNameMap_register(Char * name, UInt32 port)
{
    System_printf("registering %s service on %d with HOST\n", name, port);
    sendRpmsg(name, port, RPMSG_NS_CREATE);
}

/*
 * This to get TransportVirtio_attach() and NameServerRemoteRpmsg_attach()
 * called in lieu of using Ipc_start().
 * Must be done after BIOS_start(), as TransportVirtio startup relies on
 * passing an interrupt handshake.
 */
void myIpcStart(UInt procId)
{
    Int     status;

    /* call TransportVirtioSetup to attach to remote processor */
    status = TransportVirtioSetup_attach(procId, 0);
    Assert_isTrue(status >= 0, NULL);

    /* call NameServer_attach to remote processor */
    status = ti_sdo_utils_NameServer_SetupProxy_attach(procId, 0);
    Assert_isTrue(status >= 0, NULL);

    /*
     * Wait for other side to create his nameservice.
     * We really need a handshake in this virtio layer.
     */
    System_printf("Task Sleep...\n");
    Task_sleep(1000);

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
 *  ======== tsk1_func ========
 *  Receive and return messages
 */
Void tsk1_func(UArg arg0, UArg arg1)
{
    MessageQ_Msg     getMsg;
    MessageQ_Handle  messageQ;
    MessageQ_QueueId remoteQueueId;
    Int              status;
    UInt16           msgId = 0;
    UInt             procId = MultiProc_getId("HOST");

    System_printf("tsk1_func: In tsk1_func.\n");

    /* Get our Transport loaded in absence of Ipc module: */
    myIpcStart(procId);

    /* Create a message queue. Using SyncSem as the synchronizer */
    messageQ = MessageQ_create(SLAVE_MESSAGEQNAME, NULL);
    if (messageQ == NULL) {
        System_abort("MessageQ_create failed\n" );
    }

    remoteQueueId = MessageQ_getQueueId(messageQ);
    System_printf("tsk1_func: created MessageQ: %s; QueueID: 0x%x\n",
	SLAVE_MESSAGEQNAME, MessageQ_getQueueId(messageQ));

#if 1   // TBD: Need to implement NameServer.
    /* Open the remote message queue. Spin until it is ready. */
    do {
        System_printf("tsk1_func: Calling MessageQ_open...\n");
        status = MessageQ_open(HOST_MESSAGEQNAME, &remoteQueueId);
    }
    while (status != MessageQ_S_SUCCESS);

    System_printf("tsk1_func: Remote MessageQ %s; QueueID: 0x%x\n",
	HOST_MESSAGEQNAME, remoteQueueId);
#else
    /* No NameServer yet, so assume QueueIndex is same on both M3's: */
    /* Force procId to be the destination: */
    remoteQueueId = (remoteQueueId & 0x0000FFFF) | (procId << 16);
    System_printf("tsk1_func: remoteQueueId: 0x%x\n",
            remoteQueueId);
#endif

    System_printf("Start the main loop\n");
    while (msgId < NUMLOOPS) {
        /* Get a message */
        status = MessageQ_get(messageQ, &getMsg, MessageQ_FOREVER);
        if (status != MessageQ_S_SUCCESS) {
           System_abort("This should not happen since timeout is forever\n");
        }

#ifndef BENCHMARK
        System_printf("Received message #%d from core %d\n",
                     MessageQ_getMsgId(getMsg), procId);
#endif
        /* test id of message received */
        if (MessageQ_getMsgId(getMsg) != msgId) {
            System_abort("The id received is incorrect!\n");
        }

#ifndef BENCHMARK
        /* Send it back */
        System_printf("Sending message Id #%d to core %d\n", msgId, procId);
#endif
        status = MessageQ_put(remoteQueueId, getMsg);
        if (status != MessageQ_S_SUCCESS) {
           System_abort("MessageQ_put had a failure/error\n");
        }
        msgId++;
    }

    System_printf("Test complete!\n");
    System_exit(0);
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

    BIOS_start();
    return (0);
}
