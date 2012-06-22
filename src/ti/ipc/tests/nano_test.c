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
 *  ======== nano_test.c ========
 *
 *  Test for a particular customer use case.
 *
 *  See <syslink3_repo>/src/tests/nano_test.c for usage.
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
#include <ti/ipc/rpmsg/Rpmsg.h>
#include <ti/ipc/transports/_TransportVirtio.h>
#include <ti/ipc/family/omap4430/VirtQueue.h>

/*  ----------------------------------- BIOS6 module Headers         */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>

/*  ----------------------------------- To get globals from .cfg Header */
#include <xdc/cfg/global.h>

//#define VERBOSE 1

/* Numbers and sizes of input/output messages: */
#define NUM_SLAVE_MSGS_PER_HOST_MSG   4

#define INPUT_MSG_DATASIZE  (8192)
#define OUTPUT_MSG_DATASIZE (INPUT_MSG_DATASIZE / NUM_SLAVE_MSGS_PER_HOST_MSG)

typedef unsigned int u32;
#ifdef OMAPL138
#include <ti/resources/rsc_table_omapl138.h>
#else
#include <ti/resources/rsc_table.h>
#endif

/* Application message structures: */
typedef struct {
    MessageQ_MsgHeader	hdr;
    Char                *inBuf;
} InputMsg;

typedef struct {
    MessageQ_MsgHeader	hdr;
    Char                *outBuf;
} OutputMsg;

static OutputMsg        outMsg;

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
 *  ======== tsk1Fxn ========
 *  Receive and return messages
 */
Void tsk1Fxn(UArg arg0, UArg arg1)
{
    InputMsg         *inMsg;
    MessageQ_Handle  messageQ;
    MessageQ_QueueId remoteQueueId;
    Int              status;
    UInt16           msgId = 0;
    UInt             procId = MultiProc_getId("HOST");
    int              i;

    System_printf("tsk1Fxn: Entered\n");

    /* Get our Transport loaded in absence of Ipc module: */
    myIpcAttach(procId);

    /* Create a message queue. */
    messageQ = MessageQ_create(SLAVE_MESSAGEQNAME, NULL);
    if (messageQ == NULL) {
        System_abort("MessageQ_create failed\n" );
    }

    System_printf("tsk1Fxn: created MessageQ: %s; QueueID: 0x%x\n",
	SLAVE_MESSAGEQNAME, MessageQ_getQueueId(messageQ));

    /* Use a static message for outMsg: no need to call MessageQ_alloc(): */
    MessageQ_staticMsgInit((MessageQ_Msg)&outMsg, sizeof(OutputMsg));

    System_printf("Start the main loop\n");
    while (1) {
        /* Get one block (8Kb) of data passed as a pointer to shared memory */
        status = MessageQ_get(messageQ, (MessageQ_Msg *)&inMsg, 
                              MessageQ_FOREVER);
        if (status != MessageQ_S_SUCCESS) {
           System_abort("This should not happen since timeout is forever\n");
        }
        remoteQueueId = MessageQ_getReplyQueue(inMsg);

#ifdef VERBOSE
        System_printf ("Received msgId: %d, inBuf: 0x%x\n",
                        MessageQ_getMsgId(inMsg), inMsg->inBuf);
#endif
        /* test id of message received */
        if (MessageQ_getMsgId(inMsg) != msgId) {
            System_printf("Expected msgId: %d, got %d\n", 
                           MessageQ_getMsgId(inMsg), msgId);
            System_abort("Unexpected msgId received!\n");
        }

        for (i = 0; i < NUM_SLAVE_MSGS_PER_HOST_MSG; i++) {
            /* Send back the data in 4 chunks: */
            MessageQ_setMsgId ((MessageQ_Msg)&outMsg, i);

            /* Return pointer to ith chunk of data: */
            outMsg.outBuf = inMsg->inBuf + i * OUTPUT_MSG_DATASIZE;

#ifdef VERBOSE
            System_printf("Sending msgId: %d, outBuf: 0x%x\n",
                   i, outMsg.outBuf);
#endif
            status = MessageQ_put(remoteQueueId, (MessageQ_Msg)&outMsg);
            if (status != MessageQ_S_SUCCESS) {
               System_abort("MessageQ_put had a failure/error\n");
            }
        }
        MessageQ_free ((MessageQ_Msg)inMsg);
        msgId++;
    }
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

    System_printf("%d resources at 0x%x\n", resources.num, resources);

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
