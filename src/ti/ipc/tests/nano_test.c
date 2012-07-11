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
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/ipc/MessageQ.h>

typedef unsigned int u32;
#ifdef OMAPL138
#include <ti/resources/rsc_table_omapl138.h>
#else
#include <ti/resources/rsc_table.h>
#endif
#include <xdc/std.h>

//#define VERBOSE 1

#define NUM_SLAVE_MSGS_PER_HOST_MSG   4

#define SLAVE_MESSAGEQNAME "SLAVE"

#define INPUT_MSG_DATASIZE  (8192)
#define OUTPUT_MSG_DATASIZE (INPUT_MSG_DATASIZE / NUM_SLAVE_MSGS_PER_HOST_MSG)

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
    System_printf("main: MultiProc id = %d (%s)\n", MultiProc_self(), __TIME__);
    System_printf("%d resources at 0x%x\n", resources.num, resources);

    Task_create(tsk1Fxn, NULL, NULL);

    BIOS_start();

    return (0);
}
