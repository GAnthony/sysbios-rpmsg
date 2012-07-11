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
 *  ======== messageq_socket.c ========
 *
 *  Test for messageq over sockets.
 *
 *  Requires:
 *      tools/messageq_socket in rpmsg_3.2_rc4 branch of upstream-rpmsg.
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

#define SLAVE_MESSAGEQNAME "SLAVE"

#define MessageQ_payload(m) ((void *)((char *)(m) + sizeof(MessageQ_MsgHeader)))

/*
 *  ======== tsk1Fxn ========
 *  Receive and return messages
 */
Void tsk1Fxn(UArg arg0, UArg arg1)
{
    MessageQ_Msg msg;
    MessageQ_Handle  messageQ;
    MessageQ_QueueId remoteQueueId;
    UInt16 procId;
    Int status;
    UInt16 msgId;
    UInt32 start;
    UInt32 end;
    Uint32 numLoops;
    UInt32 print;
    UInt32 *params;

    messageQ = MessageQ_create(SLAVE_MESSAGEQNAME, NULL);
    if (messageQ == NULL) {
        System_abort("MessageQ_create failed\n" );
    }

    System_printf("tsk1Fxn: created MessageQ: %s; QueueID: 0x%x\n",
        SLAVE_MESSAGEQNAME, MessageQ_getQueueId(messageQ));

    while (1) {
        /* handshake with host to get starting parameters */
        System_printf("Awaiting sync message from host...\n");
        MessageQ_get(messageQ, &msg, MessageQ_FOREVER);

        params = MessageQ_payload(msg);
        numLoops = params[0];
        print = params[1];

        remoteQueueId = MessageQ_getReplyQueue(msg);
        procId = MessageQ_getProcId(remoteQueueId);

        System_printf("Received msg from (procId:remoteQueueId): 0x%x:0x%x"
            "\tpayload: %d bytes; loops: %s printing \n",
            procId, remoteQueueId,
            (MessageQ_getMsgSize(msg) - sizeof(MessageQ_MsgHeader)),
            numLoops, print ? "with" : "without");

        MessageQ_put(remoteQueueId, msg);

        start = Clock_getTicks();
        for (msgId = 0; msgId < numLoops; msgId++) {
            status = MessageQ_get(messageQ, &msg, MessageQ_FOREVER);
            Assert_isTrue(status == MessageQ_S_SUCCESS, NULL);

            if (print) {
                System_printf("Got msg #%d (%d bytes) from core %d\n",
                    MessageQ_getMsgId(msg), msg->msgSize, procId);
            }

            Assert_isTrue(MessageQ_getMsgId(msg) == msgId, NULL);

            if (print) {
                System_printf("Sending msg Id #%d to core %d\n", msgId, procId);
            }

            status = MessageQ_put(remoteQueueId, msg);
            Assert_isTrue(status == MessageQ_S_SUCCESS, NULL);
        }
        end = Clock_getTicks();

        System_printf("%d iterations took %d ticks or %d usecs/msg\n", numLoops,
            end - start, ((end - start) * Clock_tickPeriod) / numLoops);
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
