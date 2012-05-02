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
 *  ======== TransportVirtio.c ========
 *
 */

#include <string.h>

#include <xdc/std.h>

#include <xdc/runtime/System.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Main.h>
#include <xdc/runtime/Registry.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>


#include <ti/sdo/utils/_MultiProc.h>
#include <ti/sdo/ipc/_MessageQ.h>

#include <ti/ipc/rpmsg/virtio_ring.h>
#include <ti/ipc/rpmsg/rpmsg.h>
#include <ti/ipc/rpmsg/MessageQCopy.h>
#include <ti/ipc/rpmsg/VirtQueue.h>
#include <ti/ipc/rpmsg/_VirtQueue.h>

#include <ti/ipc/namesrv/_NameServerRemoteRpmsg.h>

#include "_TransportVirtio.h"
#include "package/internal/TransportVirtio.xdc.h"

/* Maximum RPMSG payload: */
#define MAX_PAYLOAD (RP_MSG_BUF_SIZE - sizeof(RpMsg_Header))

/* That special per processor RPMSG channel reserved to multiplex MessageQ */
#define RPMSG_MESSAGEQ_PORT         61

/* Addresses below this are assumed to be bound to MessageQ objects: */
#define RPMSG_RESERVED_ADDRESSES     (1024)

/* Name of the rpmsg socket on host: */
#define RPMSG_SOCKET_NAME  "rpmsg-proto"

static Void transportCallbackFxn(MessageQCopy_Handle msgq, UArg arg, Ptr data,
                                      UInt16 dataLen, UInt32 srcAddr);

/*  --------------  TEMP NameService over rpmsg ----------------------- */

static void nameService_register(UInt16 dstProc, char * name, UInt32 port, enum rpmsg_ns_flags flags)
{
    struct rpmsg_ns_msg nsMsg;
    UInt16 len      = sizeof(nsMsg);
    UInt32 dstEndpt = NAMESERVICE_PORT;
    UInt32 srcEndpt = port;
    Ptr    data     = &nsMsg;

    strncpy(nsMsg.name, name, RPMSG_NAME_SIZE);

    nsMsg.addr = port;
    nsMsg.flags = flags;

    MessageQCopy_send(dstProc, dstEndpt, srcEndpt, data, len);
}


/*
 *************************************************************************
 *                       Instance functions
 *************************************************************************
 */

/*
 *  ======== TransportVirtio_Instance_init ========
 *
 */
#define FXNN "TransportVirtio_Instance_init"
Int TransportVirtio_Instance_init(TransportVirtio_Object *obj,
        UInt16 remoteProcId, const TransportVirtio_Params *params,
        Error_Block *eb)
{
    Bool        flag;
    UInt32      myEndpoint = 0;

    /* set object fields */
    obj->priority     = params->priority;
    obj->remoteProcId = remoteProcId;

    Log_print1(Diags_INFO, FXNN": remoteProc: %d\n", obj->remoteProcId);

    /* Announce our "MessageQ" service to other side: */
    nameService_register(remoteProcId, RPMSG_SOCKET_NAME, RPMSG_MESSAGEQ_PORT,
                         RPMSG_NS_CREATE);

    /* Associate incomming messages with this transport's callback fxn: */
    obj->msgqHandle = MessageQCopy_createEx(RPMSG_MESSAGEQ_PORT,
                                          transportCallbackFxn,
                                          (UArg)obj,
                                          &myEndpoint);

    /* TBD: The following should be done later via a ns_announcement from
     * Linux side to set this.
     * Setting this now will cause NameServer requests from BIOS side to
     * timeout (benignly), as the app calls MessageQ_open() in a loop.
     * The NameMap module needs to register for ns_announcements.
     */
    NameServerRemote_SetNameServerPort(NAME_SERVER_RPMSG_ADDR);

    if (obj->msgqHandle) {
        /* Register the transport with MessageQ */
        flag = ti_sdo_ipc_MessageQ_registerTransport(
            TransportVirtio_Handle_upCast(obj), remoteProcId, params->priority);
    }

    if (flag == FALSE) {
        return (2);
    }

    return (0);
}
#undef FXNN

/*
 *  ======== TransportVirtio_Instance_finalize ========
 */
#define FXNN "TransportVirtio_Instance_finalize"
Void TransportVirtio_Instance_finalize(TransportVirtio_Object *obj, Int status)
{
    Log_print0(Diags_ENTRY, "--> "FXNN);

    /* Announce our "MessageQ" service is going away: */
    nameService_register(obj->remoteProcId, RPMSG_SOCKET_NAME,
                         RPMSG_MESSAGEQ_PORT, RPMSG_NS_DESTROY);


    switch(status) {
        case 0: /* MessageQ_registerTransport succeeded */
            ti_sdo_ipc_MessageQ_unregisterTransport(obj->remoteProcId,
                obj->priority);

            /* fall thru OK */
        case 1: /* NOT USED: Notify_registerEventSingle failed */
        case 2: /* MessageQ_registerTransport failed */
            break;
    }
#undef FXNN
}

/*
 *  ======== TransportVirtio_put ========
 *
 *  Notes: In keeping with the semantics of IMessageQTransport_put(), we
 *  simply return FALSE if the remote proc has made no buffers available in the
 *  vring.
 *  Otherwise, we could block here, waiting for the remote proc to add a buffer.
 *  This implies that the remote proc must always have buffers available in the
 *  vring in order for this side to send without failing!
 *
 *  Also, this is a copy-transport, to match the Linux side rpmsg.
 */
#define FXNN "TransportVirtio_put"
Bool TransportVirtio_put(TransportVirtio_Object *obj, Ptr msg)
{
    Int          status;
    UInt         msgSize;
    UInt16       dstAddr;

    /* Send to remote processor: */
    msgSize = MessageQ_getMsgSize(msg);
    Assert_isTrue(msgSize <= MAX_PAYLOAD, NULL);
    dstAddr  = (((MessageQ_Msg)msg)->dstId & 0x0000FFFF);

    Log_print3(Diags_INFO, FXNN": sending msg from: %d, to: %d, dataLen: %d",
                  (IArg)RPMSG_MESSAGEQ_PORT, (IArg)dstAddr, (IArg)msgSize);
    status = MessageQCopy_send(obj->remoteProcId, dstAddr, RPMSG_MESSAGEQ_PORT,
                      msg, msgSize);

    /* free the app's message */
    if (((MessageQ_Msg)msg)->heapId != ti_sdo_ipc_MessageQ_STATICMSG) {
       MessageQ_free(msg);
    }

    return (status == MessageQCopy_S_SUCCESS? TRUE: FALSE);
}
#undef FXNN

/*
 *  ======== TransportVirtio_control ========
 */
Bool TransportVirtio_control(TransportVirtio_Object *obj, UInt cmd,
    UArg cmdArg)
{
    return (FALSE);
}

/*
 *  ======== TransportVirtio_getStatus ========
 */
Int TransportVirtio_getStatus(TransportVirtio_Object *obj)
{
    return (0);
}

/*
 *************************************************************************
 *                       Module functions
 *************************************************************************
 */

/*
 *  ======== transportCallbackFxn ========
 *
 */
#define FXNN "transportCallbackFxn"
static Void transportCallbackFxn(MessageQCopy_Handle msgq, UArg arg, Ptr data,
                                      UInt16 dataLen, UInt32 srcAddr)
{
    UInt32            queueId;
    MessageQ_Msg      msg;
    MessageQ_Msg      buf = NULL;
    UInt              msgSize;
    NameServerMsg     * ns_msg;  /* Name Server Message */

    Log_print0(Diags_ENTRY, "--> "FXNN);

    Log_print3(Diags_INFO, FXNN": Received data: 0x%x from: %d, dataLen: %d",
                  (IArg)data, (IArg)srcAddr, (IArg)dataLen);

    if(srcAddr >= RPMSG_RESERVED_ADDRESSES) {
        /* This could either be a NameServer request or a MessageQ.
         * Check the NameServerMsg reserved field to distinguish.
         */
        ns_msg = (NameServerMsg *)data;
        if (ns_msg->reserved == NAMESERVER_MSG_TOKEN) {
            /* Process the NameServer request/reply message: */
            NameServerRemote_processMessage(ns_msg);
            goto exit;
        }
    }

    /* Convert RpMsg payload into a MessageQ_Msg: */
    msg = (MessageQ_Msg)data;

    Log_print4(Diags_INFO, FXNN": \n\tmsg->heapId: %d, "
               "msg->msgSize: %d, msg->dstId: %d, msg->msgId: %d\n",
               msg->heapId, msg->msgSize, msg->dstId, msg->msgId);

    /* Alloc a message from msg->heapId to copy the msg */
    msgSize = MessageQ_getMsgSize(msg);
    buf = MessageQ_alloc(msg->heapId, msgSize);

    /* Make sure buf is not NULL */
    Assert_isTrue(buf != NULL, NULL);

    /* copy the message to the buffer allocated. */
    memcpy((Ptr)buf, (Ptr)msg, msgSize);

    /* get the queue id */
    queueId = MessageQ_getDstQueue(msg);

    /* Pass to desitination queue: */
    MessageQ_put(queueId, buf);

exit:
    Log_print0(Diags_EXIT, "<-- "FXNN);
}

/*
 *  ======== TransportVirtio_setErrFxn ========
 */
Void TransportVirtio_setErrFxn(TransportVirtio_ErrFxn errFxn)
{
    /* Ignore the errFxn */
}

