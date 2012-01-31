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
 *  ======== NameServerRemoteRpmsg.c ========
 */

#include <xdc/std.h>
#include <string.h>

#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/knl/ISync.h>

#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sdo/utils/_NameServer.h>
#include <ti/sdo/utils/INameServerRemote.h>

#include "package/internal/NameServerRemoteRpmsg.xdc.h"

#define MAXNAMEINCHAR	80
#define NAMEARRAYSZIE   (((MAXNAMEINCHAR - 1) / sizeof(Bits32)) + 1)

/* message sent to remote procId */
typedef struct NameServerMsg {
    Bits32  value;              /* holds value                      */
    Bits32  request;            /* whether its a request/response   */
    Bits32  requestStatus;      /* status of request                */
    Bits32  reserved;           /* reserved field                   */
                                /* name of NameServer instance      */
    Bits32  instanceName[NAMEARRAYSZIE];
                                /* name of NameServer entry         */
    Bits32  name[NAMEARRAYSZIE];
} NameServerMsg;


/*
 *************************************************************************
 *                       Instance functions
 *************************************************************************
 */
#define FXNN "NameServerRemoteRpmsg_Instance_init"
Void NameServerRemoteRpmsg_Instance_init(NameServerRemoteRpmsg_Object *obj,
        UInt16 remoteProcId,
        const NameServerRemoteRpmsg_Params *params)
{

    Log_print1(Diags_INFO, FXNN": remoteProc: %d", remoteProcId);

    obj->remoteProcId = remoteProcId;

    /* register the remote driver with NameServer */
    ti_sdo_utils_NameServer_registerRemoteDriver(
	NameServerRemoteRpmsg_Handle_upCast(obj), remoteProcId);

}
#undef FXNN

/*
 *  ======== NameServerRemoteRpmsg_Instance_finalize ========
 */
Void NameServerRemoteRpmsg_Instance_finalize(NameServerRemoteRpmsg_Object *obj)
{
    /* unregister remote driver from NameServer module */
    ti_sdo_utils_NameServer_unregisterRemoteDriver(obj->remoteProcId);
}

/*
 *************************************************************************
 *                       Module functions
 *************************************************************************
 */

 /*
 *  ======== NameServerRemoteRpmsg_attach ========
 *
 *  Typically called from Ipc_attach(), but since Rpmsg doesn't use Ipc
 *  module, this may need to be called manually.
 */
#define FXNN "NameServerRemoteRpmsg_attach"
Int NameServerRemoteRpmsg_attach(UInt16 remoteProcId, Ptr sharedAddr)
{
    Int status = NameServerRemoteRpmsg_E_FAIL;
    NameServerRemoteRpmsg_Handle handle;

    Log_print1(Diags_INFO, FXNN": remoteProcId: %d", remoteProcId);

    handle = NameServerRemoteRpmsg_create(remoteProcId, NULL, NULL);
    if (handle != NULL) {
        status = NameServerRemoteRpmsg_S_SUCCESS;
    }

    return (status);
}
#undef FXNN

/*
 *  ======== NameServerRemoteRpmsg_detach ========
 *
 *  Typically called from Ipc_detach(), but since Rpmsg doesn't use Ipc
 *  module, this may need to be called manually.
 */
#define FXNN "NameServerRemoteRpmsg_detach"
Int NameServerRemoteRpmsg_detach(UInt16 remoteProcId)
{
    Log_print1(Diags_INFO, FXNN": remoteProcId: %d", remoteProcId);

    NameServerRemoteRpmsg_Handle handle;
    Int status = NameServerRemoteRpmsg_S_SUCCESS;

    for (handle = NameServerRemoteRpmsg_Object_first(); handle != NULL;
        handle = NameServerRemoteRpmsg_Object_next(handle)) {
        if (handle->remoteProcId == remoteProcId) {
            break;
        }
    }

    if (handle == NULL) {
        status = NameServerRemoteRpmsg_E_FAIL;
    }
    else {
        NameServerRemoteRpmsg_delete(&handle);
    }

    return (status);
}
#undef FXNN

/*
 *  ======== NameServerRemoteRpmsg_get ========
 */
#define FXNN "NameServerRemoteRpmsg_get"
Int NameServerRemoteRpmsg_get(NameServerRemoteRpmsg_Object *obj,
                        String instanceName,
                        String name,
                        Ptr value,
                        UInt32 *valueLen,
                        ISync_Handle syncHandle,
                        Error_Block *eb)
{
    Int	status = NameServer_E_NOTFOUND;
    Int len;
    IArg key;
    NameServerMsg    msg;
    Semaphore_Handle semRemoteWait =
	NameServerRemoteRpmsg_module->semRemoteWait;
    GateMutex_Handle gateMutex = NameServerRemoteRpmsg_module->gateMutex;

    Log_print1(Diags_INFO, FXNN": name: %s", (IArg)name);

    /* enter gate - prevent multiple threads from entering */
    key = GateMutex_enter(gateMutex);

    /* Create request message and send to remote processor: */
    msg.request = NameServerRemoteRpmsg_REQUEST;
    msg.requestStatus = 0;

    len = strlen(instanceName);
    Assert_isTrue(len < MAXNAMEINCHAR, NameServerRemoteRpmsg_A_nameIsTooLong);
    strncpy((Char *)msg.instanceName, instanceName, len + 1);

    len = strlen(name);
    Assert_isTrue(len < MAXNAMEINCHAR, NameServerRemoteRpmsg_A_nameIsTooLong);
    strncpy((Char *)msg.name, name, len + 1);

#if 0
    TransportVirtio_sendRpmsg(msg, sizeof(msg), dst, src);

    /* Now pend for response */

#else  //TEMP:
    /* We expect the value from Linux side to be (procID = 0, index = 0): */
    if (!strcmp(name, "HOST")) {
	*(UInt32 *)value = 0x0;
	*valueLen = sizeof(UInt32);
	status = NameServer_S_SUCCESS;
    }
#endif

    /* leave the gate */
    GateMutex_leave(gateMutex, key);

    return (status);
}
#undef FXNN

/*
 *  ======== NameServerRemoteRpmsg_sharedMemReq ========
 */
SizeT NameServerRemoteRpmsg_sharedMemReq(Ptr sharedAddr)
{
    return (0);
}
