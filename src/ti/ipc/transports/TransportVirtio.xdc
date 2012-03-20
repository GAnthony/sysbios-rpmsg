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
 *  ======== TransportVirtio.xdc ================
 */

import ti.sysbios.knl.Swi;
import ti.sysbios.gates.GateSwi;

/*!
 *  ======== TransportVirtio ========
 *  Transport for MessageQ that uses vring structures.
 *
 *  This is a {@link ti.sdo.ipc.MessageQ} transport that utilizes
 *  a pair of vrings (see Linux virtio) to communicate with a remote processor.
 *
 */

@InstanceFinalize
@InstanceInitError


module TransportVirtio inherits ti.sdo.ipc.interfaces.IMessageQTransport
{

instance:

    /*!
     *  ======== sharedAddr ========
     *  Address in shared memory where this instance will be placed
     *
     */
    config Ptr sharedAddr = null;

    /*!
     *  ======== intVectorId ========
     *  Interrupt vector ID to be used by the driver.
     *
     *  This parameter is only used by C64x+ targets
     */
    config UInt intVectorId = ~1u;

internal:

    /*!
     *  ======== swiFxn ========
     */
    Void swiFxn(UArg arg0, UArg arg1);

    struct Module_State 
    {
        GateSwi.Handle gateSwiHandle;
    }

    /*! Instance state structure */
    struct Instance_State {
        UInt16       priority;           /* priority to register             */
        UInt16       remoteProcId;       /* dst proc id                      */
        Bool         isHost;             /* self proc id acts as a host.     */
        Swi.Object   swiObj;             /* Each instance has a swi          */
        Ptr          vq_slave;           /* Slave's VirtQueue Handle         */
        Ptr          vq_host;            /* Host's VirtQueue Handle          */
        Ptr          sbufs;              /* Buffers for sending              */
        UInt16       last_sbuf;          /* Index of last send buffer used   */
        Int          name_server_port;   /* Rpmsg src address of Name Server */
    }
}
