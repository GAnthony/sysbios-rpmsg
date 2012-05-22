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
/** ============================================================================
 *  @file       VirtQueue.c
 *
 *  @brief      Virtio Queue implementation for BIOS
 *
 *  Differences between BIOS version and Linux kernel (include/linux/virtio.h):
 *  - Renamed module from virtio.h to VirtQueue_Object.h to match the API prefixes;
 *  - BIOS (XDC) types and CamelCasing used;
 *  - virtio_device concept removed (i.e, assumes no containing device);
 *  - simplified scatterlist from Linux version;
 *  - The notify function is implicit in the implementation, and not provided
 *    by the client, as it is in Linux virtio.
 *
 *  All VirtQueue operations can be called in any context.
 *
 *  The virtio header should be included in an application as follows:
 *  @code
 *  #include <ti/ipc/rpmsg/VirtQueue.h>
 *  @endcode
 *
 */

#include <string.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Startup.h>
 
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Cache.h>

#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>
#include <ti/ipc/family/omap4430/InterruptM3.h>
#include <ti/pm/IpcPower.h>

#include <ti/ipc/MultiProc.h>

#include "package/internal/VirtQueue.xdc.h"

#include <ti/ipc/family/omap4430/virtio_ring.h>

/*
 * Define APPM3_IS_HOST to enable VirtioTransport test where appM3 is host and
 * sysM3 is slave.  Otherwise, VirtQueue defaults to Linux/A9 as host, and
 * appM3 and sysM3 as slaves (where sysM3 routes interrupts to appM3).
 */
//#define APPM3_IS_HOST

#define DIV_ROUND_UP(n,d)   (((n) + (d) - 1) / (d))
#define RP_MSG_BUFS_SPACE   (VirtQueue_RP_MSG_NUM_BUFS * VirtQueue_RP_MSG_BUF_SIZE * 2)

/* With 256 buffers, our vring will occupy 3 pages */
#define RP_MSG_RING_SIZE    ((DIV_ROUND_UP(vring_size(\
    VirtQueue_RP_MSG_NUM_BUFS, VirtQueue_RP_MSG_VRING_ALIGN),\
    VirtQueue_PAGE_SIZE)) * VirtQueue_PAGE_SIZE)

/* The total IPC space needed to communicate with a remote processor */
#define RPMSG_IPC_MEM   (RP_MSG_BUFS_SPACE + 2 * RP_MSG_RING_SIZE)

#define ID_SYSM3_TO_A9      0
#define ID_A9_TO_SYSM3      1
#define ID_APPM3_TO_A9      2
#define ID_A9_TO_APPM3      3

/* Used for defining the size of the virtqueue registry */
#define NUM_QUEUES 5

/*
 * enum - Predefined Mailbox Messages
 *
 * @RP_MSG_READY: informs the M3's that we're up and running. will be
 * followed by another mailbox message that carries the A9's virtual address
 * of the shared buffer. This would allow the A9's drivers to send virtual
 * addresses of the buffers.
 *
 * @RP_MSG_STATE_CHANGE: informs the receiver that there is an inbound
 * message waiting in its own receive-side vring. please note that currently
 * this message is optional: alternatively, one can explicitly send the index
 * of the triggered virtqueue itself. the preferred approach will be decided
 * as we progress and experiment with those design ideas.
 *
 * @RP_MSG_CRASH: this message indicates that the BIOS side is unhappy
 *
 * @RP_ECHO_REQUEST: this message requests the remote processor to reply
 * with RP_ECHO_REPLY
 *
 * @RP_ECHO_REPLY: this is a reply that is sent when RP_ECHO_REQUEST
 * is received.
 *
 * @RP_ABORT_REQUEST:  tells the M3 to crash on demand
 */
enum {
    RP_MSG_READY           = (Int)0xFFFFFF00,
    RP_MSG_STATE_CHANGE    = (Int)0xFFFFFF01,
    RP_MSG_CRASH           = (Int)0xFFFFFF02,
    RP_MSG_ECHO_REQUEST    = (Int)0xFFFFFF03,
    RP_MSG_ECHO_REPLY      = (Int)0xFFFFFF04,
    RP_MSG_ABORT_REQUEST   = (Int)0xFFFFFF05,
    RP_MSG_FLUSH_CACHE     = (Int)0xFFFFFF06,
    RP_MSG_HIBERNATION     = (Int)0xFFFFFF07
};

static VirtQueue_Object *queueRegistry[NUM_QUEUES] = {NULL};

static Bool hostReadyToRecv = FALSE;

static Bool checkPrimedBuffers(VirtQueue_Object * vq);

static inline Void * mapPAtoVA(UInt pa)
{
    return (Void *)((pa & 0x000fffffU) | 0xa0000000U);
}

static inline UInt mapVAtoPA(Void * va)
{
    return ((UInt)va & 0x000fffffU) | 0xa9000000U;
}

/*!
 * ======== VirtQueue_startup ========
 */
Void VirtQueue_startup(UInt16 remoteProcId, Bool isHost)
{
    Fxn	isr_fxn;
    IInterrupt_IntInfo intInfo;

    /* Initilize the IpcPower module */
    IpcPower_init();

    if (isHost)  {
       /* Host is responsible for zeroing out vring memory: */
       memset((void *)VirtQueue_IPU_MEM_VRING0, 0,
                RP_MSG_RING_SIZE * 2 + RP_MSG_BUFS_SPACE);

       isr_fxn = (Fxn) VirtQueue_hostIsr;
    }
    else {
       isr_fxn = (Fxn) VirtQueue_slaveIsr;
    }

    /* Plug ISR.*/
    InterruptM3_intRegister(remoteProcId, &intInfo, isr_fxn, NULL);

    if (isHost)  {
       /* Host sends init sequence to sync slave processor (sysM3): */
       InterruptM3_intSend(remoteProcId, NULL, (UInt)RP_MSG_READY);
       InterruptM3_intSend(remoteProcId, NULL, (UInt)RP_MSG_ECHO_REQUEST);

       /* Wait until host and slaves have synced: */
       VirtQueue_module->hostSlaveSynced = 0;
       while (!VirtQueue_module->hostSlaveSynced);
    }
    else {
       /* Wait until host has indicated it's ready to receive: */
       while (!hostReadyToRecv);
    }

    Log_print0(Diags_USER1, "Passed VirtQueue_startup\n");
}

/*!
 * ======== VirtQueue_Instance_init ========
 */
Void VirtQueue_Instance_init(VirtQueue_Object *vq, UInt16 remoteProcId,
                             const VirtQueue_Params *params)
{
    void *vring_phys;
    Error_Block eb;

    Error_init(&eb);

    vq->vringPtr = Memory_calloc(NULL, sizeof(struct vring), 0, &eb);
    Assert_isTrue((vq->vringPtr != NULL), NULL);

    vq->callback = params->callback;
    vq->id = VirtQueue_module->numQueues++;
    vq->procId = remoteProcId;
    vq->last_avail_idx = 0;
    vq->last_used_idx = 0;
    vq->num_free = VirtQueue_RP_MSG_NUM_BUFS;
    vq->swiHandle = params->swiHandle;

    switch (vq->id) {
        case ID_SYSM3_TO_A9:
            vring_phys = (struct vring *) VirtQueue_IPU_MEM_VRING0;
            break;
        case ID_A9_TO_SYSM3:
            vring_phys = (struct vring *) VirtQueue_IPU_MEM_VRING1;
            break;
        case ID_APPM3_TO_A9:
            vring_phys = (struct vring *) VirtQueue_IPU_MEM_VRING2;
            break;
        case ID_A9_TO_APPM3:
            vring_phys = (struct vring *) VirtQueue_IPU_MEM_VRING3;
            break;
    }

    Log_print3(Diags_USER1,
            "vring: %d 0x%x (0x%x)", vq->id, (IArg)vring_phys,
            RP_MSG_RING_SIZE);

    vring_init(vq->vringPtr, VirtQueue_RP_MSG_NUM_BUFS, vring_phys, VirtQueue_RP_MSG_VRING_ALIGN);

    queueRegistry[vq->id] = vq;

    return;
}
/*
 *  ======== VirtQueue_Instance_finalize ========
 */
Void VirtQueue_Instance_finalize(VirtQueue_Object *vq, Int status)
{
    queueRegistry[vq->id] = (VirtQueue_Object *)NULL;
    Memory_free(NULL, vq->vringPtr, sizeof(struct vring));
}

/*!
 * ======== VirtQueue_kick ========
 */
Void VirtQueue_kick(VirtQueue_Handle vq)
{
    struct vring *vring = vq->vringPtr;

    /* For now, simply interrupt remote processor */
    if (vring->avail->flags & VRING_AVAIL_F_NO_INTERRUPT) {
        Log_print0(Diags_USER1,
                "VirtQueue_kick: no kick because of VRING_AVAIL_F_NO_INTERRUPT\n");
        return;
    }

    Log_print2(Diags_USER1,
            "VirtQueue_kick: Sending interrupt to proc %d with payload 0x%x\n",
            (IArg)vq->procId, (IArg)vq->id);
    InterruptM3_intSend(vq->procId, NULL, vq->id);
}

/* By convention, Host VirtQueues host are the even number in the pair */
Bool VirtQueue_isSlave(VirtQueue_Handle vq)
{
  return (vq->id & 0x1);
}

Bool VirtQueue_isHost(VirtQueue_Handle vq)
{
  return (~(vq->id & 0x1));
}

UInt16 VirtQueue_getId (VirtQueue_Handle vq)
{
  return (vq->id);
}

Swi_Handle VirtQueue_getSwiHandle (VirtQueue_Handle vq)
{
  return (vq->swiHandle);
}

/*!
 * ======== VirtQueue_addAvailBuf ========
 */
Int VirtQueue_addAvailBuf(VirtQueue_Object *vq, Void *buf)
{
    UInt16 avail;
    struct vring *vring = vq->vringPtr;

    Log_print3(Diags_USER1,
       "addAvailBuf vq->id: %d vq->num_free: %d avail->idx: %d",
        vq->id, vq->num_free, vring->avail->idx);

    if (vq->num_free == 0) {
        /* There's no more space */
        Error_raise(NULL, Error_E_generic, 0, 0);
    }
    vq->num_free--;

    avail =  vring->avail->idx++ % vring->num;

    vring->desc[avail].addr = mapVAtoPA(buf);
    vring->desc[avail].len = VirtQueue_RP_MSG_BUF_SIZE;

    return (vq->num_free);
}

/*!
 * ======== VirtQueue_getUsedBuf ========
 */
Void *VirtQueue_getUsedBuf(VirtQueue_Object *vq)
{
    UInt16 head;
    Void *buf;
    struct vring *vring = vq->vringPtr;

    Log_print3(Diags_USER1,
       "getUsedBuf vq->id: %d last_used_idx: %d used->idx: %d",
        vq->id, vq->last_used_idx, vring->used->idx);

    /* There's nothing available? */
    if (vq->last_used_idx == vring->used->idx) {
        return (NULL);
    }

    head = vring->used->ring[vq->last_used_idx % vring->num].id;
    vq->last_used_idx++;
    vq->num_free++;

    buf = mapPAtoVA(vring->desc[head].addr);

    return (buf);
}

/*!
 * ======== VirtQueue_getAvailBuf ========
 */
Int16 VirtQueue_getAvailBuf(VirtQueue_Handle vq, Void **buf)
{
    UInt16 head;
    struct vring *vring = vq->vringPtr;

    Log_print5(Diags_USER1,
       "getAvailBuf vq->id: %d last_avail_idx: %d avail->idx: %d num: %d 0x%x",
        vq->id, vq->last_avail_idx, vring->avail->idx, vring->num,
        (IArg)&vring->avail);

    /* There's nothing available? */
    if (vq->last_avail_idx == vring->avail->idx) {
        if (VirtQueue_isHost(vq))  {
            /* We need to know about added buffers */
            vring->used->flags &= ~VRING_USED_F_NO_NOTIFY;
            /* check again after setting flag */
            if (vq->last_avail_idx == vring->avail->idx)  {
                return (-1);
            }
       }
       else {
            return (-1);
       }
    }

    /* No need to be kicked about added buffers anymore */
    if (VirtQueue_isHost(vq))  {
        vring->used->flags |= VRING_USED_F_NO_NOTIFY;
    }

    /*
     * Grab the next descriptor number they're advertising, and increment
     * the index we've seen.
     */
#ifndef APPM3_IS_HOST  /* This line works with Linux rpmsg Host: */
    head = vring->avail->ring[vq->last_avail_idx++ % vring->num];
#else  /* This line works with both Linux and appM3 as rpmsg Hosts: */
    head = vq->last_avail_idx++ % vring->num;
#endif

    *buf = mapPAtoVA(vring->desc[head].addr);

    return (head);
}

/*!
 * ======== VirtQueue_addUsedBuf ========
 */
Int VirtQueue_addUsedBuf(VirtQueue_Handle vq, Int16 head)
{
    struct vring_used_elem *used;
    struct vring *vring = vq->vringPtr;

    Log_print3(Diags_USER1,
       "addUsedBuf vq->id: %d head: %d used->idx: %d",
        vq->id, head, vring->used->idx);

    if ((head > vring->num) || (head < 0)) {
        Error_raise(NULL, Error_E_generic, 0, 0);
    }

    /*
    * The virtqueue contains a ring of used buffers.  Get a pointer to the
    * next entry in that used ring.
    */
    used = &vring->used->ring[vring->used->idx % vring->num];
    used->id = head;
    used->len = VirtQueue_RP_MSG_BUF_SIZE;

    vring->used->idx++;

    return (0);
}

/*!
 * ======== VirtQueue_disableCallback ========
 */
Void VirtQueue_disableCallback(VirtQueue_Object *vq)
{
    //TODO
    Log_print0(Diags_USER1, "VirtQueue_disableCallback called.");
}

/*!
 * ======== VirtQueue_enableCallback ========
 */
Bool VirtQueue_enableCallback(VirtQueue_Object *vq)
{
    Log_print0(Diags_USER1, "VirtQueue_enableCallback called.");

    //TODO
    return (FALSE);
}

/*!
 * ======== VirtQueue_hostIsr ========
 * Note 'arg' is ignored: it is the Hwi argument, not the mailbox argument.
 */
Void VirtQueue_hostIsr(UArg msg)
{
    VirtQueue_Object *vq;

    Log_print1(Diags_USER1, "VirtQueue_hostIsr received msg = 0x%x\n", msg);

    if (MultiProc_self() == VirtQueue_sysm3ProcId) {
        switch(msg) {
            case (UInt)RP_MSG_READY:
                return;

            case (UInt)RP_MSG_ECHO_REQUEST:
                InterruptM3_intSend(VirtQueue_appm3ProcId, NULL, 
                                    (UInt)(RP_MSG_ECHO_REPLY));
                VirtQueue_module->hostSlaveSynced = 1;
                return;

            case (UInt)RP_MSG_ABORT_REQUEST:
                {
                    Fxn f = (Fxn)0x0;
                    Log_print0(Diags_USER1, "Crash on demand ...\n");
                    f();
                }
                return;

            case (UInt)RP_MSG_FLUSH_CACHE:
                Cache_wbAll();
                return;

            case (UInt)RP_MSG_HIBERNATION:
                /* Notify Core1 */
                InterruptM3_intSend(VirtQueue_appm3ProcId, NULL,
			 (UInt)(RP_MSG_HIBERNATION));
                IpcPower_suspend();
                return;

            default:
                /*
                 *  If the message isn't one of the above, it's either part of 
		 * the 2-message synchronization sequence or it a virtqueue 
                 * message.
                 */
                break;
        }
    }
    else if (msg & 0xFFFF0000) {
        if (msg == (UInt)RP_MSG_HIBERNATION) {
            IpcPower_suspend();
        }
        if ((MultiProc_self() == VirtQueue_appm3ProcId) &&
             (msg == (UInt)(RP_MSG_ECHO_REPLY)))  {
           VirtQueue_module->hostSlaveSynced = 1;
        }
        return;
    }

    vq = queueRegistry[msg];
    if (vq) {
        vq->callback(vq);
    }
}

/*!
 * ======== VirtQueue_slaveIsr ========
 * Note 'arg' is ignored: it is the Hwi argument, not the mailbox argument.
 */
Void VirtQueue_slaveIsr(UArg msg)
{
    VirtQueue_Object *vq;

    Log_print1(Diags_USER1, "VirtQueue_slaveIsr received msg = 0x%x\n", msg);

    if (MultiProc_self() == VirtQueue_sysm3ProcId) {
        switch(msg) {
            case (UInt)RP_MSG_READY:
                return;

            case (UInt)RP_MSG_ECHO_REQUEST:
                InterruptM3_intSend(VirtQueue_hostProcId, NULL, 
                                    (UInt)(RP_MSG_ECHO_REPLY));
                return;

            case (UInt)RP_MSG_ABORT_REQUEST:
                {
                    Fxn f = (Fxn)0x0;
                    Log_print0(Diags_USER1, "Crash on demand ...\n");
                    f();
                }
                return;

            case (UInt)RP_MSG_FLUSH_CACHE:
                Cache_wbAll();
                return;

            case (UInt)RP_MSG_HIBERNATION:
                /* Notify Core1 */
                InterruptM3_intSend(VirtQueue_appm3ProcId, NULL, 
                                    (UInt)(RP_MSG_HIBERNATION));
                IpcPower_suspend();
                return;

            default:
                /*
                 *  If the message isn't one of the above, it's either part of the
                 *  2-message synchronization sequence or it a virtqueue message
                 */
                break;
        }
    }
    else if (msg & 0xFFFF0000) {
        if (msg == (UInt)RP_MSG_HIBERNATION) {
            IpcPower_suspend();
        }
        return;
    }

    if (MultiProc_self() == VirtQueue_sysm3ProcId &&
            (msg == ID_A9_TO_APPM3 || msg == ID_APPM3_TO_A9)) {
        InterruptM3_intSend(VirtQueue_appm3ProcId, NULL, (UInt)msg);
    }
    else {
        vq = queueRegistry[msg];
        if (vq) {
            vq->callback(vq);

            /* Check to see if host just sent it's first interrupt: */
            if (!hostReadyToRecv) {
               hostReadyToRecv = checkPrimedBuffers(vq);
            }
        }
        else {
            Log_print0(Diags_USER1, "msg recvd before callback registered!\n");
        }
    }
}


/*!
 * ======== postCrashToMailbox ========
 */
Void postCrashToMailbox(Error_Block * eb)
{
    Error_print(eb);
    InterruptM3_intSend(VirtQueue_hostProcId, NULL, (UInt)RP_MSG_CRASH);
}


#define CACHE_WB_TICK_PERIOD    5

/*!
 * ======== VirtQueue_cacheWb ========
 *
 * Used for flushing SysMin trace buffer.
 */
Void VirtQueue_cacheWb()
{
    static UInt32 oldticks;

    if (Clock_getTicks() >= (oldticks + CACHE_WB_TICK_PERIOD)) {
        /* Don't keep flushing cache */
        return;
    }

    /* Flush the cache */
    Cache_wbAll();
}

static Bool checkPrimedBuffers(VirtQueue_Object * vq)
{
    struct vring *vring = vq->vringPtr;

    Log_print1(Diags_USER1, "avail->idx: %d\n", vring->avail->idx);
    return (vring->avail->idx == VirtQueue_RP_MSG_NUM_BUFS);
}
