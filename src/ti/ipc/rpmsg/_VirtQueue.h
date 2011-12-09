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
 *  @file       _VirtQueue.h
 *
 *  @brief      Virtio Queue private file.
 *
 */


#ifndef ti_ipc__VirtQueue__include
#define ti_ipc__VirtQueue__include

#if defined (__cplusplus)
extern "C" {
#endif

/* Predefined device addresses */
#define IPU_MEM_VRING0          0xA0000000
#define IPU_MEM_VRING1          0xA0004000
#define IPU_MEM_VRING2          0xA0008000
#define IPU_MEM_VRING3          0xA000c000

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of two)
 */
#define VQ0_SIZE                256
#define VQ1_SIZE                256
#define VQ2_SIZE                256
#define VQ3_SIZE                256

/* See VirtQueue.c also for other constants:   */
#define RP_MSG_NUM_BUFS     (VQ0_SIZE) /* must be power of two */
#define RP_MSG_BUF_SIZE     (512)

typedef struct VirtQueue_Object {
    /* Id for this VirtQueue_Object */
    UInt16                  id;

    /* The function to call when buffers are consumed (can be NULL) */
    VirtQueue_callback      callback;

    /* Client object.  */
    UArg                    arg;

    /* Shared state */
    struct vring            vring;

    /* Number of free buffers */
    UInt16                  num_free;

    /* Last available index; updated by VirtQueue_getAvailBuf */
    UInt16                  last_avail_idx;

    /* Last available index; updated by VirtQueue_addUsedBuf */
    UInt16                  last_used_idx;

    /* Will eventually be used to kick remote processor */
    UInt16                  procId;

} VirtQueue_Object;


/* By convention, Host VirtQueues host are the even number in the pair */
#define VirtQueue_isSlave(vq)   (vq->id & 0x1)
#define VirtQueue_isHost(vq)    (~(vq->id & 0x1))


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* ti_ipc__VirtQueue__include */
