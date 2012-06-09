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
 *  ======== rsc_table_omapl138.h ========
 *
 *  Include this table in each base image, which is read from remoteproc on
 *  host side.
 *
 *  These values are currently very OMAPL138 specific!
 *
 */

#ifndef _RSC_TABLE_OMAPL138_H_
#define _RSC_TABLE_OMAPL138_H_

#define DATA_DA                 0xc4000000

#ifndef DATA_SIZE
#  define DATA_SIZE  (SZ_1M * 16)
#endif

#define RPMSG_VRING0_DA         0xc3000000
#define RPMSG_VRING1_DA         0xc3004000

#define CONSOLE_VRING0_DA       0xc3008000
#define CONSOLE_VRING1_DA       0xc300C000

#define BUFS0_DA                0xc3040000
#define BUFS1_DA                0xc3080000

/*
 * sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define RPMSG_VQ0_SIZE                256
#define RPMSG_VQ1_SIZE                256

#define CONSOLE_VQ0_SIZE                256
#define CONSOLE_VQ1_SIZE                256

/* Size constants must match those used on host: include/asm-generic/sizes.h */
#define SZ_1M                           0x00100000
#define SZ_2M                           0x00200000
#define SZ_4M                           0x00400000
#define SZ_8M                           0x00800000
#define SZ_16M                          0x01000000
#define SZ_32M                          0x02000000
#define SZ_64M                          0x04000000
#define SZ_128M                         0x08000000
#define SZ_256M                         0x10000000
#define SZ_512M                         0x20000000

/* virtio ids: keep in sync with the linux "include/linux/virtio_ids.h" */
#define VIRTIO_ID_CONSOLE	3 /* virtio console */
#define VIRTIO_ID_RPMSG		7 /* virtio remote processor messaging */

/* Indices of rpmsg virtio features we support */
#define VIRTIO_RPMSG_F_NS	0 /* RP supports name service notifications */

/* flip up bits whose indices represent features we support */
#define RPMSG_IPU_C0_FEATURES         1

/* Resource info: Must match include/linux/remoteproc.h: */
#define TYPE_CARVEOUT    0
#define TYPE_DEVMEM      1
#define TYPE_TRACE       2
#define TYPE_VDEV  3

struct fw_rsc_carveout {
	u32 type;
	u32 da;
	u32 pa;
	u32 len;
	u32 flags;
	u32 reserved;
	char name[32];
};

struct fw_rsc_devmem {
	u32 type;
	u32 da;
	u32 pa;
	u32 len;
	u32 flags;
	u32 reserved;
	char name[32];
};

struct fw_rsc_trace {
	u32 type;
	u32 da;
	u32 len;
	u32 reserved;
	char name[32];
};

struct fw_rsc_vdev_vring {
	u32 da; /* device address */
	u32 align;
	u32 num;
	u32 notifyid;
	u32 reserved;
};

struct fw_rsc_vdev {
	u32 type;
	u32 id;
	u32 notifyid;
	u32 dfeatures;
	u32 gfeatures;
	u32 config_len;
	char status;
	char num_of_vrings;
	char reserved[2];
};

struct resource_table {
	u32 version;
	u32 num;
	u32 reserved[2];
	u32 offset[13];

	/* rpmsg vdev entry */
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring rpmsg_vring0;
	struct fw_rsc_vdev_vring rpmsg_vring1;

	/* data carveout entry */
	struct fw_rsc_carveout data_cout;

	/* trace entry */
	struct fw_rsc_trace trace;
};

extern char * xdc_runtime_SysMin_Module_State_0_outbuf__A;
#define TRACEBUFADDR (u32)&xdc_runtime_SysMin_Module_State_0_outbuf__A

#pragma DATA_SECTION(resources, ".resource_table")
#pragma DATA_ALIGN(resources, 4096)

struct resource_table resources = {
	1, /* we're the first version that implements this */
	3, /* number of entries in the table */
	0, 0, /* reserved, must be zero */
	/* offsets to entries */
	{
		offsetof(struct resource_table, rpmsg_vdev),
		offsetof(struct resource_table, data_cout),
		offsetof(struct resource_table, trace),
	},

	/* rpmsg vdev entry */
	{
		TYPE_VDEV, VIRTIO_ID_RPMSG, 0,
		RPMSG_IPU_C0_FEATURES, 0, 0, 0, 2, { 0, 0 },
		/* no config data */
	},
	/* the two vrings */
	{ RPMSG_VRING0_DA, 4096, RPMSG_VQ0_SIZE, 1, 0 },
	{ RPMSG_VRING1_DA, 4096, RPMSG_VQ1_SIZE, 2, 0 },

	{
		TYPE_CARVEOUT, DATA_DA, DATA_DA, DATA_SIZE, 0, 0, "IPU_MEM_DATA",
	},

	{
		TYPE_TRACE, TRACEBUFADDR, 0x8000, 0, "trace:dsp",
	},

};

#endif /* _RSC_TABLE_OMAPL138_H_ */
