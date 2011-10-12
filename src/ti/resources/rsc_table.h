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
 *  ======== rsc_table.h ========
 *
 *  Include this table in each base image, which is read from remoteproc on
 *  host side.
 *
 *  These values are currently very OMAP4 specific!
 *
 */


#ifndef _RSC_TABLE_H_
#define _RSC_TABLE_H_



/* Ducati Memory Map: */
#define L4_44XX_BASE            0x4a000000

#define L4_PERIPHERAL_L4CFG     (L4_44XX_BASE)
#define IPU_PERIPHERAL_L4CFG    0xAA000000

#define L4_PERIPHERAL_L4PER     0x48000000
#define IPU_PERIPHERAL_L4PER    0xA8000000

#define L4_PERIPHERAL_L4EMU     0x54000000
#define IPU_PERIPHERAL_L4EMU    0xB4000000

#define L3_IVAHD_CONFIG         0x5A000000
#define IPU_IVAHD_CONFIG        0xBA000000

#define L3_IVAHD_SL2            0x5B000000
#define IPU_IVAHD_SL2           0xBB000000

#define L3_TILER_MODE_0_1       0x60000000
#define IPU_TILER_MODE_0_1      0x60000000

#define L3_TILER_MODE_2         0x70000000
#define IPU_TILER_MODE_2        0x70000000

#define L3_TILER_MODE_3         0x78000000
#define IPU_TILER_MODE_3        0x78000000

#define L3_IVAHD_CONFIG         0x5A000000
#define IPU_IVAHD_CONFIG        0xBA000000

#define L3_IVAHD_SL2            0x5B000000
#define IPU_IVAHD_SL2           0xBB000000

#define TEXT_DA                 0x00000000
#define DATA_DA                 0x80000000

#define IPC_DA                  0xA0000000
#define IPC_PA                  0xA9800000

#define VRING0_DA               0xA0000000
#define VRING1_DA               0xA0003000
#define VRING2_DA               0xA0008000
#define VRING3_DA               0xA000b000
#define BUFS0_DA                0xA0040000
#define BUFS1_DA                0xA0080000


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

#ifndef DATA_SIZE
#  define DATA_SIZE  (SZ_1M * 96)  /* OMX is a little piggy */
#endif

/* Resource info: Must match include/linux/remoteproc.h: */
#define TYPE_CARVEOUT    0
#define TYPE_DEVMEM      1
#define TYPE_DEVICE      2
#define TYPE_IRQ         3
#define TYPE_TRACE       4
#define TYPE_ENTRYPOINT  5
#define TYPE_VRING       6

struct resource {
    u32 type;
    u32 id;
    u32 da_low;       /* Device (Ducati virtual) Address */
    u32 da_high;
    u32 pa_low;       /* Physical Address */
    u32 pa_high;
    u32 len;
    u32 flags;
    u32 pad1;
    u32 pad2;
    u32 pad3;
    u32 pad4;
    char name[48];
};

#pragma DATA_SECTION(resources, ".resource_table")
#pragma DATA_ALIGN(resources, 4096)
struct resource resources[] = {
    /*
     * memory allocation entries must come first. will be allocated
     * via CMA bofore M3 is booted.
     */
    { TYPE_CARVEOUT, 0, DATA_DA, 0, 0, 0, DATA_SIZE, 0,0,0,0,0, "IPU_MEM_DATA"},
    { TYPE_CARVEOUT, 0, TEXT_DA, 0, 0, 0, SZ_4M, 0, 0,0,0,0,"IPU_MEM_TEXT"},
    /*
     * Auxillary entries
     */
    { TYPE_VRING, 0, VRING0_DA, 0, 0, 0, 0x3000, 0, 0,0,0,0,"vring:sysm3->mpu"},
    { TYPE_VRING, 1, VRING1_DA, 0, 0, 0, 0x3000, 0, 0,0,0,0,"vring:mpu->sysm3"},
    { TYPE_VRING, 2, VRING2_DA, 0, 0, 0, 0x3000, 0, 0,0,0,0,"vring:appm3->mpu"},
    { TYPE_VRING, 3, VRING3_DA, 0, 0, 0, 0x3000, 0, 0,0,0,0,"vring:mpu->appm3"},
    { TYPE_TRACE, 0, 0, 0, 0, 0, 0x8000, 0, 0,0,0,0,"trace:sysm3"},
    { TYPE_TRACE, 1, 1, 0, 0, 0, 0x8000, 0, 0,0,0,0,"trace:appm3"},
    /* This is an evil hack that will be removed once Linux DMA API is ready */
    { TYPE_DEVMEM, 0, IPC_DA, 0, IPC_PA, 0, SZ_1M, 0, 0,0,0,0,"IPU_MEM_IPC"},
    /* These entries are essentiall IOMMU configuration requests */
    { TYPE_DEVMEM, 0, IPU_TILER_MODE_0_1, 0, L3_TILER_MODE_0_1, 0, SZ_256M,
       0, 0,0,0,0,"IPU_TILER_MODE_0_1"},
    { TYPE_DEVMEM, 0, IPU_TILER_MODE_2, 0, L3_TILER_MODE_2, 0, SZ_128M,
       0, 0,0,0,0,"IPU_TILER_MODE_2"},
    { TYPE_DEVMEM, 0, IPU_TILER_MODE_3, 0, L3_TILER_MODE_3, 0, SZ_128M,
       0, 0,0,0,0,"IPU_TILER_MODE_3"},
    { TYPE_DEVMEM, 0, IPU_PERIPHERAL_L4CFG, 0, L4_PERIPHERAL_L4CFG, 0, SZ_16M,
       0, 0,0,0,0,"IPU_PERIPHERAL_L4CFG"},
    { TYPE_DEVMEM, 0, IPU_PERIPHERAL_L4PER, 0, L4_PERIPHERAL_L4PER, 0, SZ_16M,
       0,0,0,0,0,"IPU_PERIPHERAL_L4PER"},
    { TYPE_DEVMEM, 0, IPU_IVAHD_CONFIG, 0, L3_IVAHD_CONFIG, 0, SZ_16M,
       0, 0,0,0,0,"IPU_IVAHD_CONFIG"},
    { TYPE_DEVMEM, 0, IPU_IVAHD_SL2, 0, L3_IVAHD_SL2, 0, SZ_16M,
       0, 0,0,0,0,"IPU_IVAHD_SL2"},
};

#endif /* _RSC_TABLE_H_ */
