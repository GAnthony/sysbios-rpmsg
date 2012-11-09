#
# Copyright (c) 2011-2012 Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Repo
REPO            = /db/atree/gp/repo

# Edit Dependency Versions:
XDCROOTVER      = xdctools_3_23_02_47
BIOSPRODVER	= bios_6_32_05_54
IPCPRODVER	= ipc_1_24_00_16

BIOSPROD	= $(REPO)/$(BIOSPRODVER)
IPCPROD		= $(REPO)/$(IPCPRODVER)
XDCDIST_TREE	= $(REPO)/$(XDCROOTVER)

export XDCROOT	= $(XDCDIST_TREE)

export XDCPATH	= $(BIOSPROD)/packages;$(IPCPROD)/packages;./src;

#ti.targets.arm.elf.M3 = /db/toolsrc/library/vendors2005/ti/arm/4.9.0/Linux
#ti.targets.elf.C64T = /db/toolsrc/library/vendors2005/ti/c6x/7.2.0/Linux
#ti.targets.elf.C674 = /db/toolsrc/library/vendors2005/ti/c6x/7.2.0/Linux
ti.targets.elf.C66 = /db/toolsrc/library/vendors2005/ti/c6x/7.2.0/Linux

PKGLIST = src/ti/ipc/tests

#PKGLIST = src/ti/ipc/family/omapl138 \
#	src/ti/ipc/namesrv \
#	src/ti/ipc/rpmsg \
#	src/ti/ipc/tests \
#	src/ti/ipc/transports \
#	src/ti/ipc/ipcmgr \
#	src/ti/resources

XDCARGS = \
    ti.targets.arm.elf.M3=\"$(ti.targets.arm.elf.M3)\" \
    ti.targets.elf.C64T=\"$(ti.targets.elf.C64T)\" \
    ti.targets.elf.C674=\"$(ti.targets.elf.C674)\" \
    ti.targets.elf.C66=\"$(ti.targets.elf.C66)\"

all:
	$(XDCROOT)/xdc XDCARGS="$(XDCARGS)" release -j $(j) -P $(PKGLIST)

clean:
	$(XDCROOT)/xdc clean -j $(j) -P $(PKGLIST)

.PHONY: tags
tags:
	ctags -R src/
	cscope -R -b -ssrc/
