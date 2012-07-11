/*
 *  ======== ipcstartup.c ========
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/sdo/utils/_NameServer.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/transports/TransportVirtioSetup.h>
#include <ti/ipc/rpmsg/Rpmsg.h>
#include <ti/ipc/transports/_TransportVirtio.h>

/*
 *  ======== ipcStartupTask ========
 */
static Void ipcStartupTask(UArg arg0, UArg arg1)
{
    UInt procId = MultiProc_getId("HOST");
    Int status;

    /* call TransportVirtioSetup to attach to remote processor */
    status = TransportVirtioSetup_attach(procId, 0);
    Assert_isTrue(status >= 0, NULL);

    /* call NameServer_attach to remote processor */
    status = ti_sdo_utils_NameServer_SetupProxy_attach(procId, 0);
    Assert_isTrue(status >= 0, NULL);

    /*
     * Tell the Linux host we have a MessageQ service over rpmsg.
     *
     * TBD: This should be in the VirtioTransport initialization, but we need
     * an interrupt handshake after BIOS_start().
     * TBD: Also, NameMap should go over a bare rpmsg API, rather than
     * MessageQCopy, as this clashes with MessageQ.
     */
    nameService_register("rpmsg-proto", RPMSG_MESSAGEQ_PORT, RPMSG_NS_CREATE);
};

/*
 *  ======== ipcStartup ========
 */
Void ipcStartup()
{
    Task_Params params;

    System_printf("creating IPC startup task ...\n");

    Task_Params_init(&params);
    params.priority = Task_numPriorities - 1;
    if (Task_create(ipcStartupTask, &params, NULL) == NULL) {
        System_abort("ipcStartup: could not create startup task\n");
    }
}
