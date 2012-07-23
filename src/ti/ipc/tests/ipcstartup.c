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

/*
 *  ======== ipcStartupTask ========
 */
static Void ipcStartupTask(UArg arg0, UArg arg1)
{
    UInt procId = MultiProc_getId("HOST");
    Int status;

    /* TransportVirtioSetup will busy wait until host kicks ready to recv: */
    status = TransportVirtioSetup_attach(procId, 0);
    Assert_isTrue(status >= 0, NULL);

    /* Sets up to comminicate with host's NameServer: */
    status = ti_sdo_utils_NameServer_SetupProxy_attach(procId, 0);
    Assert_isTrue(status >= 0, NULL);
}

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
