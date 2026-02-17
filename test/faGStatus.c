/*
 * File:
 *    faGStatus.c
 *
 * Description:
 *    Show the status of all fadcs found in the crate
 *
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "fadcLib.h"

int
main(int argc, char *argv[])
{

    int status;

    status = vmeOpenDefaultWindows();
    if(status != OK)
      goto CLOSE;

    vmeBusLock();
    /* Set the FADC structure pointer */
    faInit( 3 << 19 , 1 << 19, 18, FA_INIT_SKIP);

    faGStatus(0);
    vmeBusUnlock();


 CLOSE:
    vmeCloseDefaultWindows();

    exit(0);
}
