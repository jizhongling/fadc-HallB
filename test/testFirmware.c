/*
 * File:
 *    firmwareTest.c
 *
 * Description:
 *    Check the firmware file for proper formatting.
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "fadcLib.h"

char *progName;

void
Usage();

int 
main(int argc, char *argv[]) {

    char *mcs_filename;

    printf("\nJLAB fadc firmware update\n");
    printf("----------------------------\n");

    progName = argv[0];

    if(argc!=2)
      {
	printf(" ERROR: Must specify one arguments\n");
	Usage();
	exit(-1);
      }
    else
      {
	mcs_filename = argv[1];
      }

    if(fadcFirmwareReadMcsFile(mcs_filename) != OK)
      {
	exit(-1);
      }
    
    fadcFirmwareChipFromFpgaID(1);
    fadcFirmwareRevFromFpgaID(1);

    exit(0);
}


void
Usage()
{
  printf("\n");
  printf("%s <firmware MCS file>\n",progName);
  printf("\n");

}
