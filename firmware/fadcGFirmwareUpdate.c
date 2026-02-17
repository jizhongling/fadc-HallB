/*
 * File:
 *    firmwareGTest.c
 *
 * Description:
 *    Test JLab Flash ADC firmware updating
 *
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "fadcLib.h"

#define FADC_ADDR (3<<19)
#define NFADC     16
#define SKIPSS

extern int nfadc;
char *progName;

void Usage();

int 
main(int argc, char *argv[]) 
{

    GEF_STATUS status;
    int fpga_choice, firmware_choice=0;
    char *mcs_filename;
    char inputchar[16];
    int ifa=0;
    unsigned int cfw=0;

    printf("\nJLAB fadc firmware update\n");
    printf("----------------------------\n");

    progName = argv[0];

    if(argc<2)
      {
	printf(" ERROR: Must specify one argument\n");
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


    fpga_choice = fadcFirmwareChipFromFpgaID(0);
    if( fpga_choice == ERROR )
      {
	printf(" ERROR: Did not obtain FPGA type from firmware file.\n");
	printf("        Please specific FPGA type\n");
	printf("          1 for FX70T (Control FPGA)\n");
	printf("          2 for LX110 (Processing FPGA)\n");
	printf("    or q and <ENTER> to quit without update\n");
	printf("\n");

	REPEAT:
	printf(" (y/n): ");
	scanf("%s",(char *)&inputchar);

	if((strcmp(inputchar,"q")==0) || (strcmp(inputchar,"Q")==0))
	  {
	    printf("--- Exiting without update ---\n");
	    exit(0);
	  }
	else if(strcmp(inputchar,"1")==0)
	  {
	    fpga_choice = 1;
	  }
	else if(strcmp(inputchar,"2")==0)
	  {
	    fpga_choice = 2;
	  }
	else
	  {
	    goto REPEAT;
	  }
	
      }

    vmeSetQuietFlag(1);
    status = vmeOpenDefaultWindows();

    vmeCheckMutexHealth(10);
    vmeBusLock();
    int iFlag = (1<<18); // Skip firmware check

#ifdef SKIPSS
    faInit((unsigned int)(FADC_ADDR),(1<<19),NFADC+2,iFlag);
#else
    faInit((unsigned int)(FADC_ADDR),(1<<19),NFADC,iFlag);
#endif

    if(nfadc==0)
      {
	printf(" Unable to initialize any FADCs.\n");
	goto CLOSE;
      }

    for(ifa=0; ifa<nfadc; ifa++)
      {
	cfw = faGetFirmwareVersions(faSlot(ifa),0);
	printf("%2d: Control Firmware Version: 0x%04x   Proc Firmware Version: 0x%04x\n",
	       faSlot(ifa),cfw&0xFFFF,(cfw>>16)&0xFFFF);
      }

    printf(" Will update firmware for ");
    if(fpga_choice==1)
      {
	firmware_choice = FADC_FIRMWARE_FX70T;
	printf("FX70T (Control FPGA) ");
      }
    else if((fpga_choice==2)||(fpga_choice==0))
      {
	firmware_choice = FADC_FIRMWARE_LX110;
	printf("LX110 (Processing FPGA) ");
      }

    printf(" with file: \n   %s",mcs_filename);
    if(fadcFirmwareRevFromFpgaID(0))
      {
	printf(" (rev = 0x%x)\n",fadcFirmwareRevFromFpgaID(0));
      }
    else
      {
	printf("\n");
      }

 REPEAT2:
    printf(" Press y and <ENTER> to continue... n or q and <ENTER> to quit without update\n");

    scanf("%s",(char *)inputchar);

    if((strcmp(inputchar,"q")==0) || (strcmp(inputchar,"Q")==0) ||
       (strcmp(inputchar,"n")==0) || (strcmp(inputchar,"N")==0) )
      {
	printf(" Exiting without update\n");
	goto CLOSE;
      }
    else if((strcmp(inputchar,"y")==0) || (strcmp(inputchar,"Y")==0))
      {}
    else
      goto REPEAT2;

    fadcFirmwareGLoad(firmware_choice,0);

    goto CLOSE;

 CLOSE:

    vmeBusUnlock();
    status = vmeCloseDefaultWindows();
    if (status != GEF_SUCCESS)
    {
      printf("vmeCloseDefaultWindows failed: code 0x%08x\n",status);
      return -1;
    }

    exit(0);
}


void
Usage()
{
  printf("\n");
  printf("%s <firmware MCS file>\n",progName);
  printf("\n");

}
