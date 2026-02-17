/********************************************************************************
 *  fadcLib.c  -  Driver library for JLAB config and readout of JLAB 250MHz FLASH
 *                  ADC using a VxWorks >=5.4 or Linux >=2.6.18 based Single
 *                  Board computer.
 *
 *  Author: David Abbott & Bryan Moffit
 *          Jefferson Lab Data Acquisition Group
 *          June 2007
 *
 *  Revision  2.0 - Initial Revision for FADC V2
 *                    - Supports up to 20 FADC boards in a Crate
 *                    - Programmed I/O and Block reads
 *
 *  SVN: $Rev$
 *
 */

/*
  Sergey:

  fadcID() returns SLOT NUMBER, NOT ID !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  FAp[slot#]

*/
#if defined(VXWORKS) || defined(Linux_vme)


#ifdef VXWORKS
#include <vxWorks.h>
/*sergey#include "vxCompat.h"*/
#else
#include <stddef.h>
#include <pthread.h>
#include "jvme.h"
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef VXWORKS
#include <logLib.h>
#include <taskLib.h>
#include <intLib.h>
#include <iv.h>
#include <semLib.h>
#include <vxLib.h>
#else
#include <unistd.h>
#endif


/* Include ADC definitions */
#include "fadcLib.h"

#undef DEBUG

#ifdef VXWORKS
#define FALOCK
#define FAUNLOCK
#define FASDCLOCK
#define FASDCUNLOCK
#else
/* Mutex to guard flexio read/writes */
pthread_mutex_t   faMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t   fasdcMutex = PTHREAD_MUTEX_INITIALIZER;
#define FALOCK      if(pthread_mutex_lock(&faMutex)<0) perror("pthread_mutex_lock");
#define FAUNLOCK    if(pthread_mutex_unlock(&faMutex)<0) perror("pthread_mutex_unlock");
#define FASDCLOCK   if(pthread_mutex_lock(&fasdcMutex)<0) perror("pthread_mutex_lock");
#define FASDCUNLOCK if(pthread_mutex_unlock(&fasdcMutex)<0) perror("pthread_mutex_unlock");
#endif

/* Define external Functions */
#ifdef VXWORKS
IMPORT  STATUS sysBusToLocalAdrs(int, char *, char **);
IMPORT  STATUS intDisconnect(int);
IMPORT  STATUS sysIntEnable(int);
IMPORT  STATUS sysIntDisable(int);
IMPORT  STATUS sysVmeDmaDone(int, int);
IMPORT  STATUS sysVmeDmaSend(UINT32, UINT32, int, BOOL);

#define EIEIO    __asm__ volatile ("eieio")
#define SYNC     __asm__ volatile ("sync")
#endif

/* Define Interrupts variables */
BOOL              fadcIntRunning  = FALSE;                    /* running flag */
int               fadcIntID       = -1;                       /* id number of ADC generating interrupts */
LOCAL VOIDFUNCPTR fadcIntRoutine  = NULL;                     /* user interrupt service routine */
LOCAL int         fadcIntArg      = 0;                        /* arg to user routine */
LOCAL UINT32      fadcIntLevel    = FA_VME_INT_LEVEL;         /* default VME interrupt level */
LOCAL UINT32      fadcIntVec      = FA_VME_INT_VEC;           /* default interrupt Vector */

/* Define global variables */
int nfadc = 0;                                       /* Number of FADCs in Crate */
unsigned int  fadcA32Base   = 0x09000000;            /* Minimum VME A32 Address for use by FADCs */
unsigned long fadcA32Offset = 0x08000000;            /* Difference in CPU A32 Base - VME A32 Base */
unsigned long fadcA24Offset = 0x0;                   /* Difference in CPU A24 Base - VME A24 Base */
unsigned long fadcA16Offset = 0x0;                   /* Difference in CPU A16 Base - VME A16 Base */
volatile struct fadc_struct *FAp[(FA_MAX_BOARDS+1)]; /* pointers to FADC memory map */
volatile struct fadc_sdc_struct *FASDCp;             /* pointer to FADC Signal distribution card */
volatile unsigned int *FApd[(FA_MAX_BOARDS+1)];      /* pointers to FADC FIFO memory */
volatile unsigned int *FApmb;                        /* pointer to Multblock window */
int fadcID[FA_MAX_BOARDS];                           /* array of slot numbers for FADCs */
unsigned int fadcAddrList[FA_MAX_BOARDS];            /* array of a24 addresses for FADCs */
int fadcRev[(FA_MAX_BOARDS+1)];                      /* Board Revision Info for each module */
int fadcProcRev[(FA_MAX_BOARDS+1)];                  /* Processing FPGA Revision Info for each module */
unsigned short fadcChanDisable[(FA_MAX_BOARDS+1)];   /* Disabled Channel Mask for each Module*/
int fadcInited=0;                                    /* >0 if Library has been Initialized before */
int fadcMaxSlot=0;                                   /* Highest Slot hold an FADC */
int fadcMinSlot=0;                                   /* Lowest Slot holding an FADC */
int fadcSource=0;                                    /* Signal source for FADC system control*/
int fadcBlockLevel=0;                                /* Block Level for ADCs */
int fadcIntCount = 0;                                /* Count of interrupts from FADC */
int fadcUseSDC=0;                                    /* If > 0 then Use Signal Distribution board */
int fadcSDCPassthrough=0;                            /* If > 0 SDC in level translate / passthrough mode */
struct fadc_data_struct fadc_data;
int fadcBlockError=FA_BLOCKERROR_NO_ERROR; /* Whether (>0) or not (0) Block Transfer had an error */

/*sergey*/
static int proc_mode[(FA_MAX_BOARDS+1)];

/* Include Firmware Tools */
#ifdef HALLB
#include "cinclude/fadcFirmwareTools.c"
#else
#include "fadcFirmwareTools.c"
#endif

/*******************************************************************************
 *
 * faInit - Initialize JLAB FADC Library.
 *
 *
 *   iFlag: 18 bit integer
 *       Low 6 bits - Specifies the default Signal distribution (clock,trigger)
 *                    sources for the board (Internal, FrontPanel, VXS, VME(Soft))
 *       bit    0:  defines Sync Reset source
 *                     0  VME (Software Sync-Reset)
 *                     1  Front Panel/VXS/P2 (Depends on Clk/Trig source selection)
 *       bits 3-1:  defines Trigger source
 *               0 0 0  VME (Software Triggers)
 *               0 0 1  Front Panel Input
 *               0 1 0  VXS (P0)
 *               1 0 0  Internal Trigger Logic (HITSUM FPGA)
 *               (all others Undefined - default to Internal)
 *       bits 5-4:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel
 *           1 0  VXS (P0)
 *           1 1  P2 Connector (Backplane)
 *
 *       Common Modes of Operation:
 *           Value = 0  CLK (Int)  TRIG (Soft)   SYNC (Soft)    (Debug/Test Mode)
 *                   2  CLK (Int)  TRIG (FP)     SYNC (Soft)    (Single Board
 *                   3  CLK (Int)  TRIG (FP)     SYNC (FP)         Modes)
 *                0x10  CLK (FP)   TRIG (Soft)   SYNC (Soft)
 *                0x13  CLK (FP)   TRIG (FP)     SYNC (FP)      (VME SDC Mode)
 *                0x20  CLK (VXS)  TRIG (Soft)   SYNC (Soft)
 *                0x25  CLK (VXS)  TRIG (VXS)    SYNC (VXS)     (VXS SD Mode)
 *
 *
 *      High 10bits - A16 Base address of FADC Signal Distribution Module
 *                    This board can control up to 7 FADC Boards.
 *                    Clock Source must be set to Front Panel (bit4 = 1)
 *
 *      bit 16:  Exit before board initialization
 *             0 Initialize FADC (default behavior)
 *             1 Skip initialization (just setup register map pointers)
 *
 *      bit 17:  Use fadcAddrList instead of addr and addr_inc
 *               for VME addresses.
 *             0 Initialize with addr and addr_inc
 *             1 Use fadcAddrList
 *
 *      bit 18:  Skip firmware check.  Useful for firmware updating.
 *             0 Perform firmware check
 *             1 Skip firmware check
 *
 *
 * RETURNS: OK, or ERROR if the address is invalid or a board is not present.
 */

STATUS
faInit (UINT32 addr, UINT32 addr_inc, int nadc, int iFlag)
{
  int ii, res, errFlag = 0;
  int boardID = 0;
  int maxSlot = 1;
  int minSlot = 21;
  int trigSrc=0, clkSrc=0, srSrc=0;
  unsigned int rdata, a32addr, a16addr=0;
  unsigned long laddr=0, laddr_inc=0;
  volatile struct fadc_struct *fa;
  unsigned short sdata;
  int noBoardInit=0;
  int useList=0;
  int noFirmwareCheck=0;
  unsigned short supported_ctrl[FA_SUPPORTED_CTRL_FIRMWARE_NUMBER]
    = {FA_SUPPORTED_CTRL_FIRMWARE};
  unsigned short supported_proc[FA_SUPPORTED_PROC_FIRMWARE_NUMBER]
    = {FA_SUPPORTED_PROC_FIRMWARE};
  unsigned short ctrl_version = 0, proc_version = 0;
  int icheck=0, ctrl_supported=0, proc_supported=0;

  /* Check if we have already Initialized boards before */
  if((fadcInited>0) && (fadcID[0] != 0))
    {
      /* Hard Reset of all FADC boards in the Crate */
      for(ii=0;ii<nfadc;ii++)
	{
	  vmeWrite32(&(FAp[fadcID[ii]]->csr),FA_CSR_HARD_RESET);
	}
      taskDelay(5);
    }

  /* Check if we are to exit when pointers are setup */
  noBoardInit=(iFlag&FA_INIT_SKIP)>>16;

  /* Check if we're initializing using a list */
  useList=(iFlag&FA_INIT_USE_ADDRLIST)>>17;

  /* Are we skipping the firmware check? */
  noFirmwareCheck=(iFlag&FA_INIT_SKIP_FIRMWARE_CHECK)>>18;

  /* Check for valid address */
  if(addr==0)
    {
      printf("faInit: ERROR: Must specify a Bus (VME-based A24) address for FADC 0\n");
      return(ERROR);
    }
  else if(addr > 0x00ffffff)
    { /* A24 Addressing */
      printf("faInit: ERROR: A32 Addressing not allowed for FADC configuration space\n");
      return(ERROR);
    }
  else
    { /* A24 Addressing */
      if( ((addr_inc==0)||(nadc==0)) && (useList==0) )
	nadc = 1; /* assume only one FADC to initialize */

      /* get the FADC address */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x39,(char *)addr,(char **)&laddr);
#else
      res = vmeBusToLocalAdrs(0x39,(char *)(unsigned long)addr,(char **)&laddr);
#endif
      if (res != 0)
	{
#ifdef VXWORKS
	  printf("faInit: ERROR in sysBusToLocalAdrs(0x39,0x%x,&laddr) \n",addr);
#else
	  printf("faInit: ERROR in vmeBusToLocalAdrs(0x39,0x%x,&laddr) \n",addr);
#endif
	  return(ERROR);
	}
      fadcA24Offset = laddr - addr;
    }

  /* Init Some Global variables */
  fadcSource = iFlag&FA_SOURCE_MASK;
  fadcInited = nfadc = 0;
  fadcUseSDC = 0;
  bzero((char *)fadcChanDisable,sizeof(fadcChanDisable));
  bzero((char *)fadcID,sizeof(fadcID));

  for (ii=0;ii<nadc;ii++)
    {
      if(useList==1)
	{
	  laddr_inc = fadcAddrList[ii] + fadcA24Offset;
	}
      else
	{
	  laddr_inc = laddr +ii*addr_inc;
	}
      fa = (struct fadc_struct *)laddr_inc;
      /* Check if Board exists at that address */
#ifdef VXWORKS
      res = vxMemProbe((char *) &(fa->version),VX_READ,4,(char *)&rdata);
#else
      res = vmeMemProbe((char *) &(fa->version),4,(char *)&rdata);
#endif
      if(res < 0)
	{
#ifdef DEBUG
#ifdef VXWORKS
	  printf("faInit: WARN: No addressable board at addr=0x%x\n",(UINT32) fa);
#else
	  printf("faInit: WARN: No addressable board at VME (Local) addr=0x%x (0x%lx)\n",
		 (UINT32)(laddr_inc-fadcA24Offset), (unsigned long) fa);
#endif
#endif
	  errFlag = 1;
	  continue;
	}
      else
	{
	  /* Check that it is an FA board */
	  if((rdata&FA_BOARD_MASK) != FA_BOARD_ID)
	    {
#ifdef DEBUG
	      printf("%s: WARN: For board at 0x%x, Invalid Board ID: 0x%x\n",
		     __FUNCTION__,
		     (UINT32) fa-fadcA24Offset, rdata);
#endif
	      continue;
	    }
	  else
	    {
	      ctrl_supported=0;
	      proc_supported=0;

	      /* Check if this is board has a valid slot number */
	      boardID =  ((vmeRead32(&(fa->intr)))&FA_SLOT_ID_MASK)>>16;

	      if((boardID <= 0)||(boardID >21))
		{
		  printf(" ERROR: Board Slot ID is not in range: %d\n",boardID);
		  continue;
		  /* 	      return(ERROR); */
		}
	      else
		{
		  /* Check Control FPGA firmware version */
		  ctrl_version = rdata & FA_VERSION_MASK;

		  for(icheck = 0; icheck < FA_SUPPORTED_CTRL_FIRMWARE_NUMBER; icheck++)
		    {
		      if(ctrl_version == supported_ctrl[icheck])
		      	ctrl_supported=1;
		    }

		  if(ctrl_supported == 0)
		    {
		      printf("%s: %s: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __FUNCTION__, (noFirmwareCheck)?"WARN":"ERROR",
			     boardID, ctrl_version);

		      printf("\tSupported Control Firmware:  ");
		      for(icheck=0; icheck<FA_SUPPORTED_CTRL_FIRMWARE_NUMBER; icheck++)
			{
			  printf("0x%02x ",supported_ctrl[icheck]);
			}
		      printf("\n");

		      if(!noFirmwareCheck)
			{ /* Skip to the next fADC */
			  continue;
			}
		    }

		  /* Check Processing FPGA firmware version */
		  proc_version = (unsigned short)(vmeRead32(&fa->adc_status[0]) & FA_ADC_VERSION_MASK);

		  for(icheck = 0; icheck < FA_SUPPORTED_PROC_FIRMWARE_NUMBER; icheck++)
		    {
		      if(proc_version == supported_proc[icheck]) 							proc_supported=1;
		    }

		  if(proc_supported == 0)
		    {
		      printf("%s: %s: Slot %2d: Proc FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __FUNCTION__, (noFirmwareCheck)?"WARN":"ERROR",
			     boardID, proc_version);

		      printf("\tSupported Proc Firmware:  ");
		      for(icheck=0; icheck<FA_SUPPORTED_PROC_FIRMWARE_NUMBER; icheck++)
			{
			  printf("0x%04x ",supported_proc[icheck]);
			}
		      printf("\n");

		      if(!noFirmwareCheck)
			{ /* Skip to the next fADC */
			  continue;
			}
		    }

		  FAp[boardID] = (struct fadc_struct *)(laddr_inc);
		  fadcRev[boardID] = rdata&FA_VERSION_MASK;
		  fadcProcRev[boardID] = proc_version;
		  fadcID[nfadc] = boardID;
		  if(boardID >= maxSlot) maxSlot = boardID;
		  if(boardID <= minSlot) minSlot = boardID;

		  printf("Initialized FADC %2d  Slot #%2d at VME (Local) address 0x%06x (0x%lx) \n",
			 nfadc,fadcID[nfadc],
			 (UINT32) (((unsigned long)FAp[(fadcID[nfadc])])-fadcA24Offset),
			 (unsigned long) FAp[(fadcID[nfadc])]);
		}
	      nfadc++;
	      /* 	  printf("Initialized FADC %2d  Slot # %2d at address 0x%08x \n", */
	      /* 		 ii,fadcID[ii],(UINT32) FAp[(fadcID[ii])]); */
	    }
	}
    } // End loop through fadcs


      /* Check if we are using a JLAB FADC Signal Distribution Card (SDC)
	 NOTE the SDC board only supports 7 FADCs - so if there are
	 more than 7 FADCs in the crate they can only be controlled by daisychaining
	 multiple SDCs together - or by using a VXS Crate with SD switch card
      */
  a16addr = iFlag&FA_SDC_ADR_MASK;
  if(a16addr)
    {
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x29,(char *)a16addr,(char **)&laddr);
      if (res != 0)
	{
	  printf("faInit: ERROR in sysBusToLocalAdrs(0x29,0x%x,&laddr) \n",a16addr);
	  return(ERROR);
	}

      res = vxMemProbe((char *) laddr,VX_READ,2,(char *)&sdata);
#else
      res = vmeBusToLocalAdrs(0x29,(char *)(unsigned long)a16addr,(char **)&laddr);
      if (res != 0)
	{
	  printf("faInit: ERROR in vmeBusToLocalAdrs(0x29,0x%x,&laddr) \n",a16addr);
	  return(ERROR);
	}
      res = vmeMemProbe((char *) laddr,2,(char *)&sdata);
#endif
      if(res < 0)
	{
	  printf("faInit: ERROR: No addressable SDC board at addr=0x%x\n",(UINT32) laddr);
	}
      else
	{
	  fadcA16Offset = laddr-a16addr;
	  FASDCp = (struct fadc_sdc_struct *) laddr;
	  if(!noBoardInit)
	    vmeWrite16(&(FASDCp->ctrl),FASDC_CSR_INIT);   /* Reset the Module */

	  if(nfadc>7)
	    {
	      printf("WARN: A Single JLAB FADC Signal Distribution Module only supports 7 FADCs\n");
	      printf("WARN: You must use multiple SDCs to support more FADCs - this must be configured in hardware\n");
	    }
#ifdef VXWORKS
	  printf("Using JLAB FADC Signal Distribution Module at address 0x%x\n",
		 (UINT32) FASDCp);
#else
	  printf("Using JLAB FADC Signal Distribution Module at VME (Local) address 0x%x (0x%lx)\n",
		 (UINT32)a16addr, (unsigned long) FASDCp);
#endif
	  fadcUseSDC=1;
	}

      if(fadcSource == FA_SOURCE_SDC)
	{  /* Check if SDC will be used */
	  fadcUseSDC = 1;
	  printf("faInit: JLAB FADC Signal Distribution Card is Assumed in Use\n");
	  printf("faInit: Front Panel Inputs will be enabled. \n");
	}
      else
	{
	  fadcUseSDC = 0;
	  printf("faInit: JLAB FADC Signal Distribution Card will not be Used\n");
	}
    } // end if a16addr

  /* Hard Reset of all FADC boards in the Crate */
  if(!noBoardInit)
    {
      for(ii=0;ii<nfadc;ii++)
	{
	  vmeWrite32(&(FAp[fadcID[ii]]->reset),FA_RESET_ALL);
	}
      taskDelay(60);
    }

  /* Initialize Interrupt variables */
  fadcIntID = -1;
  fadcIntRunning = FALSE;
  fadcIntLevel = FA_VME_INT_LEVEL;
  fadcIntVec = FA_VME_INT_VEC;
  fadcIntRoutine = NULL;
  fadcIntArg = 0;

  /* Calculate the A32 Offset for use in Block Transfers */
#ifdef VXWORKS
  res = sysBusToLocalAdrs(0x09,(char *)fadcA32Base,(char **)&laddr);
  if (res != 0)
    {
      printf("faInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",fadcA32Base);
      return(ERROR);
    }
  else
    {
      fadcA32Offset = laddr - fadcA32Base;
    }
#else
  res = vmeBusToLocalAdrs(0x09,(char *)(unsigned long)fadcA32Base,(char **)&laddr);
  if (res != 0)
    {
      printf("faInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",fadcA32Base);
      return(ERROR);
    }
  else
    {
      fadcA32Offset = laddr - fadcA32Base;
    }
#endif

  if(!noBoardInit)
    {
      /* what are the Trigger Sync Reset and Clock sources */
      if (fadcSource == FA_SOURCE_VXS)
	{
	  printf("faInit: Enabling FADC for VXS Clock ");
	  clkSrc  = FA_REF_CLK_P0;
	  switch (iFlag&0xf)
	    {
	    case 0: case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_VME | FA_ENABLE_SOFT_TRIG;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 2:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_FP_ISYNC;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = FA_TRIG_FP_ISYNC;
	      srSrc   = FA_SRESET_FP_ISYNC;
	      break;
	    case 4: case 6:
	      printf("and VXS Triggers (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_P0_ISYNC;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 5: case 7:
	      printf("and VXS Triggers (VXS Sync Reset)\n");
	      trigSrc = FA_TRIG_P0_ISYNC;
	      srSrc   = FA_SRESET_P0_ISYNC;
	      break;
	    case 8: case 10: case 12: case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_INTERNAL;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 9: case 11: case 13: case 15:
	      printf("and Internal Trigger Logic (VXS Sync Reset)\n");
	      trigSrc = FA_TRIG_INTERNAL;
	      srSrc   = FA_SRESET_FP_ISYNC;
	      break;
	    }
	}
      else if (fadcSource == FA_SOURCE_SDC)
	{
	  printf("faInit: Enabling FADC for SDC Clock (Front Panel) ");
	  clkSrc  = FA_REF_CLK_FP;
	  switch (iFlag&0xf)
	    {
	    case 0: case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_VME | FA_ENABLE_SOFT_TRIG;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 2: case 4: case 6:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_FP_ISYNC;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 3: case 5: case 7:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = FA_TRIG_FP_ISYNC;
	      srSrc   = FA_SRESET_FP_ISYNC;
	      break;
	    case 8: case 10: case 12: case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_INTERNAL;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 9: case 11: case 13: case 15:
	      printf("and Internal Trigger Logic (Front Panel Sync Reset)\n");
	      trigSrc = FA_TRIG_INTERNAL;
	      srSrc   = FA_SRESET_FP_ISYNC;
	      break;
	    }
	  faSDC_Config(0,0);
	}
      else
	{  /* Use internal Clk */
	  printf("faInit: Enabling FADC Internal Clock, ");
	  clkSrc = FA_REF_CLK_INTERNAL;
	  switch (iFlag&0xf)
	    {
	    case 0: case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_VME | FA_ENABLE_SOFT_TRIG;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET ;
	      break;
	    case 2:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_FP_ISYNC;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = FA_TRIG_FP_ISYNC;
	      srSrc   = FA_SRESET_FP_ISYNC;
	      break;
	    case 4: case 6:
	      printf("and VXS Triggers (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_P0_ISYNC;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 5: case 7:
	      printf("and VXS Triggers (VXS Sync Reset)\n");
	      trigSrc = FA_TRIG_P0_ISYNC;
	      srSrc   = FA_SRESET_P0_ISYNC;
	      break;
	    case 8: case 10: case 12: case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = FA_TRIG_INTERNAL;
	      srSrc   = FA_SRESET_VME | FA_ENABLE_SOFT_SRESET;
	      break;
	    case 9: case 11: case 13: case 15:
	      printf("and Internal Trigger Logic (Front Panel Sync Reset)\n");
	      trigSrc = FA_TRIG_INTERNAL;
	      srSrc   = FA_SRESET_FP_ISYNC;
	      break;
	    }
	}
    }

  /* Enable Clock source - Internal Clk enabled by default */
  if(!noBoardInit)
    {
      for(ii=0;ii<nfadc;ii++)
	{
	  vmeWrite32(&(FAp[fadcID[ii]]->ctrl1),(clkSrc | FA_ENABLE_INTERNAL_CLK)) ;
	}
      taskDelay(20);


      /* Hard Reset FPGAs and FIFOs */
      for(ii=0;ii<nfadc;ii++)
	{
	  vmeWrite32(&(FAp[fadcID[ii]]->reset),
		     (FA_RESET_ADC_FPGA1 | FA_RESET_ADC_FIFO1 |
		      FA_RESET_DAC | FA_RESET_EXT_RAM_PT));

#ifdef CLAS12
	  vmeWrite32(&(FAp[fadcID[ii]]->gtx_ctrl),0x203); /*put reset*/
	  vmeWrite32(&(FAp[fadcID[ii]]->gtx_ctrl),0x800); /*release reset*/
#else
	  /* #ifdef USEMGTCTRL */
	  /* Release reset on MGTs */
	  vmeWrite32(&(FAp[fadcID[ii]]->mgt_ctrl),FA_MGT_RESET);
	  vmeWrite32(&(FAp[fadcID[ii]]->mgt_ctrl),FA_RELEASE_MGT_RESET);
	  vmeWrite32(&(FAp[fadcID[ii]]->mgt_ctrl),FA_MGT_RESET);
	  /* #endif */
#endif

	}
      taskDelay(5);
    }

  /* Write configuration registers with default/defined Sources */
  for(ii=0;ii<nfadc;ii++)
    {

      /* Program an A32 access address for this FADC's FIFO */
      a32addr = fadcA32Base + ii*FA_MAX_A32_MEM;
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0)
	{
	  printf("faInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  return(ERROR);
	}
#else
      res = vmeBusToLocalAdrs(0x09,(char *)(unsigned long)a32addr,(char **)&laddr);
      if (res != 0)
	{
	  printf("faInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  return(ERROR);
	}
#endif
      FApd[fadcID[ii]] = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAp[fadcID[ii]]->adr32),(a32addr>>16) + 1);  /* Write the register and enable */

	  /* Set Default Block Level to 1 */
	  vmeWrite32(&(FAp[fadcID[ii]]->blk_level),1);
	}
      fadcBlockLevel=1;

      /* Setup Trigger and Sync Reset sources */
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAp[fadcID[ii]]->ctrl1),
		     vmeRead32(&(FAp[fadcID[ii]]->ctrl1)) |
		     (srSrc | trigSrc) );
	}
    } //End loop through fadcs

  /* If there are more than 1 FADC in the crate then setup the Muliblock Address
     window. This must be the same on each board in the crate */
  if(nfadc > 1)
    {
      a32addr = fadcA32Base + (nfadc+1)*FA_MAX_A32_MEM; /* set MB base above individual board base */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0)
	{
	  printf("faInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  return(ERROR);
	}
#else
      res = vmeBusToLocalAdrs(0x09,(char *)(unsigned long)a32addr,(char **)&laddr);
      if (res != 0)
	{
	  printf("faInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",a32addr);
	  return(ERROR);
	}
#endif
      FApmb = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  for (ii=0;ii<nfadc;ii++)
	    {
	      /* Write the register and enable */
	      vmeWrite32(&(FAp[fadcID[ii]]->adr_mb),
			 (a32addr+FA_MAX_A32MB_SIZE) + (a32addr>>16) + FA_A32_ENABLE);
	    }
	}
      /* Set First Board and Last Board */
      fadcMaxSlot = maxSlot;
      fadcMinSlot = minSlot;
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAp[minSlot]->ctrl1),
		     vmeRead32(&(FAp[minSlot]->ctrl1)) | FA_FIRST_BOARD);
	  vmeWrite32(&(FAp[maxSlot]->ctrl1),
		     vmeRead32(&(FAp[maxSlot]->ctrl1)) | FA_LAST_BOARD);
	}
    }

  fadcInited = nfadc;
  if(errFlag > 0)
    {
#ifdef DEBUG
      printf("faInit: WARN: Unable to initialize all requested FADC Modules (%d)\n",
	     nadc);
#endif
      if(nfadc > 0)
	printf("faInit: %d FADC(s) successfully initialized\n",nfadc );
      return(ERROR);
    }
  else
    {
      return(OK);
    }
} //End of faInit

void
faSetA32BaseAddress(unsigned int addr)
{
  fadcA32Base = addr;
  printf("fadc A32 base address set to 0x%08X\n",fadcA32Base);
}

/*******************************************************************************
 *
 * faSlot - Convert an index into a slot number, where the index is
 *          the element of an array of FADCs in the order in which they were
 *          initialized.
 *
 * RETURNS: Slot number if Successfull, otherwise ERROR.
 *
 */

int
faSlot(unsigned int i)
{
  if(i>=nfadc)
    {
      printf("%s: ERROR: Index (%d) >= FADCs initialized (%d).\n",
	     __FUNCTION__,i,nfadc);
      return ERROR;
    }

  return fadcID[i];
}

/*******************************************************************************
 *
 * faSetClockSource - Set the clock source
 *
 *   This routine should be used in the case that the source clock
 *   is NOT set in faInit (and defaults to Internal).  Such is the case
 *   when clocks are synchronized in a many crate system.  The clock source
 *   of the FADC should ONLY be set AFTER those clocks have been set and
 *   synchronized.
 *
 *   clkSrc: 2 bit integer
 *       bits 1-0:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel
 *           1 0  VXS (P0)
 *           1 1  VXS (P0)
 *
 */

int
faSetClockSource(int id, int clkSrc)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(clkSrc>0x3)
    {
      printf("%s: ERROR: Invalid Clock Source specified (0x%x)\n",
	     __FUNCTION__,clkSrc);
      return ERROR;
    }

  /* Enable Clock source - Internal Clk enabled by default */
  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     (vmeRead32(&FAp[id]->ctrl1) & ~(FA_REF_CLK_MASK)) |
	     (clkSrc | FA_ENABLE_INTERNAL_CLK)) ;
  taskDelay(20);
  FAUNLOCK;

  switch(clkSrc)
    {
    case FA_REF_CLK_INTERNAL:
      printf("%s: FADC id %d clock source set to INTERNAL\n",
	     __FUNCTION__,id);
      break;

    case FA_REF_CLK_FP:
      printf("%s: FADC id %d clock source set to FRONT PANEL\n",
	     __FUNCTION__,id);
      break;

    case FA_REF_CLK_P0:
      printf("%s: FADC id %d clock source set to VXS (P0)\n",
	     __FUNCTION__,id);
      break;

    case FA_REF_CLK_MASK:
      printf("%s: FADC id %d clock source set to VXS (P0)\n",
	     __FUNCTION__,id);
      break;
    }

  return OK;
}

int
faGSetClockSource(int clkSrc)
{
  int ifa, id;
  if(clkSrc>0x3)
    {
      printf("%s: ERROR: Invalid Clock Source specified (0x%x)\n",
	     __FUNCTION__,clkSrc);
      return ERROR;
    }

  /* Enable Clock source - Internal Clk enabled by default */
  FALOCK;
  for(ifa = 0; ifa < nfadc; ifa++)
    {
      id = faSlot(ifa);
      vmeWrite32(&(FAp[id]->ctrl1),
		 (vmeRead32(&FAp[id]->ctrl1) & ~(FA_REF_CLK_MASK)) |
		 (clkSrc | FA_ENABLE_INTERNAL_CLK)) ;
    }
  taskDelay(20);
  FAUNLOCK;

  switch(clkSrc)
    {
    case FA_REF_CLK_INTERNAL:
      printf("%s: FADC clock source set to INTERNAL\n",
	     __FUNCTION__);
      break;

    case FA_REF_CLK_FP:
      printf("%s: FADC clock source set to FRONT PANEL\n",
	     __FUNCTION__);
      break;

    case FA_REF_CLK_P0:
      printf("%s: FADC clock source set to VXS (P0)\n",
	     __FUNCTION__);
      break;

    case FA_REF_CLK_MASK:
      printf("%s: FADC clock source set to VXS (P0)\n",
	     __FUNCTION__);
      break;
    }

  return OK;
}

void
faStatus(int id, int sflag)
{
  int ii;
  unsigned int a32Base, ambMin, ambMax, vers;
  unsigned int csr, ctrl1, ctrl2, count, bcount, blevel, intr, addr32, addrMB;
  unsigned int adcStat[3], adcConf[3],
    PTW, PL, NSB, NSA, NP, adcChanDisabled, playbackMode;
  unsigned int adc_enabled, adc_version, adc_option;
  unsigned int trigCnt, trig2Cnt, srCnt, itrigCnt, ramWords;
  unsigned int mgtStatus, mgtCtrl;
  unsigned int berr_count=0;
  unsigned int scaler_interval=0;
  unsigned int trigger_control=0;
  unsigned int lost_trig_scal=0;
  unsigned int tet_trg[16], tet_readout[16], delay[16];
  float gain_trg[16], ped_trg[16];
  unsigned int val, trig_mode[16], inverted[16],playback_ch[16];
  char *trig_mode_string;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return;
    }

  FALOCK;
  vers   =  vmeRead32(&FAp[id]->version);

  csr    = (vmeRead32(&(FAp[id]->csr)))&FA_CSR_MASK;
  ctrl1  = (vmeRead32(&(FAp[id]->ctrl1)))&FA_CONTROL_MASK;
  ctrl2  = (vmeRead32(&(FAp[id]->ctrl2)))&FA_CONTROL2_MASK;
  count  = (vmeRead32(&(FAp[id]->ev_count)))&FA_EVENT_COUNT_MASK;
  bcount = (vmeRead32(&(FAp[id]->blk_count)))&FA_BLOCK_COUNT_MASK;
  blevel  = (vmeRead32(&(FAp[id]->blk_level)))&FA_BLOCK_LEVEL_MASK;
  ramWords = (vmeRead32(&(FAp[id]->ram_word_count)))&FA_RAM_DATA_MASK;
  trigCnt = vmeRead32(&(FAp[id]->trig_scal));
  trig2Cnt = vmeRead32(&FAp[id]->trig2_scal);
  srCnt = vmeRead32(&FAp[id]->syncreset_scal);
  itrigCnt = vmeRead32(&(FAp[id]->internal_trig_scal));
  intr   = vmeRead32(&(FAp[id]->intr));
  addr32 = vmeRead32(&(FAp[id]->adr32));
  a32Base = (addr32&FA_A32_ADDR_MASK)<<16;
  addrMB = vmeRead32(&(FAp[id]->adr_mb));
  ambMin =  (addrMB&FA_AMB_MIN_MASK)<<16;
  ambMax =  (addrMB&FA_AMB_MAX_MASK);
  berr_count = vmeRead32(&(FAp[id]->berr_module_scal));

  for(ii=0;ii<3;ii++)
    {
      adcStat[ii] = (vmeRead32(&(FAp[id]->adc_status[ii]))&0xFFFF);
      adcConf[ii] = (vmeRead32(&(FAp[id]->adc_config[ii]))&0xFFFF);
    }
  PTW =  (vmeRead32(&(FAp[id]->adc_ptw))&0xFFFF)*FA_ADC_NS_PER_CLK;
  PL  =  (vmeRead32(&(FAp[id]->adc_pl))&0xFFFF)*FA_ADC_NS_PER_CLK;
  NSB =  (vmeRead32(&(FAp[id]->adc_nsb))&0xFFFF)*FA_ADC_NS_PER_CLK;
  NSA =  (vmeRead32(&(FAp[id]->adc_nsa))&0xFFFF)*FA_ADC_NS_PER_CLK;
  adc_version = adcStat[0]&FA_ADC_VERSION_MASK;
  adc_option  = (adcConf[0]&FA_ADC_PROC_MASK) + 1;
  NP          = (adcConf[0]&FA_ADC_PEAK_MASK)>>4;
  adc_enabled = (adcConf[0]&FA_ADC_PROC_ENABLE);
  playbackMode = (adcConf[0]&FA_ADC_PLAYBACK_MODE)>>7;
  adcChanDisabled = (adcConf[1]&FA_ADC_CHAN_MASK);

#ifdef CLAS12
  mgtStatus = vmeRead32(&(FAp[id]->gtx_status));
  mgtCtrl = vmeRead32(&(FAp[id]->gtx_ctrl));

  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      val = vmeRead32(&FAp[id]->adc_gain[ii]) & 0xFFFF;
      if(val & 0x8000) trig_mode[ii] = 1;
      else             trig_mode[ii] = 0;
      gain_trg[ii] = ((float)(val & 0x7FFF)) / 256.0f;
      delay[ii] = vmeRead16(&FAp[id]->adc_delay[ii]) & FA_ADC_DELAY_MASK;

      ped_trg[ii] = 4.0 * ((float)(vmeRead16(&FAp[id]->adc_pedestal[ii]) & FA_ADC_PEDESTAL_MASK)) / ((float)(NSA+NSB));

      val = vmeRead16(&(FAp[id]->adc_thres[ii]));
      tet_trg[ii] = (val & FA_THR_VALUE_MASK) - (int)ped_trg[ii];
      tet_readout[ii] = (val & FA_THR_IGNORE_MASK) ? 0 : ((val & FA_THR_VALUE_MASK) - (int)ped_trg[ii]);
      inverted[ii] = (val & FA_THR_INVERT_MASK) ? 1 : 0;
      playback_ch[ii] = (val & FA_PLAYBACK_DIS_MASK) ? 1 : 0;
    }

#else
  mgtStatus = vmeRead32(&(FAp[id]->mgt_status));
#endif

  scaler_interval = vmeRead32(&FAp[id]->scaler_interval) & FA_SCALER_INTERVAL_MASK;
  trigger_control = vmeRead32(&FAp[id]->trigger_control);

  FAUNLOCK;

#ifdef VXWORKS
  printf("\nSTATUS for FADC in slot %d at base address 0x%x \n",
	 id, (UINT32) FAp[id]);
#else
  printf("\nSTATUS for FADC in slot %d at VME (Local) base address 0x%x (0x%lx)\n",
	 id, (UINT32)(unsigned long)(FAp[id] - fadcA24Offset), (unsigned long) FAp[id]);
#endif
  printf("---------------------------------------------------------------------- \n");

  printf(" Board Firmware Rev/ID = 0x%04x : ADC Processing Rev = 0x%04x\n",
	 (vers)&0xffff, adc_version);
  if(addrMB&FA_AMB_ENABLE)
    {
      printf(" Alternate VME Addressing: Multiblock Enabled\n");
      if(addr32&FA_A32_ENABLE)
	printf("   A32 Enabled at VME (Local) base 0x%08x (0x%lx)\n",a32Base,(unsigned long) FApd[id]);
      else
	printf("   A32 Disabled\n");

      printf("   Multiblock VME Address Range 0x%08x - 0x%08x\n",ambMin,ambMax);
    }
  else
    {
      printf(" Alternate VME Addressing: Multiblock Disabled\n");
      if(addr32&FA_A32_ENABLE)
	printf("   A32 Enabled at VME (Local) base 0x%08x (0x%lx)\n",a32Base,(unsigned long) FApd[id]);
      else
	printf("   A32 Disabled\n");
    }

  if(ctrl1&FA_INT_ENABLE_MASK)
    {
      printf("\n  Interrupts ENABLED: ");
      if(ctrl1&FA_ENABLE_BLKLVL_INT) printf(" on Block Level(%d)",blevel);

      printf("\n");
      printf("  Interrupt Reg: 0x%08x\n",intr);
      printf("  VME INT Vector = 0x%x  Level = %d\n",(intr&FA_INT_VEC_MASK),((intr&FA_INT_LEVEL_MASK)>>8));
    }

  printf("\n Signal Sources: \n");

  if((ctrl1&FA_REF_CLK_MASK)==FA_REF_CLK_INTERNAL)
    {
      printf("   Ref Clock : Internal\n");
    }
  else if((ctrl1&FA_REF_CLK_MASK)==FA_REF_CLK_P0)
    {
      printf("   Ref Clock : VXS\n");
    }
  else if((ctrl1&FA_REF_CLK_MASK)==FA_REF_CLK_FP)
    {
      printf("   Ref Clock : Front Panel\n");
    }
  else
    {
      printf("   Ref Clock : %d (Undefined)\n",(ctrl1&FA_REF_CLK_MASK));
    }

  switch(ctrl1&FA_TRIG_MASK)
    {
    case FA_TRIG_INTERNAL:
      printf("   Trig Src  : Internal\n");
      break;
    case FA_TRIG_VME:
      printf("   Trig Src  : VME (Software)\n");
      break;
    case FA_TRIG_P0_ISYNC:
      printf("   Trig Src  : VXS (Async)\n");
      break;
    case FA_TRIG_P0:
      printf("   Trig Src  : VXS (Sync)\n");
      break;
    case FA_TRIG_FP_ISYNC:
      printf("   Trig Src  : Front Panel (Async)\n");
      break;
    case FA_TRIG_FP:
      printf("   Trig Src  : Front Panel (Sync)\n");
    }

  switch(ctrl1&FA_SRESET_MASK)
    {
    case FA_SRESET_VME:
      printf("   Sync Reset: VME (Software)\n");
      break;
    case FA_SRESET_P0_ISYNC:
      printf("   Sync Reset: VXS (Async)\n");
      break;
    case FA_SRESET_P0:
      printf("   Sync Reset: VXS (Sync)\n");
      break;
    case FA_SRESET_FP_ISYNC:
      printf("   Sync Reset: Front Panel (Async)\n");
      break;
    case FA_SRESET_FP:
      printf("   Sync Reset: Front Panel (Sync)\n");
    }

  if(fadcUseSDC)
    {
      printf("   SDC       : In Use\n");
    }


  printf("\n Configuration: \n");

  if(ctrl1&FA_ENABLE_INTERNAL_CLK)
    printf("   Internal Clock ON\n");
  else
    printf("   Internal Clock OFF\n");

  if(ctrl1&FA_ENABLE_BERR)
    printf("   Bus Error ENABLED\n");
  else
    printf("   Bus Error DISABLED\n");


  if(ctrl1&FA_ENABLE_MULTIBLOCK)
    {
      int tP0, tP2;
      tP0 = ctrl1&FA_MB_TOKEN_VIA_P0;
      tP2 = ctrl1&FA_MB_TOKEN_VIA_P2;

      if(tP0)
	{
	  if(ctrl1&FA_FIRST_BOARD)
	    printf("   MultiBlock transfer ENABLED (First Board - token via VXS)\n");
	  else if(ctrl1&FA_LAST_BOARD)
	    printf("   MultiBlock transfer ENABLED (Last Board  - token via VXS)\n");
	  else
	    printf("   MultiBlock transfer ENABLED (Token via VXS)\n");
	  /* #ifdef VERSION1 */
	}
      else if(tP2)
	{
	  if(ctrl1&FA_FIRST_BOARD)
	    printf("   MultiBlock transfer ENABLED (First Board - token via P2)\n");
	  else if(ctrl1&FA_LAST_BOARD)
	    printf("   MultiBlock transfer ENABLED (Last Board  - token via P2)\n");
	  else
	    printf("   MultiBlock transfer ENABLED (Token via P2)\n");
	  /* #endif */
	}
      else
	{
	  printf("   MultiBlock transfer ENABLED (**NO Tokens enabled**)\n");
	}
    }
  else
    {
      printf("   MultiBlock transfer DISABLED\n");
    }

  if(ctrl1&FA_ENABLE_SOFT_TRIG)
    printf("   Software Triggers   ENABLED\n");
  if(ctrl1&FA_ENABLE_SOFT_SRESET)
    printf("   Software Sync Reset ENABLED\n");


  printf("\n ADC Processing Configuration: \n");
  printf("   Channel Disable Mask = 0x%04x\n",adcChanDisabled);
  if(adc_enabled)
    printf("   Mode = %d  (ENABLED)\n",adc_option);
  else
    printf("   Mode = %d  (Disabled)\n",adc_option);

  printf("   Lookback (PL)    = %d ns   Time Window (PTW) = %d ns\n",PL,PTW);
  printf("   Time Before Peak = %d ns   Time After Peak   = %d ns\n",NSB,NSA);
  printf("   Max Peak Count   = %d \n",NP);
  printf("   Playback Mode    = %d \n",playbackMode);



  printf("\n");
  printf(" Unacknowleged Trigger Stop: %s (%d)\n",
	 (trigger_control&FA_TRIGCTL_TRIGSTOP_EN) ? " ENABLED" : "DISABLED",
	 (trigger_control&FA_TRIGCTL_MAX2_MASK)>>16);
  printf(" Unacknowleged Trigger Busy: %s (%d)\n",
	 (trigger_control&FA_TRIGCTL_BUSY_EN) ? " ENABLED" : "DISABLED",
	 trigger_control&FA_TRIGCTL_MAX1_MASK);



  printf("\n");
  if(csr&FA_CSR_ERROR_MASK)
    {
      printf("  CSR       Register = 0x%08x - **Error Condition**\n",csr);
    }
  else
    {
      printf("  CSR       Register = 0x%08x\n",csr);
    }

  printf("  Control 1 Register = 0x%08x \n",ctrl1);


  if((ctrl2&FA_CTRL_ENABLE_MASK)==FA_CTRL_ENABLED)
    {
      printf("  Control 2 Register = 0x%08x - Enabled for triggers\n",ctrl2);
    }
  else
    {
      printf("  Control 2 Register = 0x%08x - Disabled\n",ctrl2);
    }



  if((ctrl2&FA_CTRL_COMPRESS_MASK)==FA_CTRL_COMPRESS_DISABLE)
    {
      printf("  Control 2 Register = 0x%08x - Compress disabled\n",ctrl2);
    }
  else if((ctrl2&FA_CTRL_COMPRESS_MASK)==FA_CTRL_COMPRESS_ENABLE)
    {
      printf("  Control 2 Register = 0x%08x - Compress enabled\n",ctrl2);
    }
  else if((ctrl2&FA_CTRL_COMPRESS_MASK)==FA_CTRL_COMPRESS_VERIFY)
    {
      printf("  Control 2 Register = 0x%08x - Compress verify\n",ctrl2);
    }
  else
    printf("  Control 2 Register = 0x%08x - Compress error\n",ctrl2);


  printf("  Internal Triggers (Live) = %d\n",itrigCnt);
  printf("  Trigger   Scaler         = %d\n",trigCnt);
  printf("  Trigger 2 Scaler         = %d\n",trig2Cnt);
  printf("  SyncReset Scaler         = %d\n",srCnt);
  printf("  Trigger Control          = 0x%08x\n",trigger_control);
  if(trigger_control & (FA_TRIGCTL_TRIGSTOP_EN | FA_TRIGCTL_BUSY_EN))
    {
      printf("  Lost Trigger Scaler      = %d\n",lost_trig_scal);
    }

  if(scaler_interval)
    {
      printf("  Block interval for scaler events = %d\n",scaler_interval);
    }

  if(csr&FA_CSR_BLOCK_READY)
    {
      printf("  Blocks in FIFO           = %d  (Block level = %d) - Block Available\n",bcount,blevel);
      printf("  RAM Level (Bytes)        = %d \n",(ramWords*8));
    }
  else if (csr&FA_CSR_EVENT_AVAILABLE)
    {
      printf("  Events in FIFO           = %d  (Block level = %d) - Data Available\n",count,blevel);
      printf("  RAM Level (Bytes)        = %d \n",(ramWords*8));
    }
  else
    {
      printf("  Events in FIFO           = %d  (Block level = %d)\n",count,blevel);
    }

  printf("  BERR count (from module) = %d\n",berr_count);

#ifdef CLAS12
  printf("  GTX Ctrl   Register      = 0x%08x\n",mgtCtrl);
  printf("  GTX Status Register      = 0x%08x, Errors:",mgtStatus);
  if( mgtCtrl & 0x1) printf(" Reset");
  if( (mgtStatus&0x2)==0 || (mgtStatus&0x4)==0 ) printf(" LaneErr");
  if( (mgtStatus&0x1)==0 ) printf(" ChannelErr");
  printf("\n\n");

  printf("  Ch| Readout - TET | Trigger - TET | GAIN   | PED     | DELAY  | TRGMODE| INVERT | PLAYBACK_DIS |\n");
  printf("  --|---------------|---------------|--------|---------|--------|--------|------- | ------------ |\n");
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
  {
    if(trig_mode[ii]) trig_mode_string = "DISC";
    else              trig_mode_string = "PULSE";
    printf("  %2d|          %4d |          %4d |%7.3f |%8.3f |%3d | %s | %5d | %3d\n", ii, tet_readout[ii], tet_trg[ii], gain_trg[ii], ped_trg[ii], delay[ii], trig_mode_string, inverted[ii],playback_ch[ii]);
  }

  printf("\n");
#else
  printf("  MGT Status Register      = 0x%08x ",mgtStatus);
  if(mgtStatus & (FA_MGT_GTX1_HARD_ERROR | FA_MGT_GTX1_SOFT_ERROR |
		  FA_MGT_GTX2_HARD_ERROR | FA_MGT_GTX2_SOFT_ERROR))
    printf(" - **Error Condition**\n");
  else
    printf("\n");
#endif



  /*sergey for testing*/
  printf("Some registers:\n");
  printf("0x30: 0x%08x\n",vmeRead32(&(FAp[id]->trig_scal)));
  printf("0x34: 0x%08x\n",vmeRead32(&(FAp[id]->ev_count)));
  printf("0x38: 0x%08x\n",vmeRead32(&(FAp[id]->blk_count)));
  printf("0x48: 0x%08x\n",vmeRead32(&(FAp[id]->ram_word_count)));
  printf("0x4C: 0x%08x\n",vmeRead32(&(FAp[id]->dataflow_status)));
  printf("0x70: 0x%08x\n",vmeRead32(&(FAp[id]->status[0])));
  printf("0x74: 0x%08x\n",vmeRead32(&(FAp[id]->status[1])));
  printf("0x78: 0x%08x\n",vmeRead32(&(FAp[id]->status[2])));
  printf("0x7C: 0x%08x\n",vmeRead32(&(FAp[id]->status[3])));
  printf("0x84: 0x%08x\n",vmeRead32(&(FAp[id]->trigger_control)));
  printf("0xA8: 0x%08x\n",vmeRead32(&(FAp[id]->proc_words_scal)));
  printf("0xAC: 0x%08x\n",vmeRead32(&(FAp[id]->aux_scal2)));
  printf("0xB0: 0x%08x\n",vmeRead32(&(FAp[id]->header_scal)));
  printf("0xB8: 0x%08x\n",vmeRead32(&(FAp[id]->trailer_scal)));
  printf("0xC0: 0x%08x\n",vmeRead32(&(FAp[id]->busy_level)));
  printf("\n");

}

void
faGStatus(int sflag)
{
  int ifa, id, ii;
  struct fadc_struct st[FA_MAX_BOARDS+1];
  unsigned int a24addr[FA_MAX_BOARDS+1];
  int nsb;

  FALOCK;
  for (ifa=0;ifa<nfadc;ifa++)
    {
      id = faSlot(ifa);
      a24addr[id]    = (unsigned int)((unsigned long)FAp[id] - fadcA24Offset);
      st[id].version = vmeRead32(&FAp[id]->version);
      st[id].adr32   = vmeRead32(&FAp[id]->adr32);
      st[id].adr_mb  = vmeRead32(&FAp[id]->adr_mb);

      st[id].ctrl1   = vmeRead32(&FAp[id]->ctrl1);
      st[id].ctrl2   = vmeRead32(&FAp[id]->ctrl2);

      st[id].csr     = vmeRead32(&FAp[id]->csr);

      st[id].system_monitor = vmeRead32(&FAp[id]->system_monitor);

      for(ii=0;ii<3;ii++)
	{
	  st[id].adc_status[ii] =  vmeRead32(&FAp[id]->adc_status[ii])&0xFFFF;
	  st[id].adc_config[ii] =  vmeRead32(&FAp[id]->adc_config[ii])&0xFFFF;
	}
      st[id].adc_ptw = vmeRead32(&FAp[id]->adc_ptw);
      st[id].adc_pl  = vmeRead32(&FAp[id]->adc_pl);
      st[id].adc_nsb = vmeRead32(&FAp[id]->adc_nsb);
      st[id].adc_nsa = vmeRead32(&FAp[id]->adc_nsa);

      st[id].blk_count = vmeRead32(&FAp[id]->blk_count);
      st[id].blk_level = vmeRead32(&FAp[id]->blk_level);
      st[id].ram_word_count = vmeRead32(&FAp[id]->ram_word_count)&FA_RAM_DATA_MASK;

      st[id].trig_scal        = vmeRead32(&(FAp[id]->trig_scal));
      st[id].trig2_scal       = vmeRead32(&FAp[id]->trig2_scal);
      st[id].syncreset_scal   = vmeRead32(&FAp[id]->syncreset_scal);
      st[id].berr_module_scal = vmeRead32(&FAp[id]->berr_module_scal);

      st[id].gtx_ctrl = vmeRead32(&FAp[id]->gtx_ctrl);
      st[id].gtx_status = vmeRead32(&FAp[id]->gtx_status);

      for(ii = 0; ii < FA_MAX_ADC_CHANNELS; ii++)
	{
	  st[id].adc_gain[ii]     = vmeRead32(&FAp[id]->adc_gain[ii]);
	  st[id].adc_delay[ii]    = vmeRead16(&FAp[id]->adc_delay[ii]);
	  st[id].adc_pedestal[ii] = vmeRead16(&FAp[id]->adc_pedestal[ii]);
	  st[id].adc_thres[ii]    = vmeRead16(&FAp[id]->adc_thres[ii]);
	}
    }
  FAUNLOCK;

  printf("\n");

  printf("                      fADC250 Module Configuration Summary\n\n");
  printf("     Firmware Rev   .................Addresses................\n");
  printf("Slot  Ctrl   Proc      A24        A32     A32 Multiblock Range   VXS Readout\n");
  printf("--------------------------------------------------------------------------------\n");

  for(ifa=0; ifa<nfadc; ifa++)
    {
      id = faSlot(ifa);
      printf(" %2d  ",id);

      printf("0x%04x 0x%04x  ",st[id].version&0xFFFF,
	     st[id].adc_status[0]&FA_ADC_VERSION_MASK);

      printf("0x%06x  ",
	     a24addr[id]);

      if(st[id].adr32 &FA_A32_ENABLE)
	{
	  printf("0x%08x  ",
		 (st[id].adr32&FA_A32_ADDR_MASK)<<16);
	}
      else
	{
	  printf("  Disabled  ");
	}

      if(st[id].adr_mb & FA_AMB_ENABLE)
	{
	  printf("0x%08x-0x%08x  ",
		 (st[id].adr_mb&FA_AMB_MIN_MASK)<<16,
		 (st[id].adr_mb&FA_AMB_MAX_MASK));
	}
      else
	{
	  printf("Disabled               ");
	}

      printf("%s",
	     (st[id].ctrl2 & FA_CTRL_VXS_RO_ENABLE) ? " Enabled" : "Disabled");

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");


  printf("\n");
  printf("      .Signal Sources..                        ..Channel...\n");
  printf("Slot  Clk   Trig   Sync     MBlk  Token  BERR  Enabled Mask\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa=0; ifa<nfadc; ifa++)
    {
      id = faSlot(ifa);
      printf(" %2d  ",id);

      printf("%s  ",
	     (st[id].ctrl1 & FA_REF_CLK_MASK)==FA_REF_CLK_INTERNAL ? " INT " :
	     (st[id].ctrl1 & FA_REF_CLK_MASK)==FA_REF_CLK_P0 ? " VXS " :
	     (st[id].ctrl1 & FA_REF_CLK_MASK)==FA_REF_CLK_FP ? "  FP " :
	     " ??? ");

      printf("%s  ",
	     (st[id].ctrl1 & FA_TRIG_MASK)==FA_TRIG_INTERNAL ? " INT " :
	     (st[id].ctrl1 & FA_TRIG_MASK)==FA_TRIG_VME ? " VME " :
	     (st[id].ctrl1 & FA_TRIG_MASK)==FA_TRIG_P0_ISYNC ? " VXS " :
	     (st[id].ctrl1 & FA_TRIG_MASK)==FA_TRIG_FP_ISYNC ? "  FP " :
	     (st[id].ctrl1 & FA_TRIG_MASK)==FA_TRIG_P0 ? " VXS " :
	     (st[id].ctrl1 & FA_TRIG_MASK)==FA_TRIG_FP ? "  FP " :
	     " ??? ");

      printf("%s    ",
	     (st[id].ctrl1 & FA_SRESET_MASK)==FA_SRESET_VME ? " VME " :
	     (st[id].ctrl1 & FA_SRESET_MASK)==FA_SRESET_P0_ISYNC ? " VXS " :
	     (st[id].ctrl1 & FA_SRESET_MASK)==FA_SRESET_FP_ISYNC ? "  FP " :
	     (st[id].ctrl1 & FA_SRESET_MASK)==FA_SRESET_P0 ? " VXS " :
	     (st[id].ctrl1 & FA_SRESET_MASK)==FA_SRESET_FP ? "  FP " :
	     " ??? ");

      printf("%s   ",
	     (st[id].ctrl1 & FA_ENABLE_MULTIBLOCK) ? "YES":" NO");

      printf("%s",
	     st[id].ctrl1 & (FA_MB_TOKEN_VIA_P0)?" P0":
	     st[id].ctrl1 & (FA_MB_TOKEN_VIA_P2)?" P0":
	     " NO");
      printf("%s  ",
	     st[id].ctrl1 & (FA_FIRST_BOARD) ? "-F":
	     st[id].ctrl1 & (FA_LAST_BOARD) ? "-L":
	     "  ");

      printf("%s     ",
	     st[id].ctrl1 & FA_ENABLE_BERR ? "YES" : " NO");

      printf("0x%04X",
	     ~(st[id].adc_config[1] & FA_ADC_CHAN_MASK) & 0xFFFF);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                         fADC250 Processing Mode Config\n\n");
  printf("      Block          ...[nanoseconds]...     \n");
  printf("Slot  Level  Mode    PL   PTW   NSB  NSA  NP   Compression  Playback\n");
  printf("--------------------------------------------------------------------------------\n");

  for(ifa=0; ifa<nfadc; ifa++)
    {
      id = faSlot(ifa);
      printf(" %2d    ",id);

      printf("%3d    ",st[id].blk_level & FA_BLOCK_LEVEL_MASK);

      printf("%2d   ",(st[id].adc_config[0] & FA_ADC_PROC_MASK) + 1);

      printf("%4d  ", (st[id].adc_pl & 0xFFFF)*FA_ADC_NS_PER_CLK);

      printf("%4d   ", ((st[id].adc_ptw & 0xFFFF) + 1)*FA_ADC_NS_PER_CLK);

      nsb = st[id].adc_nsb & FA_ADC_NSB_READBACK_MASK;
      nsb = (nsb & 0x7) * ((nsb & FA_ADC_NSB_NEGATIVE)?-1:1) * FA_ADC_NS_PER_CLK;
      printf("%3d  ", nsb);

      printf("%3d   ", (st[id].adc_nsa & FA_ADC_NSA_READBACK_MASK)*FA_ADC_NS_PER_CLK);

      printf("%1d      ", ((st[id].adc_config[0] & FA_ADC_PEAK_MASK)>>4) + 1);


      printf("%s  ",
	     ((st[id].ctrl2 & FA_CTRL_COMPRESS_MASK) == FA_CTRL_COMPRESS_DISABLE) ?
	     "Disabled" :
	     ((st[id].ctrl2 & FA_CTRL_COMPRESS_MASK) == FA_CTRL_COMPRESS_ENABLE) ?
	     " Enabled" :
	     ((st[id].ctrl2 & FA_CTRL_COMPRESS_MASK) == FA_CTRL_COMPRESS_VERIFY) ?
	     "  Verify" : "UNKNOWN");

      printf("%s   ",
	     (st[id].adc_config[0] &FA_ADC_PLAYBACK_MODE)>>7 ?" Enabled":"Disabled");

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("           .........fADC250 Signal Scalers..........     ..System Monitor..\n");
  printf("Slot       Trig1       Trig2   SyncReset        BERR     TempC   1.0V   2.5V\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa=0; ifa<nfadc; ifa++)
    {
      id = faSlot(ifa);
      printf(" %2d   ",id);

      printf("%10d  ", st[id].trig_scal);

      printf("%10d  ", st[id].trig2_scal);

      printf("%10d  ", st[id].syncreset_scal);

      printf("%10d     ", st[id].berr_module_scal);

      double fpga_temperature =
	(((double)(st[id].system_monitor & FA_SYSMON_CTRL_TEMP_MASK) ) * (503.975/1024.0))
	- 273.15;
      printf("%3.1f    ", fpga_temperature);

      double fpga_1V =
	(((double)((st[id].system_monitor & FA_SYSMON_FPGA_CORE_V_MASK)>>11)) * (3.0/1024.0));
      printf("%3.1f    ", fpga_1V);

      double fpga_25V =
	(((double)((st[id].system_monitor & FA_SYSMON_FPGA_AUX_V_MASK)>>22)) * (3.0/1024.0));
      printf("%3.1f    ", fpga_25V);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                              fADC250 Data Status\n\n");
  printf("                                                  .......Error Status.......\n");
  printf("      Trigger   Block                             Local   ....... MGT ......\n");
  printf("Slot  Source    Ready  Blocks In Fifo  RAM Level   Bus    Reset  Lane  Chan\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa=0; ifa<nfadc; ifa++)
    {
      id = faSlot(ifa);
      printf(" %2d  ",id);

      printf("%s    ",
	     st[id].ctrl2 & FA_CTRL_ENABLE_MASK ? " Enabled" : "Disabled");

      printf("%s       ",
	     st[id].csr & FA_CSR_BLOCK_READY ? "YES" : " NO");

      printf("%10d ",
	     st[id].blk_count&FA_BLOCK_COUNT_MASK);

      printf("%10d  ",
	     (st[id].ram_word_count&FA_RAM_DATA_MASK)*8);

      printf("%s     ",
	     st[id].csr & FA_CSR_ERROR_MASK ? "ERROR" : "  OK " );

      printf("%s  ",
	     (st[id].gtx_ctrl & 0x1) ?
	     " ON" : "OFF" );

      printf("%s  ",
	     ((st[id].gtx_status & 0x1) != 0x1) ? "Down" : " Up " );

      printf("%s ",
	     ((st[id].gtx_status & 0x6) != 0x6) ? "Down" : " Up " );

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                      fADC250 Trigger Path Processing\n\n");
  for(ifa=0; ifa<nfadc; ifa++)
    {
      printf("           .......TET.......                                           \n");
      printf("Slot  Ch   Readout   Trigger      Gain      Ped   Delay  TrigMode  Invert\n");
      printf("--------------------------------------------------------------------------------\n");

      id = faSlot(ifa);

      int ichan;
      for(ichan = 0; ichan < FA_MAX_ADC_CHANNELS; ichan++)
	{
	  if(ichan == 0)
	    printf(" %2d",id);
	  else
	    printf("   ");

	  printf("   ");
	  printf("%2d      ",ichan);

	  int NSB = (st[id].adc_nsb & 0xFFFF) * FA_ADC_NS_PER_CLK;
	  int NSA = (st[id].adc_nsa & 0xFFFF) * FA_ADC_NS_PER_CLK;
	  float gain_trg = (st[id].adc_gain[ichan] & 0x7FFF) / 256.0f;
	  float ped_trg = 4.0 * ((float)(st[id].adc_pedestal[ichan] & FA_ADC_PEDESTAL_MASK)) /
	    ((float)(NSA+NSB));

	  int tet_trg = (st[id].adc_thres[ichan] & FA_THR_VALUE_MASK) - (int)ped_trg;

	  int tet_readout = (st[id].adc_thres[ichan] & FA_THR_IGNORE_MASK) ? 0
	    : ((st[id].adc_thres[ichan] & FA_THR_VALUE_MASK) - (int)ped_trg);

	  printf("%4d      ", tet_readout);

	  printf("%4d   ", tet_trg);

	  printf("%7.3f ", gain_trg*1.);

	  printf("%8.3f     ", ped_trg);

	  printf("%3d     ", st[id].adc_delay[ichan]);

	  printf("%s       ", (st[id].adc_gain[ichan] & 0x8000) ? " DISC" : "PULSE");

	  printf("%d", (st[id].adc_thres[ichan] & FA_THR_INVERT_MASK) ? 1 : 0);

	  printf("\n");
	}

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("\n");

}

/***********************
 *
 *  faGetFirmwareVersions - Get the firmware versions of each FPGA
 *
 *    ARG:   pval
 *             0: Print nothing to stdout
 *            !0: Print firmware versions to stdout
 *
 *   RETURNS: (fpga_control.version) | (fpga_processing.version<<16)
 *            or -1 if error
 */
unsigned int
faGetFirmwareVersions(int id, int pflag)
{
  unsigned int cntl=0, proc=0, rval=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n",__FUNCTION__,id);
      return(ERROR);
    }

  FALOCK;
  /* Control FPGA firmware version */
  cntl = vmeRead32(&FAp[id]->version) & 0xFFFF;

  /* Processing FPGA firmware version */
  proc = vmeRead32(&(FAp[id]->adc_status[0]))&FA_ADC_VERSION_MASK;
  FAUNLOCK;

  rval = (cntl) | (proc<<16);

  if(pflag)
    {
      printf("%s:  Board Firmware Rev/ID = 0x%04x : ADC Processing Rev = 0x%04x\n",
	     __FUNCTION__,
	     cntl, proc);
    }

  return rval;
}

/***********************
 *
 *  faSetProcMode - Setup ADC processing modes.
 *
 *   VERSION2: bank is ignored
 */
int
faSetProcMode(int id, int pmode, unsigned int PL, unsigned int PTW,
	      unsigned int NSB, unsigned int NSA, unsigned int NP, int bank)
{

  int err=0;
  unsigned int ptw_last_adr, ptw_max_buf;


  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetProcMode: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if((pmode<=0)||(pmode>8))
    {
      printf("faSetProcMode: ERROR: Processing mode (%d) out of range (pmode= 1-8)\n",pmode);
      return(ERROR);
    }
  else
    {
      /*       if((pmode>3)&&(pmode<8))  */
      /* 	{ */
      /* 	  printf("faSetProcMode: ERROR: Processing mode (%d) not implemented \n",pmode); */
      /* 	} */
    }

  /*sergey*/
  proc_mode[id] = pmode;

  if(NP>4)
    {
      printf("faSetProcMode: ERROR: Invalid Peak count %d (must be 0-4)\n",NP);
      return(ERROR);
    }

  /*Defaults */
  if((PL==0)||(PL>FA_ADC_MAX_PL))  PL  = FA_ADC_DEFAULT_PL;
  if((PTW==0)||(PTW>FA_ADC_MAX_PTW)) PTW = FA_ADC_DEFAULT_PTW;
  if((NSB==0)||(NSB>FA_ADC_MAX_NSB)) NSB = FA_ADC_DEFAULT_NSB;
  if((NSA==0)||(NSA>FA_ADC_MAX_NSA)) NSA = FA_ADC_DEFAULT_NSA;
  if((NP==0)&&(pmode!=FA_ADC_PROC_MODE_WINDOW))  NP = FA_ADC_DEFAULT_NP;

  /* Consistancy check */
  if(PTW > PL)
    {
      err++;
      printf("faSetProcMode: ERROR: Window must be <= Latency\n");
    }
  if(((NSB+NSA)%2)==0)
    {
      err++;
      printf("faSetProcMode: ERROR: NSB+NSA must be an odd number\n");
    }

  /* Calculate Proc parameters */
  ptw_max_buf  = (unsigned int) (2016/(PTW + 8));
  ptw_last_adr = ptw_max_buf * (PTW + 8) - 1;

  /* Current firmware (version<=0x0208) requires a call to faSetNormalMode
     before enabling the window registers */
  faSetNormalMode(id,0);

  FALOCK;
  /* Disable ADC processing while writing window info */
  vmeWrite32(&(FAp[id]->adc_config[0]),
	     ((pmode-1) | (NP<<4) ) );
  vmeWrite32(&(FAp[id]->adc_config[1]), fadcChanDisable[id]);
  vmeWrite32(&(FAp[id]->adc_pl),PL);
  vmeWrite32(&(FAp[id]->adc_ptw),PTW);
  vmeWrite32(&(FAp[id]->adc_nsb),NSB);
  vmeWrite32(&(FAp[id]->adc_nsa),NSA);
  vmeWrite32(&(FAp[id]->ptw_max_buf),
	     ptw_max_buf);
  vmeWrite32(&(FAp[id]->ptw_last_adr),
	     ptw_last_adr);
  /* Enable ADC processing */
  vmeWrite32(&(FAp[id]->adc_config[0]),
	     ((pmode-1) | (NP<<4) | FA_ADC_PROC_ENABLE) );

  FAUNLOCK;

  return(OK);
}

int
faGetNSA(int id)
{
  int ret;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetProcMode: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  ret = vmeRead32(&(FAp[id]->adc_nsa)) & 0xFFFF;
  FAUNLOCK;

  /*  printf("faGetNSA returns %d\n",ret); */

  return(ret);
}

int
faGetNSB(int id)
{
  int ret;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetProcMode: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  ret = vmeRead32(&(FAp[id]->adc_nsb)) & 0xFFFF;
  FAUNLOCK;

  /*  printf("faGetNSB returns %d\n",ret); */

  return(ret);
}


void
faGSetProcMode(int pmode, unsigned int PL, unsigned int PTW,
	       unsigned int NSB, unsigned int NSA, unsigned int NP, int bank)
{
  int ii, res;

  for (ii=0;ii<nfadc;ii++)
    {
      res = faSetProcMode(fadcID[ii],pmode,PL,PTW,NSB,NSA,NP,bank);

      if(res<0) printf("ERROR: slot %d, in faSetProcMode()\n",fadcID[ii]);
    }

}

/*********************************************************************/
/*********************************************************************/
/* sergey: begin new functions from Bryan Moffit for trigger_control */

/**
 *  @ingroup Config
 *  @brief Return the maximum number of unacknowledged triggers a specific
 *         mode can handle.
 *
 *  @param pmode  Processing Mode
 *  @param ptw  Window Width
 *  @param nsb  Number of samples before pulse over threshold
 *  @param nsa  Number of samples after pulse over threshold
 *  @param np   Number of pulses processed per window
 *
 *  @return The minimum of 9 and the calculated maximum number of triggers
 *    allowed given specified mode and window paramters.
 */

int
faCalcMaxUnAckTriggers(int mode, int ptw, int nsa, int nsb, int np)
{
  int max;
  int imode=0, supported_modes[FA_SUPPORTED_NMODES] = {FA_SUPPORTED_MODES};
  int mode_supported=0;

  for(imode=0; imode<FA_SUPPORTED_NMODES; imode++)
    {
      if(mode == supported_modes[imode])
	mode_supported=1;
    }
  if(!mode_supported)
    {
      printf("%s: ERROR: Processing Mode (%d) not supported\n",
	     __FUNCTION__,mode);
      return ERROR;
    }

  switch(mode)
    {
    case 9: /* PULSE PARAMETER */
      max = (int)(1024 / ((np * 2) + 8));
      break;

    case 10: /* DEBUG */
      max = (int)(1024 / (((np * 2) + 8) + ptw + 1));
      break;

    default:
      printf("%s: ERROR: Mode %d is not supported\n",
	     __FUNCTION__,mode);
    }

  return ((max < 9) ? max : 9);
}

/**
 *  @ingroup Config
 *  @brief Set the maximum number of unacknowledged triggers before module
 *         stops accepting incoming triggers.
 *  @param id Slot number
 *  @param trigger_max Limit for maximum number of unacknowledged triggers.
 *         If 0, disables the condition.
 *  @return OK if successful, otherwise ERROR.
 */

int
faSetTriggerStopCondition(int id, int trigger_max)
{
  if(id==0) id=fadcID[0];
  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n",__FUNCTION__,id);
      return ERROR;
    }

  if(trigger_max>0xFF)
    {
      printf("%s: ERROR: Invalid trigger_max (%d)\n",
	     __FUNCTION__,trigger_max);
      return ERROR;
    }

  FALOCK;
  if(trigger_max>0)
    {
      vmeWrite32(&FAp[id]->trigger_control,
		 (vmeRead32(&FAp[id]->trigger_control) &
		  ~(FA_TRIGCTL_TRIGSTOP_EN | FA_TRIGCTL_MAX2_MASK)) |
		 (FA_TRIGCTL_TRIGSTOP_EN | (trigger_max<<16)));
    }
  else
    {
      vmeWrite32(&FAp[id]->trigger_control,
		 (vmeRead32(&FAp[id]->trigger_control) &
		  ~(FA_TRIGCTL_TRIGSTOP_EN | FA_TRIGCTL_MAX2_MASK)));
    }
  FAUNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the maximum number of unacknowledged triggers before module
 *         asserts BUSY.
 *  @param id Slot number
 *  @param trigger_max Limit for maximum number of unacknowledged triggers
 *         If 0, disables the condition
 *  @return OK if successful, otherwise ERROR.
 */

int
faSetTriggerBusyCondition(int id, int trigger_max)
{
  if(id==0) id=fadcID[0];
  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n",__FUNCTION__,id);
      return ERROR;
    }

  if(trigger_max>0xFF)
    {
      printf("%s: ERROR: Invalid trigger_max (%d)\n",
	     __FUNCTION__,trigger_max);
      return ERROR;
    }

  FALOCK;
  if(trigger_max>0)
    {
      vmeWrite32(&FAp[id]->trigger_control,
		 (vmeRead32(&FAp[id]->trigger_control) &
		  ~(FA_TRIGCTL_BUSY_EN | FA_TRIGCTL_MAX1_MASK)) |
		 (FA_TRIGCTL_BUSY_EN | (trigger_max)));
    }
  else
    {
      vmeWrite32(&FAp[id]->trigger_control,
		 (vmeRead32(&FAp[id]->trigger_control) &
		  ~(FA_TRIGCTL_BUSY_EN | FA_TRIGCTL_MAX1_MASK)));
    }
  FAUNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the number of samples that are included before and after
 *    threshold crossing that are sent through the trigger path
 *  @param id Slot number
 *  @param NSB Number of samples before threshold crossing
 *  @param NSA Number of samples after threshold crossing
 *  @return OK if successful, otherwise ERROR.
 */
int
faSetTriggerPathSamples(int id, unsigned int TNSA, unsigned int TNSAT)
{
  unsigned int readback_nsa=0, readback_config1=0;

  if(id==0) id=fadcID[0];
  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n",__FUNCTION__,id);
      return ERROR;
    }

  if(fadcProcRev[id]<0x90B)
    {
      printf("%s: ERROR: Processing Firmware does not support this function\n",
	     __FUNCTION__);
      printf("      Requires 0x90B and above\n");
      return ERROR;
    }

  if((TNSA < FA_ADC_MIN_TNSA) || (TNSA > FA_ADC_MAX_TNSA))
    {
      printf("%s: WARN: TNSA (%d) out of range. Setting to %d\n",
	     __FUNCTION__,
	     TNSA, FA_ADC_DEFAULT_TNSA);
      TNSA = FA_ADC_DEFAULT_TNSA;
    }

  if((TNSAT < FA_ADC_MIN_TNSAT) || (TNSAT > FA_ADC_MAX_TNSAT))
    {
      printf("%s: WARN: TNSAT (%d) out of range. Setting to %d\n",
	     __FUNCTION__,
	     TNSAT, FA_ADC_DEFAULT_TNSAT);
      TNSAT = FA_ADC_DEFAULT_TNSAT;
    }

  FALOCK;

  readback_nsa     = vmeRead32(&FAp[id]->adc_nsa)       & FA_ADC_NSA_READBACK_MASK;
  readback_config1 = vmeRead32(&FAp[id]->adc_config[0]) & ~FA_ADC_CONFIG1_TNSAT_MASK;

  vmeWrite32(&FAp[id]->adc_nsa,       (TNSA  << 9)  | readback_nsa);
  vmeWrite32(&FAp[id]->adc_config[0], ((TNSAT-1) << 12) | readback_config1);

  FAUNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the number of samples that are included before and after
 *    threshold crossing that are sent through the trigger path for
 *    all initialized fADC250s
 *  @param NSB Number of samples before threshold crossing
 *  @param NSA Number of samples after threshold crossing
 *  @sa faSetTriggerPathSamples
 */
void
faGSetTriggerPathSamples(unsigned int TNSA, unsigned int TNSAT)
{
  int ii, res;

  for (ii=0;ii<nfadc;ii++)
    {
      res = faSetTriggerPathSamples(fadcID[ii], TNSA, TNSAT);
      if(res<0) printf("ERROR: slot %d, in faSetTriggerPathSamples()\n",fadcID[ii]);
    }

}

/**
 *  @ingroup Config
 *  @brief Set the threshold used to determine what samples are sent through the
 *     trigger path
 *  @param id Slot number
 *  @param threshold Trigger Path Threshold
 *  @return OK if successful, otherwise ERROR.
 */
int
faSetTriggerPathThreshold(int id, unsigned int TPT)
{
  if(id==0) id=fadcID[0];
  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n",__FUNCTION__,id);
      return ERROR;
    }

  if(fadcProcRev[id]<0x90B)
    {
      printf("%s: ERROR: Processing Firmware does not support this function\n",
	     __FUNCTION__);
      printf("      Requires 0x90B and above\n");
      return ERROR;
    }

  if(TPT>FA_ADC_MAX_TPT)
    {
      printf("%s: WARN: TPT (%d) greater than MAX.  Setting to %d\n",
	     __FUNCTION__, TPT, FA_ADC_MAX_TPT);
      TPT = FA_ADC_MAX_TPT;
    }

#if 0
  FALOCK;
  vmeWrite32(&FAp[id]->config3,
	     (vmeRead32(&FAp[id]->config3) & ~FA_ADC_CONFIG3_TPT_MASK) |
	     TPT);
  FAUNLOCK;
#endif

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the threshold used to determine what samples are sent through the
 *     trigger path for all initialized fADC250s
 *  @param threshold Trigger Path Threshold
 *  @sa faSetTriggerPathThreshold
 */
void
faGSetTriggerPathThreshold(unsigned int TPT)
{
  int ii, res;

  for (ii=0;ii<nfadc;ii++)
    {
      res = faSetTriggerPathThreshold(fadcID[ii], TPT);
      if(res<0) printf("ERROR: slot %d, in faSetTriggerPathThreshold()\n",fadcID[ii]);
    }
}

/* sergey: end new functions from Bryan Moffit for trigger_control */
/*******************************************************************/
/*******************************************************************/





/*
 * faWaitForAdcReady()
 *   - Static routine, to wait for the ADC processing chip ready bit
 *     before proceeding with further programming
 *
 */
static void
faWaitForAdcReady(int id)
{
  int iwait=0;

  while((iwait<100) && (vmeRead32(&FAp[id]->adc_status[0])&0x8000)==0)
    {
      iwait++;
    }

  if(iwait==100)
    printf("%s: ERROR: Wait timeout.\n",__FUNCTION__);

}

/* faSetNormalMode
 *    - Configure the ADC Processing in "Normal Mode"
 *      This is temporary until the firmware is confirmed to be stable
 *
 */
void
faSetNormalMode(int id, int opt)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetProcMode: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[3], 0x0F02);
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0xC0);

  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[3], 0x179F);
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0xC0);

  /* 01dec2011 This portion commented out... would change the input gain */
  /*   faWaitForAdcReady(id); */
  /*   vmeWrite32(&FAp[id]->adc_config[3], 0x1811); */
  /*   faWaitForAdcReady(id); */
  /*   vmeWrite32(&FAp[id]->adc_config[2], 0x40); */
  /*   faWaitForAdcReady(id); */
  /*   vmeWrite32(&FAp[id]->adc_config[2], 0xC0);	 */

  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[3], 0xFF01);		/* transfer register values */
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0xC0);
  /*
    printf("%s: ---- FADC %2d ADC chips initialized ----\n",
    __FUNCTION__,id);
  */
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[3], 0x0D00);
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0xC0);

  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[3], 0xFF01);		/* transfer register values */
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAp[id]->adc_config[2], 0xC0);

  FAUNLOCK;


}

/***********************
 *
 *  faSetPPG - Setup FADC Progammable Pulse Generator
 *
 *
 */
int
faSetPPG(int id, int pmode, unsigned short *sdata, int nsamples)
{

  int ii;
  unsigned short rval;


  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetPPG: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(sdata == NULL)
    {
      printf("faSetPPG: ERROR: Invalid Pointer to sample data\n");
      return(ERROR);
    }

  /*Defaults */
  if((nsamples <= 0)||(nsamples>FA_PPG_MAX_SAMPLES))
    nsamples = FA_PPG_MAX_SAMPLES;

  FALOCK;
  for(ii=0;ii<(nsamples-2);ii++)
    {
      vmeWrite32(&FAp[id]->adc_test_data, (sdata[ii]|FA_PPG_WRITE_VALUE));
      rval = vmeRead32(&FAp[id]->adc_test_data);
      if( (rval&FA_PPG_SAMPLE_MASK) != sdata[ii])
	printf("faSetPPG: ERROR: Write error %x != %x (ii=%d)\n",rval, sdata[ii],ii);

    }

  vmeWrite32(&FAp[id]->adc_test_data, (sdata[(nsamples-2)]&FA_PPG_SAMPLE_MASK));
  rval = vmeRead32(&FAp[id]->adc_test_data);
  if(rval != sdata[(nsamples-2)])
    printf("faSetPPG: ERROR: Write error %x != %x\n",
	   rval, sdata[nsamples-2]);
  vmeWrite32(&FAp[id]->adc_test_data, (sdata[(nsamples-1)]&FA_PPG_SAMPLE_MASK));
  rval = vmeRead32(&FAp[id]->adc_test_data);
  if(rval != sdata[(nsamples-1)])
    printf("faSetPPG: ERROR: Write error %x != %x\n",
	   rval, sdata[nsamples-1]);

  /*   vmeWrite32(&FAp[id]->adc_test_data, (sdata[(nsamples-2)]&FA_PPG_SAMPLE_MASK)); */
  /*   vmeWrite32(&FAp[id]->adc_test_data, (sdata[(nsamples-1)]&FA_PPG_SAMPLE_MASK)); */

  FAUNLOCK;

  return(OK);
}

void
faPPGEnable(int id)
{
  unsigned short val1;

  if(id==0) id=fadcID[0];

  FALOCK;
  val1 = (vmeRead32(&FAp[id]->adc_config[0])&0xFFFF);
  val1 |= (FA_PPG_ENABLE | 0xff00);
  vmeWrite32(&FAp[id]->adc_config[0], val1);
  FAUNLOCK;

}

void
faPPGDisable(int id)
{
  unsigned short val1;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faPPGDisable: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  val1 = (vmeRead32(&FAp[id]->adc_config[0])&0xFFFF);
  val1 &= ~FA_PPG_ENABLE;
  val1 &= ~(0xff00);
  vmeWrite32(&FAp[id]->adc_config[0], val1);
  FAUNLOCK;

}


#ifdef VERSION1
/*************************************************************************************
 *
 *  faItrigBurstConfig - Setup Internal Trigger Burst control Parameters
 *
 *   ntrig        = max triggers (1-128) allowed in Burst Window
 *   burst_window = size (in clock ticks 4ns/tick) of Burst window (1 - 4 microsec)
 *   busy_period  = size (in clocks) of busy period to wait after max triggers reached
 *                   (0 - 262 microsec)
 */
int
faItrigBurstConfig(int id, unsigned int ntrig,
		   unsigned int burst_window, unsigned int busy_period)
{


  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) {
    logMsg("faItrigBurstConfig: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
    return(ERROR);
  }

  /* Set Defaults */
  if((ntrig==0)||(ntrig>128))                    ntrig        = 4;
  if((burst_window<0x100)||(burst_window>0x3ff)) burst_window = 0x200;
  if((busy_period==0)||(busy_period>0xffff))     busy_period  = 0x800;

  FALOCK;
  vmeWrite32(&(FAp[id]->itrig_burst_count),ntrig);
  vmeWrite32(&(FAp[id]->itrig_burst_ctrl),((busy_period)<<16) | burst_window);
  FAUNLOCK;

  return(OK);
}
#endif

/*
 * Set Internal trigger pulse width and deadtime between triggers
 *   Range for each :   4ns <-> 1020ns
 *
 *    Units are in clock ticks (4ns/tick)
 */
unsigned int
faItrigControl(int id, unsigned short itrig_width, unsigned short itrig_dt)
{
  unsigned int retval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) {
    logMsg("faItrigControl: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
    return(0xffffffff);
  }

  /* If both parameters = 0 then just return the current value */
  FALOCK;
  if((itrig_width==0)&&(itrig_dt==0)) {
    retval = vmeRead32(&(FAp[id]->itrig_cfg));
  }else{
    if((itrig_width==0)||(itrig_width>255))    itrig_width = 0xc; /* default 48ns */
    if((itrig_dt==0)||(itrig_dt>255))          itrig_dt    = 0xa; /* default 40ns */

    vmeWrite32(&(FAp[id]->itrig_cfg),(itrig_width<<16)|itrig_dt);
    retval = vmeRead32(&(FAp[id]->itrig_cfg));
  }
  FAUNLOCK;

  return(retval);
}



/**************************************************************************************
 *
 *  faReadBlock - General Data readout routine
 *
 *    id    - Slot number of module to read
 *    data  - local memory address to place data
 *    nwrds - Max number of words to transfer
 *    rflag - Readout Flag
 *              0 - programmed I/O from the specified board
 *              1 - DMA transfer using Universe/Tempe DMA Engine
 *                    (DMA VME transfer Mode must be setup prior)
 *              2 - Multiblock DMA transfer (Multiblock must be enabled
 *                     and daisychain in place or SD being used)
 */
int
faReadBlock(int id, volatile UINT32 *data, int nwrds, int rflag)
{
  int ii;
  int stat, retVal, xferCount, rmode, async;
  int dCnt, berr=0;
  int dummy=0;
  volatile unsigned int *laddr;
  unsigned int bhead, ehead, val;
  unsigned int vmeAdr, csr;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faReadBlock: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(data==NULL)
    {
      logMsg("faReadBlock: ERROR: Invalid Destination address\n",0,0,0,0,0,0);
      return(ERROR);
    }

  fadcBlockError=FA_BLOCKERROR_NO_ERROR;
  if(nwrds <= 0) nwrds= (FA_MAX_ADC_CHANNELS*FA_MAX_DATA_PER_CHANNEL) + 8;
  rmode = rflag&0x0f;
  async = rflag&0x80;

  if(rmode >= 1)
    { /* Block Transfers */

      /*Assume that the DMA programming is already setup. */
      /* Don't Bother checking if there is valid data - that should be done prior
	 to calling the read routine */

      /* Check for 8 byte boundary for address - insert dummy word (Slot 0 FADC Dummy DATA)*/
      if((unsigned long) (data)&0x7)
	{
#ifdef VXWORKS
	  *data = FA_DUMMY_DATA;
#else
	  *data = LSWAP(FA_DUMMY_DATA);
#endif
	  dummy = 1;
	  laddr = (data + 1);
	}
      else
	{
	  dummy = 0;
	  laddr = data;
	}

      FALOCK;
      if(rmode == 2)
	{ /* Multiblock Mode */
	  if((vmeRead32(&(FAp[id]->ctrl1))&FA_FIRST_BOARD)==0)
	    {
	      logMsg("faReadBlock: ERROR: FADC in slot %d is not First Board\n",id,0,0,0,0,0);
	      FAUNLOCK;
	      return(ERROR);
	    }
	  vmeAdr = (unsigned int)((unsigned long)(FApmb) - fadcA32Offset);
	}
      else
	{
	  vmeAdr = (unsigned int)((unsigned long)(FApd[id]) - fadcA32Offset);
	}

#ifdef HALLB
      /*
	printf("faReadBlock: fadcA32Offset=0x%08x vmeAdr=0x%08x laddr=0x%08x nwrds=%d\n",fadcA32Offset,vmeAdr,laddr,nwrds);fflush(stdout);
      */
      retVal = usrVme2MemDmaStart(vmeAdr, (UINT32)laddr, (nwrds<<2));
#else
#ifdef VXWORKS
      retVal = sysVmeDmaSend((UINT32)laddr, vmeAdr, (nwrds<<2), 0);
#else
      retVal = vmeDmaSend((unsigned long)laddr, vmeAdr, (nwrds<<2));
#endif
#endif
      if(retVal != 0)
	{
	  logMsg("faReadBlock: ERROR in DMA transfer Initialization 0x%x\n",retVal,0,0,0,0,0);
	  FAUNLOCK;
	  return(retVal);
	}

      if(async)
	{ /* Asynchronous mode - return immediately - don't wait for done!! */
	  FAUNLOCK;
	  return(OK);
	}
      else
	{
	  /* Wait until Done or Error */
#ifdef HALLB
	  retVal = usrVme2MemDmaDone();
#else
#ifdef VXWORKS
	  retVal = sysVmeDmaDone(10000,1);
#else
	  retVal = vmeDmaDone();
#endif
#endif
	}

      if(retVal > 0)
	{
	  /* Check to see that Bus error was generated by FADC */
	  if(rmode == 2)
	    {
	      csr = vmeRead32(&(FAp[fadcMaxSlot]->csr));  /* from Last FADC */
	    }
	  else
	    {
	      csr = vmeRead32(&(FAp[id]->csr));  /* from Last FADC */
	    }
	  stat = (csr)&FA_CSR_BERR_STATUS;

	  if((retVal>0) && (stat))
	    {
#ifdef HALLB
	      xferCount = ((retVal>>2) + dummy);  /* Number of Longwords transfered */
#else
#ifdef VXWORKS
	      xferCount = (nwrds - (retVal>>2) + dummy);  /* Number of Longwords transfered */
#else
	      xferCount = ((retVal>>2) + dummy);  /* Number of Longwords transfered */
#endif
#endif
	      FAUNLOCK;
	      return(xferCount); /* Return number of data words transfered */
	    }
	  else
	    {
#if defined(VXWORKS) && !defined(HALLB)
	      xferCount = (nwrds - (retVal>>2) + dummy);  /* Number of Longwords transfered */
	      logMsg("faReadBlock: DMA transfer terminated by unknown BUS Error (csr=0x%x xferCount=%d id=%d)\n",
		     csr,xferCount,id,0,0,0);
	      fadcBlockError=FA_BLOCKERROR_UNKNOWN_BUS_ERROR;
#else
	      xferCount = ((retVal>>2) + dummy);  /* Number of Longwords transfered */
	      if((retVal>>2)==nwrds)
		{
		  logMsg("faReadBlock: WARN: DMA transfer terminated by word count 0x%x\n",nwrds,0,0,0,0,0);
		  fadcBlockError=FA_BLOCKERROR_TERM_ON_WORDCOUNT;
		}
	      else
		{
		  logMsg("faReadBlock: DMA transfer terminated by unknown BUS Error (csr=0x%x xferCount=%d id=%d)\n",
			 csr,xferCount,id,0,0,0);
		  fadcBlockError=FA_BLOCKERROR_UNKNOWN_BUS_ERROR;
		}
#endif
	      FAUNLOCK;
	      if(rmode == 2)
		faGetTokenStatus(1);

	      return(xferCount);
	    }
	}
      else if (retVal == 0)
	{ /* Block Error finished without Bus Error */
#if defined(VXWORKS) && !defined(HALLB)
	  logMsg("faReadBlock: WARN: DMA transfer terminated by word count 0x%x\n",nwrds,0,0,0,0,0);
#else
	  logMsg("faReadBlock: WARN: DMA transfer returned zero word count 0x%x\n",nwrds,0,0,0,0,0);
#endif
	  fadcBlockError=FA_BLOCKERROR_ZERO_WORD_COUNT;
	  FAUNLOCK;

	  if(rmode == 2)
	    faGetTokenStatus(1);

	  return(nwrds);
	}
      else
	{  /* Error in DMA */
#if defined(VXWORKS) && !defined(HALLB)
	  logMsg("faReadBlock: ERROR: sysVmeDmaDone returned an Error\n",0,0,0,0,0,0);
#else
	  logMsg("faReadBlock: ERROR: vmeDmaDone returned an Error\n",0,0,0,0,0,0);
#endif
	  fadcBlockError=FA_BLOCKERROR_DMADONE_ERROR;
	  FAUNLOCK;

	  if(rmode == 2)
	    faGetTokenStatus(1);

	  return(retVal>>2);
	}

    }
  else
    {	/*Programmed IO */

  	/* Check if Bus Errors are enabled. If so then disable for Prog I/O reading */
      FALOCK;
      berr = vmeRead32(&(FAp[id]->ctrl1))&FA_ENABLE_BERR;
      if(berr)
	vmeWrite32(&(FAp[id]->ctrl1),vmeRead32(&(FAp[id]->ctrl1)) & ~FA_ENABLE_BERR);

      dCnt = 0;
      /* Read Block Header - should be first word */
      bhead = (unsigned int) *FApd[id];
#ifndef VXWORKS
      bhead = LSWAP(bhead);
#endif
      if((bhead&FA_DATA_TYPE_DEFINE)&&((bhead&FA_DATA_TYPE_MASK) == FA_DATA_BLOCK_HEADER))
	{
	  ehead = (unsigned int) *FApd[id];
#ifndef VXWORKS
	  ehead = LSWAP(ehead);
#endif

#ifdef VXWORKS
	  data[dCnt] = bhead;
#else
	  data[dCnt] = LSWAP(bhead); /* Swap back to little-endian */
#endif
	  dCnt++;
#ifdef VXWORKS
	  data[dCnt] = ehead;
#else
	  data[dCnt] = LSWAP(ehead); /* Swap back to little-endian */
#endif
	  dCnt++;
	}
      else
	{
	  /* We got bad data - Check if there is any data at all */
	  if( (vmeRead32(&(FAp[id]->ev_count)) & FA_EVENT_COUNT_MASK) == 0)
	    {
	      logMsg("faReadBlock: FIFO Empty (0x%08x)\n",bhead,0,0,0,0,0);
	      FAUNLOCK;
	      return(0);
	    }
	  else
	    {
	      logMsg("faReadBlock: ERROR: Invalid Header Word 0x%08x\n",bhead,0,0,0,0,0);
	      FAUNLOCK;
	      return(ERROR);
	    }
	}

      ii=0;
      while(ii<nwrds)
	{
	  val = (unsigned int) *FApd[id];
	  data[ii+2] = val;
#ifndef VXWORKS
	  val = LSWAP(val);
#endif
	  if( (val&FA_DATA_TYPE_DEFINE)
	      && ((val&FA_DATA_TYPE_MASK) == FA_DATA_BLOCK_TRAILER) )
	    break;
	  ii++;
	}
      ii++;
      dCnt += ii;

      if(berr)
	vmeWrite32(&(FAp[id]->ctrl1), vmeRead32(&(FAp[id]->ctrl1)) | FA_ENABLE_BERR);

      FAUNLOCK;
      return(dCnt);
    }

  FAUNLOCK;
  return(OK);

} //End faReadBlock

/**
 *  @ingroup Status
 *  @brief Return the type of error that occurred while attempting a
 *    block read from faReadBlock.
 *  @param pflag
 *     - >0: Print error message to standard out
 *  @sa faReadBlock
 *  @return OK if successful, otherwise ERROR.
 */
int
faGetBlockError(int pflag)
{
  int rval=0;
  const char *block_error_names[FA_BLOCKERROR_NTYPES] =
    {
      "NO ERROR",
      "DMA Terminated on Word Count",
      "Unknown Bus Error",
      "Zero Word Count",
      "DmaDone Error"
    };

  rval = fadcBlockError;
  if(pflag)
    {
      if(rval!=FA_BLOCKERROR_NO_ERROR)
	{
	  logMsg("faGetBlockError: Block Transfer Error: %s\n",
		 block_error_names[rval],2,3,4,5,6);
	}
    }

  return rval;
}

int
faReadBlockStatus(int id, volatile UINT32 *data, int nwrds, int rflag)
{

  int stat, retVal, xferCount, rmode;
  int dummy=0;
  unsigned int csr=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faReadBlockStatus: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(nwrds <= 0) nwrds= (FA_MAX_ADC_CHANNELS*FA_MAX_DATA_PER_CHANNEL) + 8;
  rmode = rflag&0x0f;

  /* Check for 8 byte boundary for address - insert dummy word (Slot 0 FADC Dummy DATA)*/
  if((unsigned long) (data)&0x7)
    {
      dummy = 1;
    }
  else
    {
      dummy = 0;
    }

#ifdef HALLB
  retVal = usrVme2MemDmaDone();
#else
#ifdef VXWORKS
  retVal = sysVmeDmaDone(10000,1);
#else
  retVal = vmeDmaDone();
#endif
#endif

  FALOCK;
  if(retVal > 0)
    {
      /* Check to see that Bus error was generated by FADC */
      if(rmode == 2)
	{
	  csr = vmeRead32(&(FAp[fadcMaxSlot]->csr));  /* from Last FADC */
	  stat = (csr)&FA_CSR_BERR_STATUS;  /* from Last FADC */
	}
      else
	{
	  stat = vmeRead32(&(FAp[id]->csr))&FA_CSR_BERR_STATUS;  /* from FADC id */
	}

      if((retVal>0) && (stat))
	{
#ifdef HALLB
	  xferCount = ((retVal>>2) + dummy);  /* Number of Longwords transfered */
#else
	  xferCount = (nwrds - (retVal>>2) + dummy);  /* Number of Longwords transfered */
#endif
	  FAUNLOCK;
	  return(xferCount); /* Return number of data words transfered */
	}
      else
	{
#ifdef HALLB
	  xferCount = ((retVal>>2) + dummy);  /* Number of Longwords transfered */
#else
	  xferCount = (nwrds - (retVal>>2) + dummy);  /* Number of Longwords transfered */
#endif
	  logMsg("faReadBlockStatus: DMA transfer terminated by unknown BUS Error (csr=0x%x nwrds=%d)\n",csr,xferCount,0,0,0,0);
	  FAUNLOCK;
	  return(ERROR);
	}
    }
  else if (retVal == 0)
    { /* Block Error finished without Bus Error */
      logMsg("faReadBlockStatus: WARN: DMA transfer terminated by word count 0x%x\n",nwrds,0,0,0,0,0);
      FAUNLOCK;
      return(nwrds);
    }
  else
    {  /* Error in DMA */
      logMsg("faReadBlockStatus: ERROR: sysVmeDmaDone returned an Error\n",0,0,0,0,0,0);
      FAUNLOCK;
      return(retVal);
    }

}

int
faPrintBlock(int id, int rflag)
{

  int ii;
  int nwrds=32768, dCnt, berr=0;
  unsigned int data, bhead, ehead;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("faPrintEvent: ERROR : FADC in slot %d is not initialized \n",id);
      return(ERROR);
    }

  /* Check if data available */
  FALOCK;
  if((vmeRead32(&(FAp[id]->ev_count))&FA_EVENT_COUNT_MASK)==0)
    {
      printf("faPrintEvent: ERROR: FIFO Empty\n");
      FAUNLOCK;
      return(0);
    }

  /* Check if Bus Errors are enabled. If so then disable for reading */
  berr = vmeRead32(&(FAp[id]->ctrl1))&FA_ENABLE_BERR;
  if(berr)
    vmeWrite32(&(FAp[id]->ctrl1),vmeRead32(&(FAp[id]->ctrl1)) & ~FA_ENABLE_BERR);

  dCnt = 0;
  /* Read Block Header - should be first word */
  bhead = (unsigned int) *FApd[id];
#ifndef VXWORKS
  bhead = LSWAP(bhead);
#endif
  if( (bhead&FA_DATA_TYPE_DEFINE)&&((bhead&FA_DATA_TYPE_MASK) == FA_DATA_BLOCK_HEADER))
    {
      ehead = (unsigned int) *FApd[id];
#ifndef VXWORKS
      ehead = LSWAP(ehead);
#endif
      printf("%4d: ",dCnt+1);
      faDataDecode(bhead);
      dCnt++;
      printf("%4d: ",dCnt+1);
      faDataDecode(ehead);
      dCnt++;
    }
  else
    {
      /* We got bad data - Check if there is any data at all */
      if((vmeRead32(&(FAp[id]->ev_count))&FA_EVENT_COUNT_MASK)==0)
	{
	  logMsg("faPrintBlock: FIFO Empty (0x%08x)\n",bhead,0,0,0,0,0);
	  FAUNLOCK;
	  return(0);
	}
      else
	{
	  logMsg("faPrintBlock: ERROR: Invalid Header Word 0x%08x\n",bhead,0,0,0,0,0);
	  FAUNLOCK;
	  return(ERROR);
	}
    }

  ii=0;
  while(ii<nwrds)
    {
      data = (unsigned int) *FApd[id];
#ifndef VXWORKS
      data = LSWAP(data);
#endif
      printf("%4d: ",dCnt+1+ii);
      faDataDecode(data);
      if((data&FA_DATA_TYPE_DEFINE)&&((data&FA_DATA_TYPE_MASK) == FA_DATA_BLOCK_TRAILER))
	break;

      if((data&FA_DATA_TYPE_DEFINE)&&((data&FA_DATA_TYPE_MASK) == FA_DATA_INVALID))
	break;

      ii++;
    }
  ii++;
  dCnt += ii;

  if(berr)
    vmeWrite32(&(FAp[id]->ctrl1),
	       vmeRead32( &(FAp[id]->ctrl1)) | FA_ENABLE_BERR );

  FAUNLOCK;
  return(dCnt);

}

/*****************************************************************************/

unsigned int
faReadCSR(int id)
{
  unsigned int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faReadCSR: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0);
    }

  FALOCK;
  rval = vmeRead32(&(FAp[id]->csr));
  FAUNLOCK;

  return(rval);
}


void
faClear(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faClear: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }
  FALOCK;
  vmeWrite32(&(FAp[id]->csr),FA_CSR_SOFT_RESET);
  FAUNLOCK;
}


void
faGClear()
{

  int ii, id;

  FALOCK;
  for(ii=0;ii<nfadc;ii++)
    {
      id = fadcID[ii];
      if((id<=0) || (id>21) || (FAp[id] == NULL))
	{
	  logMsg("faGClear: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
	}
      else
	{
	  vmeWrite32(&(FAp[id]->csr),FA_CSR_SOFT_RESET);
	}
    }
  FAUNLOCK;

}

void
faClearError(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faClearErr: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->csr),FA_CSR_ERROR_CLEAR);
  FAUNLOCK;

}


void
faGClearError()
{

  int ii, id;

  FALOCK;
  for(ii=0;ii<nfadc;ii++)
    {
      id = fadcID[ii];
      if((id<=0) || (id>21) || (FAp[id] == NULL))
	{
	  logMsg("faGClearErr: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
	}
      else
	{
	  vmeWrite32(&(FAp[id]->csr),FA_CSR_ERROR_CLEAR);
	}
    }
  FAUNLOCK;

}


void
faReset(int id, int iFlag)
{
  unsigned int a32addr, addrMB;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faReset: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  if(iFlag==0)
    {
      a32addr = vmeRead32(&(FAp[id]->adr32));
      addrMB  = vmeRead32(&(FAp[id]->adr_mb));
    }

  vmeWrite32(&(FAp[id]->csr),FA_CSR_HARD_RESET);
  taskDelay(2);

  if(iFlag==0)
    {
      vmeWrite32(&(FAp[id]->adr32),a32addr);
      vmeWrite32(&(FAp[id]->adr_mb),addrMB);
    }
  FAUNLOCK;

}

/**
 *  @ingroup Config
 *  @brief Perform a hard reset on all initialized fADC250s
 *  @param iFlag Decision to restore A32 readout after reset.
 *     -  0: Restore A32 readout after reset.
 *     - !0: Do not restore A32 readout after reset. (Useful for configuration changes)
 */
void
faGReset(int iFlag)
{
  unsigned int a32addr[(FA_MAX_BOARDS+1)], addrMB[(FA_MAX_BOARDS+1)];
  int ifa=0, id=0;

  FALOCK;
  if(iFlag==0)
    {
      for(ifa=0; ifa<nfadc; ifa++)
	{
	  id = faSlot(ifa);
	  a32addr[id] = vmeRead32(&(FAp[id]->adr32));
	  addrMB[id]  = vmeRead32(&(FAp[id]->adr_mb));
	}
    }

  for(ifa=0; ifa<nfadc; ifa++)
    {
      id = faSlot(ifa);
      vmeWrite32(&(FAp[id]->csr),FA_CSR_HARD_RESET);
    }

  taskDelay(10);

  if(iFlag==0)
    {
      for(ifa=0; ifa<nfadc; ifa++)
	{
	  id = faSlot(ifa);
	  vmeWrite32(&(FAp[id]->adr32),a32addr[id]);
	  vmeWrite32(&(FAp[id]->adr_mb),addrMB[id]);
	}
    }

  FAUNLOCK;

}

void
faSoftReset(int id, int cflag)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faReset: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  if(cflag) /* perform soft clear */
    vmeWrite32(&(FAp[id]->csr),FA_CSR_SOFT_CLEAR);
  else      /* normal soft reset */
    vmeWrite32(&(FAp[id]->csr),FA_CSR_SOFT_RESET);
  FAUNLOCK;

}

void
faResetToken(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faResetToken: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->reset),FA_RESET_TOKEN);
  FAUNLOCK;
}

int
faTokenStatus(int id)
{
  int rval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faResetToken: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return ERROR;
    }

  FALOCK;
  rval = (vmeRead32(&FAp[id]->csr) & FA_CSR_TOKEN_STATUS)>>4;
  FAUNLOCK;

  return rval;
}

int
faGTokenStatus()
{
  int ifa=0, bit=0, rval=0;

  for(ifa = 0; ifa<nfadc; ifa++)
    {
      bit = faTokenStatus(faSlot(ifa));
      rval |= (bit<<(faSlot(ifa)));
    }

  return rval;
}

unsigned int
faGetTokenStatus(int pflag)
{
  unsigned int rval = 0;
  int ifa = 0;

  if(pflag)
    logMsg("faGetTokenStatus: Token in slot(s) ",1,2,3,4,5,6);

  rval = faGTokenStatus();

  if(pflag)
    {
      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  if(rval & (1<<fadcID[ifa]))
	    logMsg("%2d ", fadcID[ifa], 2, 3, 4, 5, 6);
	}
    }

  if(pflag)
    logMsg("\n", 1, 2, 3, 4, 5, 6);

  return rval;
}

void
faSetCalib(int id, unsigned short sdelay, unsigned short tdelay)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetCalib: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->delay),(sdelay<<16) | tdelay);
  FAUNLOCK;

}

void
faChanDisable(int id, unsigned short cmask)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faChanDisable: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  fadcChanDisable[id] = cmask;  /* Set Global Variable */

  FALOCK;
  /* Write New Disable Mask */
  vmeWrite32(&(FAp[id]->adc_config[1]), cmask);
  FAUNLOCK;

}


unsigned int
faGetChanMask(int id)
{
  unsigned int tmp, cmask = 0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetChanMask: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0);
    }


  FALOCK;
  tmp = vmeRead32(&(FAp[id]->adc_config[1]))&0xFFFF;
  cmask = (tmp&FA_ADC_CHAN_MASK);
  fadcChanDisable[id] = cmask;  /* Set Global Variable */
  FAUNLOCK;


  return(cmask);
}


/* opt=0 - disable, 1-enable, 2-verify */
void
faSetCompression(int id, int opt)
{
  unsigned int ctrl2;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetCompression: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  printf("faSetCompression: id=%d opt=%d\n",id,opt);

  FALOCK;

  ctrl2  = (vmeRead32(&(FAp[id]->ctrl2)))&FA_CONTROL2_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faSetCompression: read ctrl2=0x%08x\n",ctrl2);
#endif /* DEBUG_COMPRESSION */

  ctrl2 = ctrl2 & ~FA_CTRL_COMPRESS_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faSetCompression: masked ctrl2=0x%08x\n",ctrl2);
#endif /* DEBUG_COMPRESSION */

  if(opt==0)
    {
      printf("faSetCompression: doing nothing\n");
      ;
    }
  else if(opt==1)
    {
      ctrl2 = ctrl2 | FA_CTRL_COMPRESS_ENABLE;
      printf("faSetCompression: setting mode 1 ctrl2=0x%08x\n",ctrl2);
    }
  else if(opt==2)
    {
      ctrl2 = ctrl2 | FA_CTRL_COMPRESS_VERIFY;
      printf("faSetCompression: setting mode 2 ctrl2=0x%08x\n",ctrl2);
    }
  else
    printf("faSetCompression: illegal opt=%d\n",opt);

#ifdef DEBUG_COMPRESSION
  printf("faSetCompression: writing ctrl2=0x%08x\n",ctrl2);
#endif /* DEBUG_COMPRESSION */
  vmeWrite32(&(FAp[id]->ctrl2), ctrl2);

  FAUNLOCK;
}


int
faGetCompression(int id)
{
  unsigned int ctrl2;
  int opt;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetCompression: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;

  ctrl2  = (vmeRead32(&(FAp[id]->ctrl2)))&FA_CONTROL2_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faGetCompression: read ctrl2=0x%08x\n",ctrl2);
#endif /* DEBUG_COMPRESSION */

  ctrl2 = ctrl2 & FA_CTRL_COMPRESS_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faGetCompression: masked ctrl2=0x%08x\n",ctrl2);
#endif /* DEBUG_COMPRESSION */

  if(ctrl2 == FA_CTRL_COMPRESS_DISABLE) opt = 0;
  else if(ctrl2 == FA_CTRL_COMPRESS_ENABLE) opt = 1;
  else if(ctrl2 == FA_CTRL_COMPRESS_VERIFY) opt = 2;
  else opt = -2;

  FAUNLOCK;

  return(opt);
}

/* opt=0 - disable, 1-enable */
void
faSetVXSReadout(int id, int opt)
{
  unsigned int ctrl2;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetVXSReadout: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;

  ctrl2  = vmeRead32(&FAp[id]->ctrl2);

  if(opt==0)
    {
      ctrl2 = ctrl2 & ~FA_CTRL_VXS_RO_ENABLE;
    }
  else
    {
      ctrl2 = ctrl2 | FA_CTRL_VXS_RO_ENABLE;
    }

  vmeWrite32(&(FAp[id]->ctrl2), ctrl2);

  FAUNLOCK;
}

void
faGSetVXSReadout(int opt)
{
  int ifa=0;

  for(ifa = 0; ifa<nfadc; ifa++)
    {
      faSetVXSReadout(fadcID[ifa], opt);
    }

}


int
faGetVXSReadout(int id)
{
  unsigned int ctrl2;
  int opt;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetVXSReadout: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;

  ctrl2  = vmeRead32(&FAp[id]->ctrl2) & FA_CTRL_VXS_RO_ENABLE;

  if(ctrl2)
    opt = 1;
  else
    opt = 0;

  FAUNLOCK;

  return(opt);
}


void
faEnableSyncReset(int id)
{
  unsigned int ctrl2;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnable: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  ctrl2 = vmeRead32(&FAp[id]->ctrl2) | FA_CTRL_ENABLE_SRESET;
  vmeWrite32(&FAp[id]->ctrl2, ctrl2);
  FAUNLOCK;
}

void
faEnable(int id, int eflag, int bank)
{
  unsigned int ctrl2;
  int compress_opt, vxsro_opt;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnable: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  /* call it BEFORE 'FALOCK' !!! */
  compress_opt = faGetCompression(id);
  vxsro_opt = faGetVXSReadout(id);

  FALOCK;

  ctrl2 = FA_CTRL_GO | FA_CTRL_ENABLE_TRIG | FA_CTRL_ENABLE_SRESET;

  if(eflag) /* Enable Internal Trigger logic as well*/
    {
      ctrl2 = ctrl2 | FA_CTRL_ENABLE_INT_TRIG;
    }

  if(compress_opt==1)
    {
      ctrl2 = ctrl2 | FA_CTRL_COMPRESS_ENABLE;
    }
  else if(compress_opt==2)
    {
      ctrl2 = ctrl2 | FA_CTRL_COMPRESS_VERIFY;
    }

  if(vxsro_opt==1)
    {
      ctrl2 = ctrl2 | FA_CTRL_VXS_RO_ENABLE;
    }

  vmeWrite32(&(FAp[id]->ctrl2), ctrl2);

  FAUNLOCK;
}

void
faGEnable(int eflag, int bank)
{
  int ii;

  for(ii=0;ii<nfadc;ii++)
    faEnable(fadcID[ii],eflag,bank);

  if(fadcUseSDC && !fadcSDCPassthrough)
    faSDC_Enable(1);

}

void
faDisable(int id, int eflag)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faDisable: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  if(eflag)
    vmeWrite32(&(FAp[id]->ctrl2),0);   /* Turn FIFO Transfer off as well */
  else
    vmeWrite32(&(FAp[id]->ctrl2),(FA_CTRL_GO|FA_CTRL_ENABLE_SRESET));  /* Keep SYNC RESET detection enabled */
  FAUNLOCK;
}

void
faGDisable(int eflag)
{
  int ii;

  if(fadcUseSDC && !fadcSDCPassthrough)
    faSDC_Disable();

  for(ii=0;ii<nfadc;ii++)
    faDisable(fadcID[ii],eflag);

}


void
faTrig(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faTrig: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  if( vmeRead32(&(FAp[id]->ctrl1)) & (FA_ENABLE_SOFT_TRIG) )
    vmeWrite32(&(FAp[id]->csr), FA_CSR_TRIGGER);
  else
    logMsg("faTrig: ERROR: Software Triggers not enabled",0,0,0,0,0,0);
  FAUNLOCK;
}

void
faGTrig()
{
  int ii;

  for(ii=0;ii<nfadc;ii++)
    faTrig(fadcID[ii]);
}

void
faTrig2(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faTrig2: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }
  FALOCK;
  if( vmeRead32(&(FAp[id]->ctrl1)) & (FA_ENABLE_SOFT_TRIG) )
    vmeWrite32(&(FAp[id]->csr), FA_CSR_SOFT_PULSE_TRIG2);
  else
    logMsg("faTrig2: ERROR: Software Triggers not enabled",0,0,0,0,0,0);
  FAUNLOCK;
}

void
faGTrig2()
{
  int ii;

  for(ii=0;ii<nfadc;ii++)
    faTrig2(fadcID[ii]);
}

int
faSetTrig21Delay(int id, int delay)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,id);
      return ERROR;
    }

  if(delay>FA_TRIG21_DELAY_MASK)
    {
      printf("%s: ERROR: Invalid value for delay (%d).\n",
	     __FUNCTION__,delay);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->trig21_delay, delay);
  FAUNLOCK;

  return OK;
}

int
faGetTrig21Delay(int id)
{
  int rval=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,id);
      return ERROR;
    }

  FALOCK;
  rval = vmeRead32(&FAp[id]->trig21_delay) & FA_TRIG21_DELAY_MASK;
  FAUNLOCK;

  return rval;
}


int
faEnableInternalPlaybackTrigger(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,id);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->ctrl1,
	     (vmeRead32(&FAp[id]->ctrl1) & ~FA_TRIG_MASK) | FA_TRIG_VME_PLAYBACK);
  FAUNLOCK;

  return OK;
}

void
faSync(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSync: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  if(vmeRead32(&(FAp[id]->ctrl1))&(FA_ENABLE_SOFT_SRESET))
    vmeWrite32(&(FAp[id]->csr), FA_CSR_SYNC);
  else
    logMsg("faSync: ERROR: Software Sync Resets not enabled\n",0,0,0,0,0,0);
  FAUNLOCK;
}



/* Return Event/Block count for ADC in slot id */
int
faDready(int id, int dflag)
{
  unsigned int dcnt=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faDready: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  if(dflag)
    dcnt = vmeRead32(&(FAp[id]->blk_count))&FA_BLOCK_COUNT_MASK;
  else
    dcnt = vmeRead32(&(FAp[id]->ev_count))&FA_EVENT_COUNT_MASK;
  FAUNLOCK;


  return(dcnt);
}

/* Return a Block Ready status for ADC. If Block Level is =1 then return Event Ready status */
int
faBready(int id)
{
  int stat=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faBready: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  stat = (vmeRead32(&(FAp[id]->csr))) &FA_CSR_BLOCK_READY;
  FAUNLOCK;

  if(stat)
    return(1);
  else
    return(0);
}

unsigned int
faGBready()
{
  int ii, id, stat=0;
  unsigned int dmask=0;

  FALOCK;
  for(ii=0;ii<nfadc;ii++)
    {
      id = fadcID[ii];

      stat = vmeRead32(&(FAp[id]->csr))&FA_CSR_BLOCK_READY;

      if(stat)
	dmask |= (1<<id);
    }
  FAUNLOCK;

  return(dmask);
}

unsigned int
faGBlockReady(unsigned int slotmask, int nloop)
{
  int iloop, islot, stat=0;
  unsigned int dmask=0;

  FALOCK;
  for(iloop = 0; iloop < nloop; iloop++)
    {

      for(islot = 0; islot < 21; islot++)
	{

	  if(slotmask & (1<<islot))
	    { /* slot used */

	      if(!(dmask & (1<<islot)))
		{ /* No block ready yet. */

		  stat = vmeRead32(&FAp[islot]->csr) & FA_CSR_BLOCK_READY;

		  if(stat)
		    dmask |= (1<<islot);

		  if(dmask == slotmask)
		    { /* Blockready mask matches user slotmask */
		      FAUNLOCK;
		      return(dmask);
		    }
		}
	    }
	}
    }
  FAUNLOCK;

  return(dmask);

}

/* return Scan mask for all initialized FADCs */
unsigned int
faScanMask()
{
  int ifadc, id, dmask=0;

  for(ifadc=0; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      dmask |= (1<<id);
    }

  return(dmask);
}


/* if val>0 then set the busy level, if val=0 then read it back.
   if bflag>0 then force the module Busy */
int
faBusyLevel(int id, unsigned int val, int bflag)
{
  unsigned int blreg=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faBusyLevel: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }
  if(val>FA_BUSY_LEVEL_MASK)
    return(ERROR);

  /* if Val > 0 then set the Level else leave it alone*/
  FALOCK;
  if(val)
    {
      if(bflag)
	vmeWrite32(&(FAp[id]->busy_level),(val | FA_FORCE_BUSY));
      else
	vmeWrite32(&(FAp[id]->busy_level),val);
    }
  else
    {
      blreg = vmeRead32(&(FAp[id]->busy_level));
      if(bflag)
	vmeWrite32(&(FAp[id]->busy_level),(blreg | FA_FORCE_BUSY));
    }
  FAUNLOCK;

  return((blreg&FA_BUSY_LEVEL_MASK));
}

int
faBusy(int id)
{
  unsigned int blreg=0;
  unsigned int dreg=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faBusy: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  blreg = vmeRead32(&(FAp[id]->busy_level))&FA_BUSY_LEVEL_MASK;
  dreg  = vmeRead32(&(FAp[id]->ram_word_count))&FA_RAM_DATA_MASK;
  FAUNLOCK;

  if(dreg>=blreg)
    return(1);
  else
    return(0);
}


void
faEnableSoftTrig(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnableSoftTrig: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  /* Clear the source */
  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_TRIG_MASK );
  /* Set Source and Enable*/
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | (FA_TRIG_VME | FA_ENABLE_SOFT_TRIG) );
  FAUNLOCK;
}


void
faGEnableSoftTrig()
{
  int ii, id;

  for(ii=0;ii<nfadc;ii++)
    {
      id = fadcID[ii];
      faEnableSoftTrig(id);
    }

}


void
faDisableSoftTrig(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faDisableSoftTrig: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_ENABLE_SOFT_TRIG );
  FAUNLOCK;

}

void
faEnableSoftSync(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnableSoftSync: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  /* Clear the source */
  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_SRESET_MASK);
  /* Set Source and Enable*/
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | (FA_SRESET_VME | FA_ENABLE_SOFT_SRESET));
  FAUNLOCK;
}

void
faDisableSoftSync(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faDisableSoftSync: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_ENABLE_SOFT_SRESET);
  FAUNLOCK;

}

void
faEnableClk(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnableClk: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | (FA_REF_CLK_INTERNAL|FA_ENABLE_INTERNAL_CLK) );
  FAUNLOCK;

}

void
faDisableClk(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faDisableClk: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_ENABLE_INTERNAL_CLK );
  FAUNLOCK;

}

/*************************************************************************************
 *
 *  faEnableTriggerOut - Enable trigger out for front panel or p0
 *
 *   output = 0 for FP trigger out
 *            1 for P0 trigger out
 *            2 for FP and P0 trigger out
 */

void
faEnableTriggerOut(int id, int output)
{
  int bitset=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnableBusError: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  if(output>2)
    {
      logMsg("faEnableTriggerOut: ERROR: output (%d) out of range.  Must be less than 3",
	     output,2,3,4,5,6);
      return;

    }

  switch(output)
    {
    case 0:
      bitset = FA_ENABLE_TRIG_OUT_FP;
      break;
    case 1:
      bitset = FA_ENABLE_TRIG_OUT_P0;
      break;
    case 2:
      bitset = FA_ENABLE_TRIG_OUT_FP | FA_ENABLE_TRIG_OUT_P0;
      break;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | bitset );
  FAUNLOCK;

}

void
faEnableBusError(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnableBusError: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | FA_ENABLE_BERR );
  FAUNLOCK;

}


void
faGEnableBusError()
{
  int ii;

  FALOCK;
  for(ii=0;ii<nfadc;ii++)
    {
      vmeWrite32(&(FAp[fadcID[ii]]->ctrl1),
		 vmeRead32(&(FAp[fadcID[ii]]->ctrl1)) | FA_ENABLE_BERR );
    }
  FAUNLOCK;

}


void
faDisableBusError(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faDisableBusError: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_ENABLE_BERR );
  FAUNLOCK;

}


void
faEnableMultiBlock(int tflag)
{
  int ii, id;
  unsigned int mode;

  if((nfadc <= 1) || (FAp[fadcID[0]] == NULL))
    {
      logMsg("faEnableMultiBlock: ERROR : Cannot Enable MultiBlock mode \n",0,0,0,0,0,0);
      return;
    }

  /* if token = 0 then send via P2 else via VXS */
  if(tflag)
    mode = (FA_ENABLE_MULTIBLOCK | FA_MB_TOKEN_VIA_P0);
  else
    mode = (FA_ENABLE_MULTIBLOCK | FA_MB_TOKEN_VIA_P2);

  for(ii=0;ii<nfadc;ii++)
    {
      id = fadcID[ii];
      FALOCK;
      vmeWrite32(&(FAp[id]->ctrl1),
		 vmeRead32(&(FAp[id]->ctrl1)) | mode );
      FAUNLOCK;
      faDisableBusError(id);
      if(id == fadcMinSlot)
	{
	  FALOCK;
	  vmeWrite32(&(FAp[id]->ctrl1),
		     vmeRead32(&(FAp[id]->ctrl1)) | FA_FIRST_BOARD );
	  FAUNLOCK;
	}
      if(id == fadcMaxSlot)
	{
	  FALOCK;
	  vmeWrite32(&(FAp[id]->ctrl1),
		     vmeRead32(&(FAp[id]->ctrl1)) | FA_LAST_BOARD );
	  FAUNLOCK;
	  faEnableBusError(id);   /* Enable Bus Error only on Last Board */
	}
    }

}

void
faDisableMultiBlock()
{
  int ii;

  if((nfadc <= 1) || (FAp[fadcID[0]] == NULL))
    {
      logMsg("faDisableMultiBlock: ERROR : Cannot Disable MultiBlock Mode\n",0,0,0,0,0,0);
      return;
    }

  FALOCK;
  for(ii=0;ii<nfadc;ii++)
    vmeWrite32(&(FAp[fadcID[ii]]->ctrl1),
	       vmeRead32(&(FAp[fadcID[ii]]->ctrl1)) & ~FA_ENABLE_MULTIBLOCK );
  FAUNLOCK;

}



int
faSetBlockLevel(int id, int level)
{
  int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetBlockLevel: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(level<=0) level = 1;

  logMsg("faSetBlockLevel: INFO: Set ADC slot %d block level to %d \n",id,level,0,0,0,0);

  FALOCK;
  vmeWrite32(&(FAp[id]->blk_level), level);
  fadcBlockLevel = level;
  rval = vmeRead32(&(FAp[id]->blk_level)) & FA_BLOCK_LEVEL_MASK;
  FAUNLOCK;

  return(rval);

}

void
faGSetBlockLevel(int level)
{
  int ii;

  if(level<=0) level = 1;
  FALOCK;
  for(ii=0;ii<nfadc;ii++)
    vmeWrite32(&(FAp[fadcID[ii]]->blk_level), level);
  FAUNLOCK;

  fadcBlockLevel = level;
}


int
faSetClkSource(int id, int source)
{
  int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetClkSource: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_REF_CLK_SEL_MASK );
  if((source<0)||(source>7)) source = FA_REF_CLK_INTERNAL;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | source );
  rval = vmeRead32(&(FAp[id]->ctrl1)) & FA_REF_CLK_SEL_MASK;
  FAUNLOCK;


  return(rval);

}

int
faSetTrigSource(int id, int source)
{
  int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetTrigSource: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_TRIG_SEL_MASK );
  if((source<0)||(source>7)) source = FA_TRIG_FP_ISYNC;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | source );
  rval = vmeRead32(&(FAp[id]->ctrl1)) & FA_TRIG_SEL_MASK;
  FAUNLOCK;

  return(rval);

}

int
faSetSyncSource(int id, int source)
{
  int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetSyncSource: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) & ~FA_SRESET_SEL_MASK );
  if((source<0)||(source>7)) source = FA_SRESET_FP_ISYNC;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | source );
  rval = vmeRead32(&(FAp[id]->ctrl1)) & FA_SRESET_SEL_MASK;
  FAUNLOCK;

  return(rval);

}

/* Enable Front Panel Inputs (and Disable software triggers/syncs
   but leave the clock source alone */
void
faEnableFP(int id)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnableFP: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) &
	     ~(FA_TRIG_SEL_MASK | FA_SRESET_SEL_MASK | FA_ENABLE_SOFT_SRESET | FA_ENABLE_SOFT_TRIG));
  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&(FAp[id]->ctrl1)) | (FA_TRIG_FP_ISYNC | FA_SRESET_FP_ISYNC));
  FAUNLOCK;

}

/* Set trigger output options
 *   trigout bits:
 *      0  0  1  Live Internal Trigger to Output
 *      0  1  0  Enable Front Panel Trigger Output
 *      1  0  0  Enable VXS Trigger Output
 *
 * RETURNS: OK, or ERROR if unsuccessful
 */

int
faSetTrigOut(int id, int trigout)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("faSetTrigOut: ERROR : ADC in slot %d is not initialized \n",id);
      return ERROR;
    }

  if(trigout<0 || trigout > 7)
    {
      printf("faSetTrigOut: ERROR : Invalid trigout value (%d) \n",trigout);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->ctrl1),
	     (vmeRead32(&(FAp[id]->ctrl1)) & ~FA_TRIGOUT_MASK) |
	     trigout<<12);
  FAUNLOCK;

  return OK;
}

/* Routine to get the Trigger Counter value */
unsigned int
faGetTriggerCount(int id)
{
  unsigned int rval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetTrigCount: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return 0xffffffff;
    }

  /* Just reading - not need to Lock mutex */
  rval = vmeRead32(&FAp[id]->trig_scal);

  return rval;

}

int
faResetTriggerCount(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faResetTriggerCount: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  vmeWrite32(&FAp[id]->trig_scal,FA_TRIG_SCAL_RESET);
  FAUNLOCK;

  return OK;
}

int
faSetThreshold(int id, unsigned short tvalue, unsigned short chmask)
{

  int ii, doWrite=0;
  unsigned int lovalue=0, hivalue=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetThreshold: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chmask==0) chmask = 0xffff;  /* Set All channels the same */

  /*printf("faSetThreshold: slot %d, value %d, mask 0x%04X\n", id, tvalue, chmask);*/

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      if(ii%2==0)
	{
	  lovalue = (vmeRead16(&FAp[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAp[id]->adc_thres[ii+1]));

	  if((1<<ii)&chmask)
	    {
	      lovalue = (lovalue & ~FA_THR_VALUE_MASK) | tvalue;
	      doWrite=1;
	    }
	  if((1<<(ii+1))&chmask)
	    {
	      hivalue = (hivalue & ~FA_THR_VALUE_MASK) | tvalue;
	      doWrite=1;
	    }

	  if(doWrite)
	    vmeWrite32((unsigned int *)&(FAp[id]->adc_thres[ii]),
		       lovalue<<16 | hivalue);

	  lovalue = 0;
	  hivalue = 0;
	  doWrite=0;
	}
    }
  FAUNLOCK;

  return(OK);
}


int
faPrintThreshold(int id)
{
  int ii;
  unsigned short tval[FA_MAX_ADC_CHANNELS];

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faPrintThreshold: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      tval[ii] = vmeRead16(&(FAp[id]->adc_thres[ii]));
    }
  FAUNLOCK;

  printf(" Threshold Settings for FADC in slot %d:",id);
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      if((ii%4)==0)
	{
	  printf("\n");
	}
      printf("Chan %2d: %5d(%d)   ",(ii+1),tval[ii] & FA_THR_VALUE_MASK, (tval[ii] & FA_THR_IGNORE_MASK)>>15);
    }
  printf("\n");

  return(OK);
}


int
faSetDAC(int id, unsigned short dvalue, unsigned short chmask)
{
  int ii, doWrite=0;
  unsigned int lovalue=0, hivalue=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetDAC: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chmask==0) chmask = 0xffff;  /* Set All channels the same */

  if(dvalue>0xfff)
    {
      logMsg("faSetDAC: ERROR : DAC value (%d) out of range (0-255) \n",
	     dvalue,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {

      if(ii%2==0)
	{
	  lovalue = (vmeRead16(&FAp[id]->dac[ii]));
	  hivalue = (vmeRead16(&FAp[id]->dac[ii+1]));

	  if((1<<ii)&chmask)
	    {
	      lovalue = dvalue&FA_DAC_VALUE_MASK;
	      doWrite=1;
	    }
	  if((1<<(ii+1))&chmask)
	    {
	      hivalue = (dvalue&FA_DAC_VALUE_MASK);
	      doWrite=1;
	    }

	  if(doWrite)
	    vmeWrite32((unsigned int *)&(FAp[id]->dac[ii]),
		       lovalue<<16 | hivalue);

	  lovalue = 0;
	  hivalue = 0;
	  doWrite=0;
	}

    }
  FAUNLOCK;

  return(OK);
}


void
faPrintDAC(int id)
{
  int ii;
  unsigned short dval[FA_MAX_ADC_CHANNELS];

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faPrintDAC: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    dval[ii] = vmeRead16(&(FAp[id]->dac[ii])) & FA_DAC_VALUE_MASK;
  FAUNLOCK;


  printf(" DAC Settings for FADC in slot %d:",id);
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      if((ii%4)==0) printf("\n");
      printf("Chan %2d: %5d   ",(ii+1),dval[ii]);
    }
  printf("\n");

}

unsigned int
faGetChannelDAC(int id, unsigned int chan)
{
  unsigned int val;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetChannelDAC: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0);
    }

  FALOCK;
  val = vmeRead16(&(FAp[id]->dac[chan])) & FA_DAC_VALUE_MASK;
  FAUNLOCK;


  return(val);
}

int
faSetChannelPedestal(int id, unsigned int chan, unsigned int ped)
{
  unsigned int lovalue=0, hivalue=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetChannelPedestal: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faSetChannelPedestal: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  if(ped>0xffff)
    {
      logMsg("faSetChannelPedestal: ERROR : PED value (%d) out of range (0-65535) \n",
	     ped,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  lovalue = vmeRead16(&FAp[id]->adc_pedestal[(chan&0xE)+0]);
  hivalue = vmeRead16(&FAp[id]->adc_pedestal[(chan&0xE)+1]);

  if(chan & 0x1) hivalue = ped;
  else           lovalue = ped;

  vmeWrite32((unsigned int *)&(FAp[id]->adc_pedestal[chan&0xE]), (lovalue<<16) | hivalue);
  FAUNLOCK;

  return(OK);
}

int
faGetChannelPedestal(int id, unsigned int chan)
{
  unsigned int rval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetChannelPedestal: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faSetChannelPedestal: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  rval = vmeRead16(&FAp[id]->adc_pedestal[chan]) & FA_ADC_PEDESTAL_MASK;
  FAUNLOCK;

  return(rval);
}



#ifdef CLAS12

int
faSetChannelDelay(int id, unsigned int chan, unsigned int delay)
{
  unsigned int lovalue=0, hivalue=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetChannelDelay: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faSetChannelDelay: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  if(delay>512)
    {
      logMsg("faSetChannelDelay: ERROR : Delay value (%d) out of range (0-31) \n",
	     delay,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  lovalue = vmeRead16(&FAp[id]->adc_delay[(chan&0xE)+0]);
  hivalue = vmeRead16(&FAp[id]->adc_delay[(chan&0xE)+1]);

  if(chan & 0x1) hivalue = delay;
  else           lovalue = delay;

  vmeWrite32((unsigned int *)&(FAp[id]->adc_delay[chan&0xE]), (lovalue<<16) | hivalue);
  FAUNLOCK;

  return(OK);
}

/*Begin Andrea*/
int faSetDelayAll(int id,unsigned int delay){
  int ch=0;
  int ret;
  for (ch=0;ch<16;ch++){
    ret=faSetChannelDelay(id,ch,delay);
    if (ret!=OK){
      logMsg("faSetDelayAll: ERROR for slot %d ch %d\n",id,ch,3,4,5,6);
      break;
    }
  }
  return ret;
}

int faSetGlobalDelay(unsigned int delay)
{
  int fadc;
  int id;
  int ret;
  for (fadc=0;fadc<nfadc;fadc++)
    {
      id=fadcID[fadc];
      ret = faSetDelayAll(id,delay);
      if (ret!=OK)
	{
	  logMsg("faSetGlobalDelay: ERROR for slot %d \n",fadc,2,3,4,5,6);
	  break;
	}
    }
  return ret;
}
/*End Andrea*/
int
faGetChannelDelay(int id, unsigned int chan)
{
  unsigned int rval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetChannelDelay: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faSetChannelDelay: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  rval = vmeRead16(&FAp[id]->adc_delay[chan]) & FA_ADC_DELAY_MASK;
  FAUNLOCK;

  return(rval);
}



int
faInvert(int id, unsigned short chmask)
{
  int ii;
  unsigned int lovalue=0, hivalue=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faInvert: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      if(ii%2 == 0)
	{
	  lovalue = (vmeRead16(&FAp[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAp[id]->adc_thres[ii+1]));

	  if((1<<ii)&chmask)     lovalue |= FA_THR_INVERT_MASK;
	  else                   lovalue &=~FA_THR_INVERT_MASK;

	  if((1<<(ii+1))&chmask) hivalue |= FA_THR_INVERT_MASK;
	  else                   hivalue &=~FA_THR_INVERT_MASK;

	  vmeWrite32((unsigned int *)&(FAp[id]->adc_thres[ii]),
		     lovalue<<16 | hivalue);
	}
    }
  FAUNLOCK;
  return(OK);
}

unsigned int
faGetInvertMask(int id)
{
  int ii;
  unsigned int tmp, cmask = 0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetInvertMask: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0);
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      tmp = vmeRead16(&FAp[id]->adc_thres[ii]);
      if(tmp & FA_THR_INVERT_MASK)
	cmask |= (1<<ii);
    }
  FAUNLOCK;

  return(cmask);
}

int
faSetHitbitTrigMask(int id, unsigned short chmask)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetHitbitTrigMask: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->hitbit_trig_mask), chmask);
  FAUNLOCK;
  return(OK);
}

unsigned int
faGetHitbitTrigMask(int id)
{
  unsigned int rvalue = 0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetHitbitTrigMask: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0);
    }

  FALOCK;
  rvalue = vmeRead32(&(FAp[id]->hitbit_trig_mask)) & 0xFFFF;
  FAUNLOCK;
  return(rvalue);
}

int
faGSetHitbitMinTOT(unsigned short width)
{
  int ii;

  for(ii=0;ii<nfadc;ii++)
    faSetHitbitMinTOT(fadcID[ii], width);

  return(OK);
}


int
faGSetHitbitMinMultiplicity(unsigned short mult)
{
  int ii;

  for(ii=0;ii<nfadc;ii++)
    faSetHitbitMinMultiplicity(fadcID[ii], mult);

  return OK;
}

int
faGetHitbitMinTOT(int id)
{
  unsigned int val;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  val = vmeRead32(&(FAp[id]->hitbit_cfg));
  FAUNLOCK;
  return (val & 0xFF);
}

int
faSetHitbitMinTOT(int id, unsigned short width)
{
  unsigned int val;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  val = vmeRead32(&(FAp[id]->hitbit_cfg));
  val = (val & 0xFFFFFF00) | (width & 0xFF);
  vmeWrite32(&(FAp[id]->hitbit_cfg), val);
  FAUNLOCK;
  return(OK);
}

int
faGetHitbitMinMultiplicity(int id)
{
  unsigned int val;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  val = vmeRead32(&(FAp[id]->hitbit_cfg));
  FAUNLOCK;
  return(val>>8)&0x1F;
}

int
faSetHitbitMinMultiplicity(int id, unsigned short mult)
{
  unsigned int val;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  val = vmeRead32(&(FAp[id]->hitbit_cfg));
  val = (val & 0xFFFFE0FF) | ((mult & 0x1F)<<8);
  vmeWrite32(&(FAp[id]->hitbit_cfg), val);
  FAUNLOCK;
  return(OK);
}

int
faSetHitbitTrigWidth(int id, unsigned short width)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->hitbit_trig_width), width);
  FAUNLOCK;
  return(OK);
}

unsigned int
faGetHitbitTrigWidth(int id)
{
  unsigned int rvalue = 0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0);
    }

  FALOCK;
  rvalue = vmeRead32(&(FAp[id]->hitbit_trig_width)) & 0xFFFF;
  FAUNLOCK;
  return(rvalue);
}

int
faThresholdIgnore(int id, unsigned short chmask)
{
  int ii;
  unsigned int lovalue=0, hivalue=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faThresholdIgnore: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      if(ii%2 == 0)
	{
	  lovalue = (vmeRead16(&FAp[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAp[id]->adc_thres[ii+1]));

	  if((1<<ii)&chmask)     lovalue |= FA_THR_IGNORE_MASK;
	  else                   lovalue &=~FA_THR_IGNORE_MASK;

	  if((1<<(ii+1))&chmask) hivalue |= FA_THR_IGNORE_MASK;
	  else                   hivalue &=~FA_THR_IGNORE_MASK;

	  vmeWrite32((unsigned int *)&(FAp[id]->adc_thres[ii]),
		     lovalue<<16 | hivalue);
	}
    }
  FAUNLOCK;
  return(OK);
}

unsigned int
faGetThresholdIgnoreMask(int id)
{
  int ii;
  unsigned int tmp, cmask = 0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetThresholdIgnoreMask: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0);
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      tmp = vmeRead16(&FAp[id]->adc_thres[ii]);
      if(tmp & FA_THR_IGNORE_MASK)
	cmask |= (1<<ii);
    }
  FAUNLOCK;

  return(cmask);
}

int
faPlaybackDisable(int id, unsigned short chmask)
{
  int ii;
  unsigned int lovalue=0, hivalue=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faPlaybackDisable: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(-1);
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      if(ii%2 == 0)
	{
	  lovalue = (vmeRead16(&FAp[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAp[id]->adc_thres[ii+1]));

	  if((1<<ii)&chmask)     lovalue |= FA_PLAYBACK_DIS_MASK;
	  else                   lovalue &=~FA_PLAYBACK_DIS_MASK;

	  if((1<<(ii+1))&chmask) hivalue |= FA_PLAYBACK_DIS_MASK;
	  else                   hivalue &=~FA_PLAYBACK_DIS_MASK;

	  vmeWrite32((unsigned int *)&(FAp[id]->adc_thres[ii]),
		     lovalue<<16 | hivalue);
	}
    }
  FAUNLOCK;
  return(OK);
}

unsigned int
faGetPlaybackDisableMask(int id)
{
  int ii;
  unsigned int tmp, cmask = 0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetPlaybackDisableMask: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0);
    }

  FALOCK;
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      tmp = vmeRead16(&FAp[id]->adc_thres[ii]);
      if(tmp & FA_PLAYBACK_DIS_MASK)
	cmask |= (1<<ii);
    }
  FAUNLOCK;

  return(cmask);
}


int
faGetChThreshold(int id, int ch)
{
  unsigned int rvalue=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetThresholdAll: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  rvalue = vmeRead16(&(FAp[id]->adc_thres[ch])) & FA_THR_VALUE_MASK;
  FAUNLOCK;

  return rvalue;
}

int
faSetChThreshold(int id, int ch, int threshold)
{
  /*  printf("faSetChThreshold: slot %d, ch %d, threshold=%d\n",id,ch,threshold);*/

  return faSetThreshold(id, threshold, (1<<ch));
}

int
faGLoadChannelPedestals(char *fname, int updateThresholds)
{
  FILE *fd = NULL;
  float ped, adc_ped[FA_MAX_BOARDS+1][FA_MAX_ADC_CHANNELS];
  int offset, slot, chan, ii, nsamples, threshold, status;
  float sigma;

  fd = fopen(fname,"r");
  if(fd==NULL)
    {
      printf("faGLoadChannelPedestals: ERROR: cannot open pedestal file >%s<\n",fname);
      return(ERROR);
    }
  else
    { /* pedestal file opened */
      printf("faGLoadChannelPedestals: pedestal file >%s< is opened for reading\n",fname);

      memset(adc_ped, 0, sizeof(adc_ped));

      while( (status=fscanf(fd,"%d %d %f %f %d\n",&slot,&chan,&ped,&sigma,&offset)) > 0 )
	{
	  /*      printf("status=%d -> slot=%2d chan=%2d ped=%7.3f sigma=%6.3f\n",status,slot,chan,ped,sigma); */
	  if(slot>=2&&slot<21&&chan>=0&&chan<16)
	    {
	      adc_ped[slot][chan] = ped;
	      /*        printf("PED=%f\n",adc_ped[slot][chan]); */
	    }
	  else
	    {
	      printf("bad slot=%d or chan=%d\n",slot,chan);
	    }
	}

      if(status==EOF) { /*printf("EOF reached\n");*/ }
      else            printf("fscanf() returned error %d\n",status);

      fclose(fd);
      printf("faGLoadChannelPedestals: pedestal file >%s< is closed\n",fname);
    }

  for(ii=0; ii<nfadc; ii++)
    {
      slot = fadcID[ii];
      nsamples = faGetNSA(slot) + faGetNSB(slot);
      /*    printf("faGLoadChannelPedestals: slot=%d, nsamples=%d\n",slot,nsamples); */
      for(chan = 0; chan < FA_MAX_ADC_CHANNELS; chan++)
	{
	  ped = adc_ped[slot][chan] * ((float)nsamples);
	  /*      printf("faGLoadChannelPedestals: chan=%d, ped=%d\n",chan,(int)ped); */
	  faSetChannelPedestal(slot, chan, (int)ped);

	  if(updateThresholds)
	    {
	      threshold = faGetChThreshold(slot, chan);
	      /* if threshold=0, don't add pedestal since user is disabling zero suppression */
	      if(threshold) threshold += (int)adc_ped[slot][chan];
	      /*        printf("faGLoadChannelPedestals: chan=%d, threshold=%d\n",chan,threshold); */
	      faSetChThreshold(slot, chan, threshold);
	    }
	}
    }

  return OK;
}

#define FA_MEASURE_PED_NTIMES		10

int
faMeasureChannelPedestal(int id, unsigned int chan, fa250Ped *ped)
{
  int status, i, n;
  unsigned int sample0, sample1;
  double adc_val, nsamples;
  fa250Ped p;

  p.avg = 0.0;
  p.rms = 0.0;
  p.min = 4095.0;
  p.max = 0.0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faMeasureChannelPedestal: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faMeasureChannelPedestal: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  for(n = 0; n < FA_MEASURE_PED_NTIMES; n++)
    {
      FALOCK;
      vmeWrite16(&FAp[id]->la_ctrl, 0);       /* disable logic analyzer */
      for(i=0;i<16;i++)
	{
	  vmeWrite16(&FAp[id]->la_cmp_mode0[i], 0);	/* setup a don't care trigger */
	  vmeWrite16(&FAp[id]->la_cmp_thr0[i], 0);	/* setup a don't care trigger */
	}
      vmeWrite16(&FAp[id]->la_ctrl, 1); /* enable logic analyzer */
      FAUNLOCK;

      taskDelay(1);

      FALOCK;
      status = vmeRead16(&FAp[id]->la_status);
      vmeWrite16(&FAp[id]->la_ctrl, 0);       /* disable logic analyzer */
      FAUNLOCK;

      if(!status)
	{
	  logMsg("faMeasureChannelPedestal: ERROR : timeout \n");
	  return(ERROR);
	}

      FALOCK;
      for(i = 0; i < 512; i++)
	{
	  unsigned int idx   = (chan*13)/16;
	  unsigned int shift = (chan*13)%16;
	  sample0 = (unsigned int)vmeRead16(&FAp[id]->la_data[idx]);
	  if(idx<12) sample1 = (unsigned int)vmeRead16(&FAp[id]->la_data[idx+1]);

	  adc_val = (double)(((sample0>>shift) | (sample1<<(16-shift))) & 0xFFF);

	  p.avg+= adc_val;

	  p.rms+= adc_val*adc_val;

	  if(adc_val < p.min)
	    p.min = adc_val;

	  if(adc_val > p.max)
	    p.max = adc_val;
	}
      FAUNLOCK;
    }

  nsamples = 512.0 * (double)FA_MEASURE_PED_NTIMES;

  p.avg /= nsamples;
  p.rms = sqrt(p.rms / nsamples - p.avg*p.avg);

  printf("faMeasureChannelPedestal: slot %d, chan %d => avg %6.3f, rms %6.3f, min %.0f, max %.0f\n",
	 id, chan, p.avg, p.rms, p.min, p.max);

  if(ped)
    *ped = p;

  return(OK);
}

// Mode=0: normal pulse integration trigger mode
// Mode=1: discriminator trigger mode
int
faSetTriggerProcessingMode(int id, unsigned int chan, unsigned int mode)
{
  unsigned int rval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetTriggerProcessingMode: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faSetTriggerProcessingMode: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  rval = vmeRead32(&FAp[id]->adc_gain[chan]);
  if(mode)
    rval |= 0x8000;
  else
    rval &= 0x7FFF;
  vmeWrite32(&FAp[id]->adc_gain[chan], rval);
  FAUNLOCK;

  return(OK);
}

int
faGetTriggerProcessingMode(int id, unsigned int chan)
{
  unsigned int rval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetTriggerProcessingMode: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faGetTriggerProcessingMode: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  rval = vmeRead32(&FAp[id]->adc_gain[chan]);
  if(rval & 0x8000)
    rval = 1;
  else
    rval = 0;
  FAUNLOCK;

  return(rval);
}

int
faSetChannelGain(int id, unsigned int chan, float gain)
{
  unsigned int rval=0;
  int igain;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetChannelGain: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faSetChannelGain: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  if(gain>=127.0 || gain<0.0)
    {
      logMsg("faSetChannelGain: ERROR : GAIN value (%f) out of range (0.0-127.0) \n",
	     gain,0,0,0,0,0);
      return(ERROR);
    }

  igain = (int)(gain*256.0);
  /*  printf("gain=%f igain=%d\n",gain,igain);*/

  FALOCK;
  rval = vmeRead32(&FAp[id]->adc_gain[chan]) & 0x8000;
  rval|= igain & 0x7FFF;
  vmeWrite32(&FAp[id]->adc_gain[chan], igain);
  FAUNLOCK;

  return(OK);
}

float
faGetChannelGain(int id, unsigned int chan)
{
  unsigned int rval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetChannelGain: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(chan>16)
    {
      logMsg("faSetChannelGain: ERROR : Channel (%d) out of range (0-15) \n",
	     chan,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  rval = vmeRead32(&FAp[id]->adc_gain[chan]) & 0x7FFF;
  FAUNLOCK;

  return( ((float)rval)/256.0 );
}

int
faResetMGT(int id, int reset)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return(ERROR);
    }

  //  faSetMGTSettings(id, 0, 2, 2);

  FALOCK;
  if(reset)
    vmeWrite32(&(FAp[id]->gtx_ctrl),0x203); /*put reset*/
  else
    vmeWrite32(&(FAp[id]->gtx_ctrl),0x800); /*release reset*/
  FAUNLOCK;

  taskDelay(2);

  return(OK);
}

int
faGetMGTChannelStatus(int id)
{
  int status = 0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return(0);
    }

  FALOCK;
  status = vmeRead32(&(FAp[id]->gtx_status));
  //status = vmeRead32(&(FAp[id]->trx_ctrl));
  FAUNLOCK;

  if(status & 0x1)
    //  if(status & 0x1000)
    return(1);

  return(0);
}

/*
  int
  faSetMGTSettings(int id, int txpre, int txswing, int rxequ)
  {
  int val;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
  {
  printf("%s: ERROR : ADC in slot %d is not initialized \n",
  __FUNCTION__,id);
  return(ERROR);
  }

  val = ((txpre & 0x1F)<<0) |
  ((txswing & 0xF)<<10) |
  ((rxequ & 0x3)<<14);

  FALOCK;
  vmeWrite32(&(FAp[id]->trx_ctrl),val);
  FAUNLOCK;

  taskDelay(2);

  return(OK);
  }
*/


#endif



/**************************************************************************************
 *
 *  faSetMGTTestMode -
 *  faSyncResetMode  -  Set the fa250 operation when Sync Reset is received
 *
 *        When SYNC RESET is received by the module, the module may:
 *      mode : 0  - Send a calibration sequence to the CTP for alignment purposes
 *             1  - Normal operation
 *
 *        In both instances, timestamps and counters will be reset.
 *
 *   RETURNS OK, or ERROR if unsuccessful.
 */

int
faSetMGTTestMode(int id, unsigned int mode)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return(ERROR);
    }

  FALOCK;
  if(mode)
    { /* After Sync Reset (Normal mode) */
      vmeWrite32(&FAp[id]->mgt_ctrl, FA_MGT_RESET);
      vmeWrite32(&FAp[id]->mgt_ctrl, FA_MGT_FRONT_END_TO_CTP);
    }
  else
    { /* Before Sync Reset (Calibration Mode) */
      vmeWrite32(&FAp[id]->mgt_ctrl,FA_RELEASE_MGT_RESET);
      vmeWrite32(&FAp[id]->mgt_ctrl,FA_MGT_RESET);
      vmeWrite32(&FAp[id]->mgt_ctrl,FA_MGT_ENABLE_DATA_ALIGNMENT);
    }
  FAUNLOCK;

  return(OK);
}

int
faSyncResetMode(int id, unsigned int mode)
{
  return faSetMGTTestMode(id, mode);
}

/**************************************************************************************
 *
 *  faReadScalers - Scaler Data readout routine
 *        Readout the desired scalers (indicated by the channel mask), as well
 *        as the timer counter.  The timer counter will be the last word
 *        in the "data" array.
 *
 *    id     - Slot number of module to read
 *    data   - local memory address to place data
 *    chmask - Channel Mask (indicating which channels to read)
 *    rflag  - Readout Flag
 *            bit 0 - Latch Scalers before read
 *            bit 1 - Clear Scalers after read
 *
 *   RETURNS the number of 32bit words read, or ERROR if unsuccessful.
 */
int
faReadScalers(int id, volatile unsigned int *data, unsigned int chmask, int rflag)
{
  int doLatch=0, doClear=0, ichan=0;
  int dCnt=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faReadScalers: ERROR : ADC in slot %d is not initialized \n",
	     id,0,0,0,0,0);
      return ERROR;
    }

  if(rflag & ~(FA_SCALER_CTRL_MASK))
    {
      logMsg("faReadScalers: WARN : rflag (0x%x) has undefined bits \n",
	     rflag,0,0,0,0,0);
    }

  doLatch = rflag&(1<<0);
  doClear = rflag&(1<<1);

  FALOCK;
  if(doLatch)
    vmeWrite32(&FAp[id]->scaler_ctrl,
	       FA_SCALER_CTRL_ENABLE | FA_SCALER_CTRL_LATCH);

  for(ichan=0; ichan<16; ichan++)
    {
      if( (1<<ichan) & chmask )
	{
	  data[dCnt] = vmeRead32(&FAp[id]->scaler[ichan]);
	  dCnt++;
	}
    }

  data[dCnt] =  vmeRead32(&FAp[id]->time_count);
  dCnt++;

  if(doClear)
    vmeWrite32(&FAp[id]->scaler_ctrl,
	       FA_SCALER_CTRL_ENABLE | FA_SCALER_CTRL_RESET);
  FAUNLOCK;

  return dCnt;

}

/**************************************************************************************
 *
 *  faPrintScalers - Scaler Print Out routine
 *        Print out the scalers as well as the timer counter.
 *
 *    id     - Slot number of module to read
 *    rflag  - Printout Flag
 *            bit 0 - Latch Scalers before read
 *            bit 1 - Clear Scalers after read
 *
 *   RETURNS ok if successful , or ERROR if unsuccessful.
 */
int
faPrintScalers(int id, int rflag)
{
  int doLatch=0, doClear=0, ichan=0;
  unsigned int data[16], time_count;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faPrintScalers: ERROR : ADC in slot %d is not initialized \n",
	     id,0,0,0,0,0);
      return ERROR;
    }

  if(rflag & ~(FA_SCALER_CTRL_MASK))
    {
      logMsg("faPrintScalers: WARN : rflag (0x%x) has undefined bits \n",
	     rflag,0,0,0,0,0);
    }

  doLatch = rflag&(1<<0);
  doClear = rflag&(1<<1);

  FALOCK;
  if(doLatch)
    vmeWrite32(&FAp[id]->scaler_ctrl,
	       FA_SCALER_CTRL_ENABLE | FA_SCALER_CTRL_LATCH);

  for(ichan=0; ichan<16; ichan++)
    {
      data[ichan] = vmeRead32(&FAp[id]->scaler[ichan]);
    }

  time_count =  vmeRead32(&FAp[id]->time_count);

  if(doClear)
    vmeWrite32(&FAp[id]->scaler_ctrl,
	       FA_SCALER_CTRL_ENABLE | FA_SCALER_CTRL_RESET);
  FAUNLOCK;

  printf("%s: Scaler Counts\n",__FUNCTION__);
  for(ichan=0; ichan<16; ichan++)
    {
      if( (ichan%4) == 0 )
	printf("\n");

      printf("%2d: %10d ",ichan,data[ichan]);
    }
  printf("\n  timer: %10d\n",time_count);

  return OK;

}

int
faClearScalers(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faClearScalers: ERROR : ADC in slot %d is not initialized \n",
	     id,0,0,0,0,0);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->scaler_ctrl,
	     FA_SCALER_CTRL_ENABLE | FA_SCALER_CTRL_RESET);
  FAUNLOCK;

  return OK;
}


int
faLatchScalers(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faLatchScalers: ERROR : ADC in slot %d is not initialized \n",
	     id,0,0,0,0,0);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->scaler_ctrl,
	     FA_SCALER_CTRL_ENABLE | FA_SCALER_CTRL_LATCH);
  FAUNLOCK;

  return OK;
}

int
faEnableScalers(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faEnableScalers: ERROR : ADC in slot %d is not initialized \n",
	     id,0,0,0,0,0);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->scaler_ctrl,FA_SCALER_CTRL_ENABLE);
  FAUNLOCK;

  return OK;
}

int
faDisableScalers(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faDisableScalers: ERROR : ADC in slot %d is not initialized \n",
	     id,0,0,0,0,0);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->scaler_ctrl,~FA_SCALER_CTRL_ENABLE);
  FAUNLOCK;

  return OK;
}


/**************************************************************************************
 *
 *  faReadChargeScalers - Scaler Data readout routine
 *        Readout the desired scalers (indicated by the channel mask), as well
 *        as the timer counter.  The timer counter will be the last word
 *        in the "data" array.
 *
 *    id     - Slot number of module to read
 *    data   - local memory address to place data
 *    chmask - Channel Mask (indicating which channels to read)
 *    rflag  - Readout Flag
 *            bit 0 - Latch Scalers before read
 *            bit 1 - Clear Scalers after read
 *
 *   RETURNS the number of 32bit words read, or ERROR if unsuccessful.
 */
int
faReadChargeScalers(int id, volatile unsigned long long *data, unsigned int chmask)
{
  int ichan=0;
  int dCnt=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faReadChargeScalers: ERROR : ADC in slot %d is not initialized \n",
	     id,0,0,0,0,0);
      return ERROR;
    }

  FALOCK;
  vmeWrite16(&FAp[id]->adc_scaler_ctrl, 1);

  for(ichan=0; ichan<16; ichan++)
    {
      if( (1<<ichan) & chmask )
	{
	  data[dCnt]  = (unsigned long long)vmeRead16(&FAp[id]->adc_accumulator0[ichan]);
	  data[dCnt] |= ((unsigned long long)vmeRead16(&FAp[id]->adc_accumulator1[ichan]))<<16;
	  data[dCnt] |= ((unsigned long long)vmeRead16(&FAp[id]->adc_accumulator2[ichan]))<<32;
	  dCnt++;
	}
    }

  FAUNLOCK;

  return dCnt;

}

/**
 * @ingroup Status
 *  @brief Return the base address of the A32 for specified module
 *  @param id
 *   - Slot Number
 *  @return A32 address base, if successful. Otherwise ERROR.
 */

unsigned int
faGetA32(int id)
{
  unsigned int rval = 0;
  if(FApd[id])
    {
      rval = (unsigned int)((unsigned long)(FApd[id]) - fadcA32Offset);
    }
  else
    {
      logMsg("faGetA32(%d): A32 pointer not initialized\n",
	     id, 2, 3, 4, 5, 6);
      rval = ERROR;
    }

  return rval;
}


/**
 * @ingroup Status
 *  @brief Return the base address of the A32 Multiblock
 *  @return A32 multiblock address base, if successful. Otherwise ERROR.
 */

unsigned int
faGetA32M()
{
  unsigned int rval = 0;
  if(FApmb)
    {
      rval = (unsigned int)((unsigned long)(FApmb) - fadcA32Offset);
    }
  else
    {
      logMsg("faGetA32M: A32M pointer not initialized\n",
	     1, 2, 3, 4, 5, 6);
      rval = ERROR;
    }

  return rval;
}



/**
 *  @ingroup Status
 *  @brief Get the minimum address used for multiblock
 *  @param id Slot number
 *  @return multiblock min address if successful, otherwise ERROR.
 */
unsigned int
faGetMinA32MB(int id)
{
  unsigned int rval=0, a32addr, addrMB;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  FALOCK;

  a32addr = vmeRead32(&(FAp[id]->adr32));
  addrMB  = vmeRead32(&(FAp[id]->adr_mb));

  a32addr = (a32addr & FA_A32_ADDR_MASK)<<16;
  addrMB  = (addrMB & FA_AMB_MIN_MASK)<<16;

#ifdef DEBUG
  printf("faGetMinA32MB: a32addr=0x%08x addrMB=0x%08x for slot %d\n",a32addr,addrMB,id);
#endif

  id = fadcID[0];
  a32addr = vmeRead32(&(FAp[id]->adr32));
  a32addr = (a32addr & FA_A32_ADDR_MASK)<<16;

  rval = a32addr;
#ifdef DEBUG
  printf("faGetMinA32MB: rval=0x%08x\n",rval);
#endif

  FAUNLOCK;


  return rval;
}

/**
 *  @ingroup Status
 *  @brief Get the maximum address used for multiblock
 *  @param id Slot number
 *  @return multiblock max address if successful, otherwise ERROR.
 */
unsigned int
faGetMaxA32MB(int id)
{
  unsigned int rval=0, a32addr, addrMB;

  if(id==0) id=fadcID[0];
  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  FALOCK;

  a32addr = vmeRead32(&(FAp[id]->adr32));
  addrMB  = vmeRead32(&(FAp[id]->adr_mb));

  a32addr = (a32addr & FA_A32_ADDR_MASK)<<16;
  addrMB  = addrMB & FA_AMB_MAX_MASK;

  rval = addrMB;

#ifdef DEBUG
  printf("faGetMaxA32MB: a32addr=0x%08x addrMB=0x%08x for slot %d\n",a32addr,addrMB,id);
  printf("faGetMaxA32MB: rval=0x%08x\n",rval);
#endif

  FAUNLOCK;

  return rval;
}






/*********************************************
 *
 *  FADC Internal Trigger FADC Configuration and Control
 *  Routines.
 */
/*
  #include "cinclude/faItrig.c" / *sergey* /
*/ /*Ben*/



/* -------------------------------------------------------------------------------------

   Utility routines
*/

void
faPrintAuxScal(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faPrintAuxScal: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  printf("Auxillary Scalers:\n");
  printf("       Word Count:         %d\n",
	 vmeRead32(&FAp[id]->proc_words_scal));
  printf("       Headers   :         %d\n",
	 vmeRead32(&FAp[id]->header_scal));
  printf("       Trailers  :         %d\n",
	 vmeRead32(&FAp[id]->trailer_scal));
  FAUNLOCK;

  return;
}

void
faPrintFifoStatus(int id)
{
  unsigned int ibuf, bbuf, obuf, dflow;
  unsigned int wc[2],mt[2],full[2];
  unsigned int rdy[2];

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faPrintFifoStatus: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }

  FALOCK;
  dflow = vmeRead32(&(FAp[id]->dataflow_status));
  ibuf = vmeRead32(&(FAp[id]->status[0]))&0xdfffdfff;
  bbuf = vmeRead32(&(FAp[id]->status[1]))&0x1fff1fff;
  obuf = vmeRead32(&(FAp[id]->status[2]))&0x3fff3fff;
  FAUNLOCK;

  printf("%s: Fifo Buffers Status (DataFlow Status = 0x%08x\n",
	 __FUNCTION__,dflow);

  mt[1]  = full[1] = 0;
  wc[1]  = (ibuf&0x7ff0000)>>16;
  rdy[1] = (ibuf&0x80000000)>>31;
  if(ibuf&0x8000000)  full[1]=1;
  if(ibuf&0x10000000) mt[1]=1;

  printf("  Input Buffer : 0x%08x \n",ibuf);
  printf("    FPGA : wc=%d   Empty=%d Full=%d Ready=%d\n",wc[1],mt[1],full[1],rdy[1]);

  mt[0]=full[0]=0;
  wc[0]   =  bbuf&0x7ff;
  if(bbuf&0x800) full[0]=1;
  if(bbuf&0x1000) mt[0]=1;

  mt[1]=full[1]=0;
  wc[1]   = (bbuf&0x7ff0000)>>16;
  if(bbuf&0x8000000)  full[1]=1;
  if(bbuf&0x10000000) mt[1]=1;

  printf("  Build Buffer : 0x%08x \n",bbuf);
  printf("    BUF_A: wc=%d   Empty=%d Full=%d \n",wc[1],mt[1],full[1]);
  printf("    BUF_B: wc=%d   Empty=%d Full=%d \n",wc[0],mt[0],full[0]);

  mt[0]=full[0]=0;
  wc[0]   =  obuf&0xfff;
  if(obuf&0x1000) full[0]=1;
  if(obuf&0x2000) mt[0]=1;

  mt[1]=full[1]=0;
  wc[1]   = (obuf&0xfff0000)>>16;
  if(obuf&0x10000000)  full[1]=1;
  if(obuf&0x20000000) mt[1]=1;

  printf("  Output Buffer: 0x%08x \n",obuf);
  printf("    BUF_A: wc=%d   Empty=%d Full=%d \n",wc[1],mt[1],full[1]);
  printf("    BUF_B: wc=%d   Empty=%d Full=%d \n",wc[0],mt[0],full[0]);


  return;

}

int
faLive(int id, int sflag)
{
  int ilt=0;
  unsigned int live;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faLive: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  /* Read Current Scaler values */
  FALOCK;
  live = vmeRead32(&(FAp[id]->internal_trig_scal));

  vmeWrite32(&(FAp[id]->internal_trig_scal), 0x80000000);
  FAUNLOCK;

  if(live == 0)   /* scaler is zero or disabled */
    return(0);
  ilt = live;

  return(ilt);
}


void
faDataDecode(unsigned int data)
{
  int i_print = 1;
  static unsigned int type_last = 15;	/* initialize to type FILLER WORD */
  static unsigned int time_last = 0;

  if( data & 0x80000000 )		/* data type defining word */
    {
      fadc_data.new_type = 1;
      fadc_data.type = (data & 0x78000000) >> 27;
    }
  else
    {
      fadc_data.new_type = 0;
      fadc_data.type = type_last;
    }

  switch( fadc_data.type )
    {
    case 0:		/* BLOCK HEADER */
      fadc_data.slot_id_hd = (data & 0x7C00000) >> 22;
      fadc_data.n_evts = (data & 0x3FF800) >> 11;
      fadc_data.blk_num = (data & 0x7FF);
      if( i_print )
	printf("%8X - BLOCK HEADER - slot = %d   n_evts = %d   n_blk = %d\n",
	       data, fadc_data.slot_id_hd, fadc_data.n_evts, fadc_data.blk_num);
      break;
    case 1:		/* BLOCK TRAILER */
      fadc_data.slot_id_tr = (data & 0x7C00000) >> 22;
      fadc_data.n_words = (data & 0x3FFFFF);
      if( i_print )
	printf("%8X - BLOCK TRAILER - slot = %d   n_words = %d\n",
	       data, fadc_data.slot_id_tr, fadc_data.n_words);
      break;
    case 2:		/* EVENT HEADER */
      if( fadc_data.new_type )
	{
	  fadc_data.evt_num_1 = (data & 0x7FFFFFF);
	  if( i_print )
	    printf("%8X - EVENT HEADER 1 - evt_num = %d\n", data, fadc_data.evt_num_1);
	}
      else
	{
	  fadc_data.evt_num_2 = (data & 0x7FFFFFF);
	  if( i_print )
	    printf("%8X - EVENT HEADER 2 - evt_num = %d\n", data, fadc_data.evt_num_2);
	}
      break;
    case 3:		/* TRIGGER TIME */
      if( fadc_data.new_type )
	{
	  fadc_data.time_1 = (data & 0xFFFFFF);
	  if( i_print )
	    printf("%8X - TRIGGER TIME 1 - time = %08x\n", data, fadc_data.time_1);
	  fadc_data.time_now = 1;
	  time_last = 1;
	}
      else
	{
	  if( time_last == 1 )
	    {
	      fadc_data.time_2 = (data & 0xFFFFFF);
	      if( i_print )
		printf("%8X - TRIGGER TIME 2 - time = %08x\n", data, fadc_data.time_2);
	      fadc_data.time_now = 2;
	    }
	  else if( time_last == 2 )
	    {
	      fadc_data.time_3 = (data & 0xFFFFFF);
	      if( i_print )
		printf("%8X - TRIGGER TIME 3 - time = %08x\n", data, fadc_data.time_3);
	      fadc_data.time_now = 3;
	    }
	  else if( time_last == 3 )
	    {
	      fadc_data.time_4 = (data & 0xFFFFFF);
	      if( i_print )
		printf("%8X - TRIGGER TIME 4 - time = %08x\n", data, fadc_data.time_4);
	      fadc_data.time_now = 4;
	    }
	  else
	    if( i_print )
	      printf("%8X - TRIGGER TIME - (ERROR)\n", data);

	  time_last = fadc_data.time_now;
	}
      break;
    case 4:		/* WINDOW RAW DATA */
      if( fadc_data.new_type )
	{
	  fadc_data.chan = (data & 0x7800000) >> 23;
	  fadc_data.width = (data & 0xFFF);
	  if( i_print )
	    printf("%8X - WINDOW RAW DATA - chan = %d   nsamples = %d\n",
		   data, fadc_data.chan, fadc_data.width);
	}
      else
	{
	  fadc_data.valid_1 = 1;
	  fadc_data.valid_2 = 1;
	  fadc_data.adc_1 = (data & 0x1FFF0000) >> 16;
	  if( data & 0x20000000 )
	    fadc_data.valid_1 = 0;
	  fadc_data.adc_2 = (data & 0x1FFF);
	  if( data & 0x2000 )
	    fadc_data.valid_2 = 0;
	  if( i_print )
	    printf("%8X - RAW SAMPLES - valid = %d  adc = %4d   valid = %d  adc = %4d\n",
		   data, fadc_data.valid_1, fadc_data.adc_1,
		   fadc_data.valid_2, fadc_data.adc_2);
	}
      break;
    case 5:		/* WINDOW SUM */
      fadc_data.over = 0;
      fadc_data.chan = (data & 0x7800000) >> 23;
      fadc_data.adc_sum = (data & 0x3FFFFF);
      if( data & 0x400000 )
	fadc_data.over = 1;
      if( i_print )
	printf("%8X - WINDOW SUM - chan = %d   over = %d   adc_sum = %08x\n",
	       data, fadc_data.chan, fadc_data.over, fadc_data.adc_sum);
      break;
    case 6:		/* PULSE RAW DATA */
      if( fadc_data.new_type )
	{
	  fadc_data.chan = (data & 0x7800000) >> 23;
	  fadc_data.pulse_num = (data & 0x600000) >> 21;
	  fadc_data.thres_bin = (data & 0x3FF);
	  if( i_print )
	    printf("%8X - PULSE RAW DATA - chan = %d   pulse # = %d   threshold bin = %d\n",
		   data, fadc_data.chan, fadc_data.pulse_num, fadc_data.thres_bin);
	}
      else
	{
	  fadc_data.valid_1 = 1;
	  fadc_data.valid_2 = 1;
	  fadc_data.adc_1 = (data & 0x1FFF0000) >> 16;
	  if( data & 0x20000000 )
	    fadc_data.valid_1 = 0;
	  fadc_data.adc_2 = (data & 0x1FFF);
	  if( data & 0x2000 )
	    fadc_data.valid_2 = 0;
	  if( i_print )
	    printf("%8X - PULSE RAW SAMPLES - valid = %d  adc = %d   valid = %d  adc = %d\n",
		   data, fadc_data.valid_1, fadc_data.adc_1,
		   fadc_data.valid_2, fadc_data.adc_2);
	}
      break;
    case 7:		/* PULSE INTEGRAL */
      fadc_data.chan = (data & 0x7800000) >> 23;
      fadc_data.pulse_num = (data & 0x600000) >> 21;
      fadc_data.quality = (data & 0x180000) >> 19;
      fadc_data.integral = (data & 0x7FFFF);
      if( i_print )
	printf("%8X - PULSE INTEGRAL - chan = %d   pulse # = %d   quality = %d   integral = %d\n",
	       data, fadc_data.chan, fadc_data.pulse_num,
	       fadc_data.quality, fadc_data.integral);
      break;
    case 8:		/* PULSE TIME */
      fadc_data.chan = (data & 0x7800000) >> 23;
      fadc_data.pulse_num = (data & 0x600000) >> 21;
      fadc_data.quality = (data & 0x180000) >> 19;
      fadc_data.time = (data & 0xFFFF);
      if( i_print )
	printf("%8X - PULSE TIME - chan = %d   pulse # = %d   quality = %d   time = %d\n",
	       data, fadc_data.chan, fadc_data.pulse_num,
	       fadc_data.quality, fadc_data.time);
      break;
    case 9:		/* STREAMING RAW DATA */
      if( fadc_data.new_type )
	{
	  fadc_data.chan_a = (data & 0x3C00000) >> 22;
	  fadc_data.source_a = (data & 0x4000000) >> 26;
	  fadc_data.chan_b = (data & 0x1E0000) >> 17;
	  fadc_data.source_b = (data & 0x200000) >> 21;
	  if( i_print )
	    printf("%8X - STREAMING RAW DATA - ena A = %d  chan A = %d   ena B = %d  chan B = %d\n",
		   data, fadc_data.source_a, fadc_data.chan_a,
		   fadc_data.source_b, fadc_data.chan_b);
	}
      else
	{
	  fadc_data.valid_1 = 1;
	  fadc_data.valid_2 = 1;
	  fadc_data.adc_1 = (data & 0x1FFF0000) >> 16;
	  if( data & 0x20000000 )
	    fadc_data.valid_1 = 0;
	  fadc_data.adc_2 = (data & 0x1FFF);
	  if( data & 0x2000 )
	    fadc_data.valid_2 = 0;
	  fadc_data.group = (data & 0x40000000) >> 30;
	  if( fadc_data.group )
	    {
	      if( i_print )
		printf("%8X - RAW SAMPLES B - valid = %d  adc = %d   valid = %d  adc = %d\n",
		       data, fadc_data.valid_1, fadc_data.adc_1,
		       fadc_data.valid_2, fadc_data.adc_2);
	    }
	  else
	    if( i_print )
	      printf("%8X - RAW SAMPLES A - valid = %d  adc = %d   valid = %d  adc = %d\n",
		     data, fadc_data.valid_1, fadc_data.adc_1,
		     fadc_data.valid_2, fadc_data.adc_2);
	}
      break;
    case 10:		/* PULSE AMPLITUDE DATA */
      fadc_data.chan = (data & 0x7800000) >> 23;
      fadc_data.pulse_num = (data & 0x600000) >> 21;
      fadc_data.vmin = (data & 0x1FF000) >> 12;
      fadc_data.vpeak = (data & 0xFFF);
      if( i_print )
	printf("%8X - PULSE V - chan = %d   pulse # = %d   vmin = %d   vpeak = %d\n",
	       data, fadc_data.chan, fadc_data.pulse_num,
	       fadc_data.vmin, fadc_data.vpeak);
      break;

    case 11:		/* INTERNAL TRIGGER WORD */
      fadc_data.trig_type_int = data & 0x7;
      fadc_data.trig_state_int = (data & 0x8) >> 3;
      fadc_data.evt_num_int = (data & 0xFFF0) >> 4;
      fadc_data.err_status_int = (data & 0x10000) >> 16;
      if( i_print )
	printf("%8X - INTERNAL TRIGGER - type = %d   state = %d   num = %d   error = %d\n",
	       data, fadc_data.trig_type_int, fadc_data.trig_state_int, fadc_data.evt_num_int,
	       fadc_data.err_status_int);
    case 12:		/* UNDEFINED TYPE */
      if( i_print )
	printf("%8X - UNDEFINED TYPE = %d\n", data, fadc_data.type);
      break;
    case 13:		/* END OF EVENT */
      if( i_print )
	printf("%8X - END OF EVENT = %d\n", data, fadc_data.type);
      break;
    case 14:		/* DATA NOT VALID (no data available) */
      if( i_print )
	printf("%8X - DATA NOT VALID = %d\n", data, fadc_data.type);
      break;
    case 15:		/* FILLER WORD */
      if( i_print )
	printf("%8X - FILLER WORD = %d\n", data, fadc_data.type);
      break;
    }

  type_last = fadc_data.type;	/* save type of current data word */

}

void
faTestSetSystemTestMode(int id, int mode)
{
  int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  if(mode>=1)
    reg=FA_CTRL1_SYSTEM_TEST_MODE;
  else
    reg=0;

  FALOCK;

  vmeWrite32(&(FAp[id]->ctrl1),
	     vmeRead32(&FAp[id]->ctrl1) | reg);

  /*   printf(" ctrl1 = 0x%08x\n",vmeRead32(&FAp[id]->ctrl1)); */
  FAUNLOCK;

}

void
faTestSetTrigOut(int id, int mode)
{
  int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  if(mode>=1)
    reg=FA_TESTBIT_TRIGOUT;
  else
    reg=0;

  FALOCK;
  vmeWrite32(&(FAp[id]->testBit),reg);
  FAUNLOCK;

}

void
faTestSetBusyOut(int id, int mode)
{
  int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  if(mode>=1)
    reg=FA_TESTBIT_BUSYOUT;
  else
    reg=0;

  FALOCK;
  vmeWrite32(&(FAp[id]->testBit),reg);
  FAUNLOCK;

}

void
faTestSetSdLink(int id, int mode)
{
  int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  if(mode>=1)
    reg=FA_TESTBIT_SDLINKOUT;
  else
    reg=0;

  FALOCK;
  vmeWrite32(&(FAp[id]->testBit),reg);
  FAUNLOCK;

}

void
faTestSetTokenOut(int id, int mode)
{
  int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  if(mode>=1)
    reg=FA_TESTBIT_TOKENOUT;
  else
    reg=0;

  FALOCK;
  vmeWrite32(&(FAp[id]->testBit),reg);
  FAUNLOCK;

}

int
faTestGetStatBitB(int id)
{
  int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  reg = (vmeRead32(&FAp[id]->testBit) & FA_TESTBIT_STATBITB)>>8;
  FAUNLOCK;

  return reg;

}

int
faTestGetTokenIn(int id)
{
  int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  reg = (vmeRead32(&FAp[id]->testBit) & FA_TESTBIT_TOKENIN)>>9;
  FAUNLOCK;

  return reg;

}

int
faTestGetClock250CounterStatus(int id)
{
  int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  reg = (vmeRead32(&FAp[id]->testBit) & FA_TESTBIT_CLOCK250_STATUS)>>15;
  FAUNLOCK;

  return reg;

}

unsigned int
faTestGetClock250Counter(int id)
{
  unsigned int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  reg = vmeRead32(&FAp[id]->clock250count);
  FAUNLOCK;

  return reg;

}

unsigned int
faTestGetSyncCounter(int id)
{
  unsigned int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  reg = vmeRead32(&FAp[id]->syncp0count);
  FAUNLOCK;

  return reg;

}

unsigned int
faTestGetTrig1Counter(int id)
{
  unsigned int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  reg = vmeRead32(&FAp[id]->trig1p0count);
  FAUNLOCK;

  return reg;

}

unsigned int
faTestGetTrig2Counter(int id)
{
  unsigned int reg=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  reg = vmeRead32(&FAp[id]->trig2p0count);
  FAUNLOCK;

  return reg;

}

void
faTestResetClock250Counter(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->clock250count,FA_CLOCK250COUNT_RESET);
  vmeWrite32(&FAp[id]->clock250count,FA_CLOCK250COUNT_START);
  FAUNLOCK;

}

void
faTestResetSyncCounter(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->syncp0count,FA_SYNCP0COUNT_RESET);
  FAUNLOCK;

}

void
faTestResetTrig1Counter(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->trig1p0count,FA_TRIG1P0COUNT_RESET);
  FAUNLOCK;

}

void
faTestResetTrig2Counter(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->trig2p0count,FA_TRIG2P0COUNT_RESET);
  FAUNLOCK;

}

unsigned int
faTestGetTestBitReg(int id)
{
  unsigned int rval=0;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return 0;
    }

  FALOCK;
  rval = vmeRead32(&FAp[id]->testBit);
  FAUNLOCK;

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Return the status of the current system clock
 *
 *   Enable and disables test mode during operation
 *
 *  @sa faTestSetSystemTestMode
 *  @sa faTestGetClock250CounterStatus
 *  @sa faTestGetClock250Counter
 *  @sa faTestResetClock250Counter
 *
 *  @param id Slot number
 *  @param pflag print to standard output if > 0
 *
 *  @return OK if system clock is available, otherwise ERROR.
 */

int
faTestSystemClock(int id, int pflag)
{
  unsigned int rval=OK;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  /* Enable test mode */
  faTestSetSystemTestMode(id, 1);

  /* reset clock counter */
  faTestResetClock250Counter(id);

  int iwait = 0;
  /* Wait for the 20us internal timer */
  while(iwait++ < 50)
    {
      if(faTestGetClock250CounterStatus(id) == 0)
	break;
    }

  /* Counter should return 5000 if the system clock is 250Mhz */
  int expected = 5000, measured = 0, diff = 0;

  measured = faTestGetClock250Counter(id);
  diff = abs(expected - measured);

  if(diff < 5)
    rval = OK;
  else
    rval = ERROR;

  /* Disable test mode */
  faTestSetSystemTestMode(id, 0);

  if(pflag)
    {
      printf("%s: System Clock is %s\n",
	     __func__, (rval==OK) ? "Present" : "NOT PRESENT");
    }

  return rval;
}


/**************************************************************************************
 *
 *  faGetSerialNumber - Available for firmware>=0x0211
 *      Fills 'rval' with a character array containing the fa250 serial number.
 *
 *      If snfix >= 1, will attempt to format the serial number to maintain
 *        consistency between serials made by the same vendor.
 *        e.g. Advanced Assembly:
 *          snfix=0: B21595-02R  B2159515R1
 *          snfix=1: B21595-02R  B21595-15R1
 *        e.g. ACDI
 *          snfix=0: ACDI002
 *          snfix=1: ACDI-002
 *
 *
 *   RETURNS length of character array 'rval' if successful, otherwise ERROR
 */

int
faGetSerialNumber(int id, char **rval, int snfix)
{
  unsigned int sn[3];
  int i=0, ivme=0, ibyte=0;
  unsigned int byte;
  unsigned int shift=0, mask=0;
  unsigned int boardID;
  char boardID_c[4];
  char byte_c[2];
  char adv_str[12], acdi_str[12];
  char ret[12];
  int ret_len;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  for(i=0; i<3; i++)
    sn[i] = vmeRead32(&FAp[id]->serial_number[i]);
  FAUNLOCK;

  if(sn[0]==FA_SERIAL_NUMBER_ACDI)
    { /* ACDI */
      strcpy(acdi_str,"");
      for(ibyte=3; ibyte>=0; ibyte--)
	{
	  shift = (ibyte*8);
	  mask = (0xFF)<<shift;
	  byte = (sn[ivme]&mask)>>shift;
	  sprintf(byte_c,"%c",byte);
	  strcat(acdi_str,byte_c);
	}
      boardID = (sn[1] & FA_SERIAL_NUMBER_ACDI_BOARDID_MASK);
      if(boardID>999)
	{
	  printf("%s: WARN: Invalid Board ACDI Board ID (%d)\n",
		 __FUNCTION__,boardID);
	}

      if(snfix>0) /* If needed, Add '-' after the ACDI */
	sprintf(boardID_c,"-%03d",boardID);
      else
	sprintf(boardID_c,"%03d",boardID);

      strcat(acdi_str,boardID_c);
#ifdef DEBUGSN
      printf("acdi_str = %s\n",acdi_str);
#endif
      strcpy(ret,acdi_str);

    }

  else if((sn[0] & FA_SERIAL_NUMBER_ADV_ASSEM_MASK)==FA_SERIAL_NUMBER_ADV_ASSEM)

    { /* ADV ASSEM */
      /* Make sure manufacture's ID is correct */
      if((sn[0] == FA_SERIAL_NUMBER_ADV_MNFID1) &&
	 ((sn[1] & FA_SERIAL_NUMBER_ADV_MNFID2_MASK) == FA_SERIAL_NUMBER_ADV_MNFID2) )
	{
	  strcpy(adv_str,"");
	  for(ivme=0; ivme<3; ivme++)
	    {
	      for(ibyte=3; ibyte>=0; ibyte--)
		{
		  shift = (ibyte*8);
		  mask = (0xFF)<<shift;
		  byte = (sn[ivme]&mask)>>shift;
		  if(byte==0xFF)
		    {
		      break;
		    }
		  if(snfix>0)
		    { /* If needed, Add '-' after the B21595 */
		      if(ivme==1 && ibyte==1)
			{
			  if(byte!=0x2D) /* 2D = - */
			    {
			      strcat(adv_str,"-");
			    }
			}
		    }

		  sprintf(byte_c,"%c",byte);
		  strcat(adv_str,byte_c);
		}
	    }
#ifdef DEBUGSN
	  printf("adv_str = %s\n",adv_str);
#endif
	  strcpy(ret,adv_str);
	}
      else
	{
	  printf("%s: ERROR: Unable to determine manufacture's ID.  SN regs:\n",
		 __FUNCTION__);
	  for(i=0; i<3; i++)
	    printf("\t%d: 0x%08x\n",i,sn[i]);
	  return -1;
	}
    }
  else
    {
      printf("%s: ERROR: Unable to determine manufacture's ID. SN regs:\n",
	     __FUNCTION__);
      for(i=0; i<3; i++)
	printf("\t%d: 0x%08x\n",i,sn[i]);
      return -1;
    }

#ifdef DEBUGSN
  printf("ret = %s\n",ret);
#endif
  strcpy((char *)rval,ret);

  ret_len = (int)strlen(ret);

  return(ret_len);

}


/**************************************************************************************
 *
 *  faSetScalerBlockInterval
 *      Data from scalers may be inserted into the readout data stream
 *      at regular event count intervals.  The interval is specified in
 *      multiples of blocks.
 *
 *    Argument:
 *        nblock:
 *             0 : No insertion of scaler data into the data stream
 *           >=1 : The current scaler values are appended to the last event
 *                  of the appropriate n'th block of events.
 *
 *    Note: Scalers are NOT reset after their values are captured.
 *
 *   RETURNS OK if successful, otherwise ERROR.
 */

int
faSetScalerBlockInterval(int id, unsigned int nblock)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  if(nblock > FA_SCALER_INTERVAL_MASK)
    {
      printf("%s: ERROR: Invalid value of nblock (%d).\n",
	     __FUNCTION__,nblock);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->scaler_interval,nblock);
  FAUNLOCK;

  return OK;
}

int
faGetScalerBlockInterval(int id)
{
  int rval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",__FUNCTION__,
	     id);
      return ERROR;
    }

  FALOCK;
  rval = vmeRead32(&FAp[id]->scaler_interval) & FA_SCALER_INTERVAL_MASK;
  FAUNLOCK;

  return rval;
}

/**************************************************************************************
 *
 *  faForceEndOfBlock
 *      Allows for the insertion of a block trailer into the data stream.  This is
 *      useful for the efficient extraction of a partial block of events
 *      from the FADC (e.g. for an end of run event, or the resynchonize with
 *      other modules).
 *      Event count within block is reset, after successful execution.
 *
 *   ARG:
 *     scalers:  If set to > 0, scalers will also be inserted with the End of Block
 *
 *   RETURNS OK if successful, otherwise ERROR.
 */

int
faForceEndOfBlock(int id, int scalers)
{
  int rval=OK, icheck=0, timeout=1000, csr=0;
  int proc_config=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faForceEndOfBlock: ERROR : ADC in slot %d is not initialized \n",
	     id,2,3,4,5,6);
      return ERROR;
    }

  FALOCK;
  /* Disable triggers to Processing FPGA (if enabled) */
  proc_config = vmeRead32(&FAp[id]->adc_config[0]);
  vmeWrite32(&FAp[id]->adc_config[0],
	     proc_config & ~(FA_ADC_PROC_ENABLE));

  csr = FA_CSR_FORCE_EOB_INSERT;
  if(scalers>0)
    csr |= FA_CSR_DATA_STREAM_SCALERS;

  vmeWrite32(&FAp[id]->csr, csr);

  for(icheck=0; icheck<timeout; icheck++)
    {
      csr = vmeRead32(&FAp[id]->csr);
      if(csr & FA_CSR_FORCE_EOB_SUCCESS)
	{
	  logMsg("faForceEndOfBlock: Block trailer insertion successful\n",
		 1,2,3,4,5,6);
	  rval = ERROR;
	  break;
	}

      if(csr & FA_CSR_FORCE_EOB_FAILED)
	{
	  logMsg("faForceEndOfBlock: Block trailer insertion FAILED\n",
		 1,2,3,4,5,6);
	  rval = ERROR;
	  break;
	}
    }

  if(icheck==timeout)
    {
      logMsg("faForceEndOfBlock: Block trailer insertion FAILED on timeout\n",
	     1,2,3,4,5,6);
      rval = ERROR;
    }

  /* Restore the original state of the Processing FPGA */
  vmeWrite32(&FAp[id]->adc_config[0], proc_config);

  FAUNLOCK;

  return rval;
}

void
faGForceEndOfBlock(int scalers)
{
  int ii, res;

  for (ii=0;ii<nfadc;ii++) {
    res = faForceEndOfBlock(fadcID[ii], scalers);
    if(res<0)
      printf("%s: ERROR: slot %d, in faForceEndOfBlock()\n",
	     __FUNCTION__,fadcID[ii]);
  }

}

/***************************************************************************************
   JLAB FADC Signal Distribution Card (SDC) Routines

   cFlag:  controls the configuation of the SDC

          0:  Default Mode  Internal CLK, Sync External Trigger and Sync Reset
        > 0:  Pass through mode

   bMask:  mask of Busy enables for the SDC - Do not Enable busy if there is no FADC

*/

int
faSDC_Config(unsigned short cFlag, unsigned short bMask)
{

  if(FASDCp == NULL)
    {
      logMsg("faSDC_Config: ERROR : Cannot Configure FADC Signal Board \n",0,0,0,0,0,0);
      return(ERROR);
    }

  /* Reset the Board */
  FASDCLOCK;
  vmeWrite16(&(FASDCp->csr),FASDC_CSR_INIT);

  if(cFlag == 0)
    {
      /* Default - Enable Internal Clock, Sync Trigger and Sync-Reset*/
      vmeWrite16(&(FASDCp->ctrl),(FASDC_CTRL_ENABLE_SOFT_TRIG | FASDC_CTRL_ENABLE_SOFT_SRESET));
      fadcSDCPassthrough=0;
    }
  else if(cFlag==1)
    {
      /* Pass Through - */
      vmeWrite16(&(FASDCp->ctrl),(FASDC_CTRL_CLK_EXT | FASDC_CTRL_NOSYNC_TRIG | FASDC_CTRL_NOSYNC_SRESET));
      fadcSDCPassthrough=1;
  }
  else
    {
      /* Level Translator - re-sync the signals coming in to the SDC*/
      vmeWrite16(&(FASDCp->ctrl),(FASDC_CTRL_CLK_EXT));
      fadcSDCPassthrough=1;
    }

  vmeWrite16(&(FASDCp->busy_enable),bMask);
  FASDCUNLOCK;

  return(OK);
}

void
faSDC_Status(int sFlag)
{

  unsigned short sdc[4];
  int ibit=0;

  if(FASDCp == NULL)
    {
      printf("faSDC_Status: ERROR : No FADC SDC available \n");
      return;
    }

  FASDCLOCK;
  sdc[0] = vmeRead16(&(FASDCp->csr));
  sdc[1] = vmeRead16(&(FASDCp->ctrl))&FASDC_CTRL_MASK;
  sdc[2] = vmeRead16(&(FASDCp->busy_enable))&FASDC_BUSY_MASK;
  sdc[3] = vmeRead16(&(FASDCp->busy_status));
  FASDCUNLOCK;


#ifdef VXWORKS
  printf("\nSTATUS for FADC Signal Distribution Card at base address 0x%x \n",(UINT32) FASDCp);
#else
  printf("\nSTATUS for FADC Signal Distribution Card at\n VME (Local) base address 0x%x (0x%lx)\n",
	 (UINT32)((unsigned long)FASDCp - fadcA16Offset), (unsigned long) FASDCp);
#endif
  printf("---------------------------------------------------------------- \n");

  printf(" Board Firmware Rev/ID = 0x%02x\n",((sdc[0]&0xff00)>>8));
  printf(" Registers: \n");
  printf("   CSR         = 0x%04x     Control     = 0x%04x\n",sdc[0],sdc[1]);
  printf("   Busy Enable = 0x%04x     Busy Status = 0x%04x\n",sdc[2],sdc[3]);
  printf("\n");

  if((sdc[1]&FASDC_CTRL_CLK_EXT))
    printf(" Ref Clock : External\n");
  else
    printf(" Ref Clock : Internal\n");


  printf("   Trigger :");
  if((sdc[1]&FASDC_CTRL_ENABLE_SOFT_TRIG))
    {
      printf(" Internal (Software)\n");
    }
  else
    {
      if((sdc[1]&FASDC_CTRL_NOSYNC_TRIG))
	printf(" External (Pass through)\n");
      else
	printf(" External (Sync with clock)\n");
    }

  printf(" SyncReset :");
  if((sdc[1]&FASDC_CTRL_ENABLE_SOFT_SRESET))
    {
      printf(" Internal (Software)\n");
    }
  else
    {
      if((sdc[1]&FASDC_CTRL_NOSYNC_SRESET))
	printf(" External (Pass through)\n");
      else
	printf(" External (Sync with clock)\n");
    }
  printf("\n");
  printf(" Busy Ports\n  Enabled  :");
  for(ibit = 0; ibit < 7; ibit++)
    if((1 << ibit) & sdc[2])
      printf(" %d", ibit+1);

  printf("\n");

  printf("\n");
  printf(" Busy Ports\n  Asserted :");
  for(ibit = 0; ibit < 7; ibit++)
    if((1 << ibit) & sdc[3])
      printf(" %d", ibit+1);

  printf("\n");

  printf("\n");

}


void
faSDC_Enable(int nsync)
{

  if(FASDCp == NULL)
    {
      logMsg("faSDC_Enable: ERROR : No FADC SDC available \n",0,0,0,0,0,0);
      return;
    }

  FASDCLOCK;
  if(nsync != 0) /* FP triggers only */
    vmeWrite16(&(FASDCp->ctrl),FASDC_CTRL_ENABLE_SOFT_SRESET);
  else      /* both FP triggers and sync reset */
    vmeWrite16(&(FASDCp->ctrl),0);
  FASDCUNLOCK;
}

void
faSDC_Disable()
{

  if(FASDCp == NULL)
    {
      logMsg("faSDC_Disable: ERROR : No FADC SDC available \n",0,0,0,0,0,0);
      return;
    }

  FASDCLOCK;
  vmeWrite16(&(FASDCp->ctrl),(FASDC_CTRL_ENABLE_SOFT_TRIG | FASDC_CTRL_ENABLE_SOFT_SRESET));
  FASDCUNLOCK;
}



void
faSDC_Sync()
{

  if(FASDCp == NULL)
    {
      logMsg("faSDC_Sync: ERROR : No FADC SDC available \n",0,0,0,0,0,0);
      return;
    }

  FASDCLOCK;
  vmeWrite16(&(FASDCp->csr),FASDC_CSR_SRESET);
  FASDCUNLOCK;
}

void
faSDC_Trig()
{
  if(FASDCp == NULL)
    {
      logMsg("faSDC_Trig: ERROR : No FADC SDC available \n",0,0,0,0,0,0);
      return;
    }

  FASDCLOCK;
  vmeWrite16(&(FASDCp->csr),FASDC_CSR_TRIG);
  FASDCUNLOCK;
}

int
faSDC_Busy()
{
  int busy=0;

  if(FASDCp == NULL)
    {
      logMsg("faSDC_Busy: ERROR : No FADC SDC available \n",0,0,0,0,0,0);
      return -1;
    }

  FASDCLOCK;
  busy = vmeRead16(&(FASDCp->csr))&FASDC_CSR_BUSY;
  FASDCUNLOCK;

  return(busy);
}



/********************************/
/* sergey: returns some globals */



int
faGetProcMode(int id, int *pmode, unsigned int *PL, unsigned int *PTW,
	      unsigned int *NSB, unsigned int *NSA, unsigned int *NP)
{
  unsigned int tmp;

  if(id==0) id=fadcID[0];
  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faGetProcMode: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  *PTW    = (vmeRead32(&(FAp[id]->adc_ptw))&0xFFFF);
  *PL     = (vmeRead32(&(FAp[id]->adc_pl))&0xFFFF);
  *NSB    = (vmeRead32(&(FAp[id]->adc_nsb))&0xFFFF);
  *NSA    = (vmeRead32(&(FAp[id]->adc_nsa))&0xFFFF);

  tmp     = (vmeRead32(&(FAp[id]->adc_config[0]))&0xFFFF);
  *pmode  = (tmp&FA_ADC_PROC_MASK) + 1;
  *NP     = (tmp&FA_ADC_PEAK_MASK)>>4;

  return(0);
}




int
faGetNfadc()
{
  return(nfadc);
}

/*
  int
  faSlot(unsigned int id)
  {
  if(id>=nfadc)
  {
  printf("%s: ERROR: Index (%d) >= FADCs initialized (%d).\n",__FUNCTION__,id,nfadc);
  return(ERROR);
  }

  return(fadcID[id]);
  }
*/


int
faId(unsigned int slot)
{
  int id;

  for(id=0; id<nfadc; id++)
    {
      if(fadcID[id]==slot)
	{
	  return(id);
	}
    }

  printf("%s: ERROR: FADC in slot %d does not exist or not initialized.\n",__FUNCTION__,slot);
  return(ERROR);
}

int
faSetThresholdAll(int id, unsigned short tvalue[16])
{
  int ii;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetThresholdAll: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  for(ii=0; ii<FA_MAX_ADC_CHANNELS; ii++)
    {
      faSetChThreshold(id, ii, tvalue[ii]);
    }

  return(OK);
}



/*sergey: set same pedestal for all channels, will change it later*/

/*
  todo
  setgain
  printgain
*/

int
faSetPedestal(int id, unsigned int wvalue)
{
  int ii;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faSetPedestal: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  for(ii=0; ii<FA_MAX_ADC_CHANNELS; ii++)
    {
      if(!(ii&0x1))
	vmeWrite32((unsigned int *)&(FAp[id]->adc_pedestal[ii]),wvalue | (wvalue<<16));
    }
  FAUNLOCK;

  return(OK);
}
int
faPrintPedestal(int id)
{
  int ii;
  unsigned int tval[FA_MAX_ADC_CHANNELS];

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faPrintPedestal: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  for(ii=0; ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      tval[ii] = vmeRead16(&(FAp[id]->adc_pedestal[ii]));
    }
  FAUNLOCK;


  printf(" Pedestal Settings for FADC in slot %d:",id);
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++)
    {
      if((ii%4)==0)
	{
	  printf("\n");
	}
      printf("chan %2d: %3d   ",(ii+1),tval[ii]);
    }
  printf("\n");

  return(OK);
}



/**************************/
/* begin debugging for Ed */

/* Ed's email:

   I am adding state machine tracing code to the firmware.  This will save the last 500 state changes of the data flow control state machine into a FIFO.  Please modify your code to do the following:


   STATE_CSR register:  address = 0x504

   STATE_VALUE register:  address = 0x508


   Before 'GO':

   - Arm the storage of states by writing value 0x80000000 to STATE_CSR  (0x504)


   After the error occurs and all status registers have been read and printed:

   - Disarm storage of states by writing 0x0 to STATE_CSR  (0x504)

   - Read (and print) STATE_CSR (= value) to get the number of valid states stored

   - Print all valid stored state values:

   num_states = 0x1FF & value;

   for(ii = 0; ii < num_states; ii++)

   {

   read and print STATE_VALUE register  (0x508);      // read gets next value in FIFO

   }


   ----------------

   Since the data flow state machine will stop changing when the triggers stop due to the busy assertion I can trace back the history of this state machine.  The Xilinx Chipscope tool records state values on each clock edge and thus can only show a limited history of the state machine.

   ----------------
*/

int
faArmStatesStorage(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faArmStatesStorage: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->state_csr),0x80000000);
  FAUNLOCK;

  printf("faArmStatesStorage: ARMED slot %d\n",id);

  return(OK);
}

int
faDisarmStatesStorage(int id)
{
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faDisarmStatesStorage: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;
  vmeWrite32(&(FAp[id]->state_csr),0x0);
  FAUNLOCK;

  printf("faDisarmStatesStorage: DISARMED slot %d\n",id);

  return(OK);
}

int
faReadStatesStorage(int id)
{
  int ii;
  unsigned int value, fifo;
  int num_states;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      logMsg("faReadStatesStorage: ERROR : ADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }

  FALOCK;

  value = vmeRead32(&(FAp[id]->state_csr));

  num_states = value & 0x1FF;
  printf("\nfaReadStatesStorage: slot %d, state_csr = 0x%08x, num_states = %d\n",id,value,num_states);

  /* read and print STATE_VALUE fifo */
  for(ii=0; ii<num_states; ii++)
    {
      fifo = vmeRead32(&(FAp[id]->state_value));
      printf("faReadStatesStorage: fifo[%4d] = 0x%08x\n",ii,fifo);
    }

  FAUNLOCK;

  printf("faReadStatesStorage: done printing fifo\n\n");

  return(OK);
}


/*  end debugging for Ed  */
/**************************/


#else /* dummy version*/

void
fadcLib_dummy()
{
  return;
}

#endif
