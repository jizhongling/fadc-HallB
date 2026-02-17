/*************************************************************************
 *
 *  fadc_list.c - Library of routines for the user to write for
 *                readout and buffering of events from JLab FADC using
 *                a JLab pipeline TI module and Linux VME controller.
 *
 *                In this example, clock, syncreset, and trigger are
 *                output from the TI then distributed using a Front Panel SDC.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*60      /* Size in Bytes */

/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL  /* Poll for available data, external triggers */
#define TI_ADDR    (21<<19)          /* GEO slot 21 */

#define FIBER_LATENCY_OFFSET 0x4A  /* measured longest fiber length */

#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "fadcLib.h"        /* library of FADC250 routines */

#define BUFFERLEVEL 1

/* FADC Library Variables */
extern int fadcA32Base, nfadc;
/* Number of fadc250 to initialize */
#define NFADC     1
/* Address of first fADC250 (set to slot# << 19 )*/
#define FADC_ADDR (4 << 19)
/* Increment address to find next fADC250 (increment by 1 slot) */
#define FADC_INCR (1 << 19)

#define FADC_BANK 0x3

#define FADC_WINDOW_LAT    375
#define FADC_WINDOW_WIDTH   24
#define FADC_MODE           10

/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;

/* function prototype */
void rocTrigger(int arg);

void
rocDownload()
{
  unsigned short iflag;
  int ifa, stat;

  /* Configure TI */
#ifdef TI_MASTER
  /*
   * Set Trigger source
   *    For the TI-Master, valid sources:
   *      TI_TRIGGER_FPTRG     2  Front Panel "TRG" Input
   *      TI_TRIGGER_TSINPUTS  3  Front Panel "TS" Inputs
   *      TI_TRIGGER_TSREV2    4  Ribbon cable from Legacy TS module
   *      TI_TRIGGER_PULSER    5  TI Internal Pulser (Fixed rate and/or random)
   */
  tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */

  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 );

  /* Load the trigger table that associates
   *    - TS#1,2,3,4,5,6 : Physics trigger,
   */
  tiLoadTriggerTable(3);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set the SyncReset width to 4 microSeconds */
  tiSetSyncResetType(1);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  /* Sync event every 1000 blocks */
  tiSetSyncEventInterval(1000);

  /* Set L1A prescale ... rate/(x+1) */
  tiSetPrescale(0);

  /* Set TS input #1 prescale rate/(2^(x-1) + 1)*/
  tiSetInputPrescale(1, 0);
#endif

  /* Add trigger latch pattern to datastream */
  tiSetFPInputReadout(1);

  /* Program/Init FADC Modules Here */
  iflag = 0xea00; /* SDC Board address */
  iflag |= FA_INIT_EXT_SYNCRESET;  /* Front panel sync-reset */
  iflag |= FA_INIT_FP_TRIG;  /* Front Panel Input trigger source */
  iflag |= FA_INIT_FP_CLKSRC;  /* Internal 250MHz Clock source */

  fadcA32Base = 0x09000000;

  vmeSetQuietFlag(1);
  faInit(FADC_ADDR, FADC_INCR, NFADC, iflag);
  vmeSetQuietFlag(0);

  for(ifa = 0; ifa < nfadc; ifa++)
    {
      faEnableBusError(faSlot(ifa));

      /* Set input DAC level */
      faSetDAC(faSlot(ifa), 3250, 0);

      /*  Set All channel thresholds to 150 */
      faSetThreshold(faSlot(ifa), 150, 0xffff);

      /*********************************************************************************
       * faSetProcMode(int id, int pmode, unsigned int PL, unsigned int PTW,
       *    int NSB, unsigned int NSA, unsigned int NP,
       *    unsigned int NPED, unsigned int MAXPED, unsigned int NSAT);
       *
       *   id Slot number
       *   pmode  Processing Mode
       *     -     1 - Raw Window
       *     -     2 - Pulse Raw Window
       *     -     3 - Pulse Integral
       *     -     4 - High-resolution time
       *     -     7 - Mode 3 + Mode 4
       *     -     8 - Mode 1 + Mode 4
       *    PL  Window Latency
       *   PTW  Window Width
       *   NSB  Number of samples before pulse over threshold
       *   NSA  Number of samples after pulse over threshold
       *   NP   Number of pulses processed per window
       *   bank Ignored
       */
      faSetProcMode(faSlot(ifa),
		    FADC_MODE,   /* Processing Mode */
		    FADC_WINDOW_LAT, /* PL */
		    FADC_WINDOW_WIDTH,  /* PTW */
		    3,   /* NSB */
		    6,   /* NSA */
		    1,   /* NP */
		    0);  /* not used */


    }

  faSDC_Status(0);
  faGStatus(0);

  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  int ifa;

  /* Program/Init VME Modules Here */
  for(ifa=0; ifa < nfadc; ifa++)
    {
      faSoftReset(faSlot(ifa),0);
      faResetTriggerCount(faSlot(ifa));
      faEnableSyncReset(faSlot(ifa));
    }

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  /* Sync Reset to synchronize TI and fadc250 timestamps and their internal buffers */
  faSDC_Sync();

  tiStatus(0);
  faGStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{
  int fadc_mode = 0, pl=0, ptw=0, nsb=0, nsa=0, np=0;

  /* Get the current block level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n",
	 __FUNCTION__,blockLevel);

  faGSetBlockLevel(blockLevel);

  /* Set Max words from fadc (proc mode == 1 produces the most)
     nfadc * ( Block Header + Trailer + 2  # 2 possible filler words
               blockLevel * ( Event Header + Header2 + Timestamp1 + Timestamp2 +
	                      nchan * (Channel Header + (WindowSize / 2) )
             ) +
     scaler readout # 16 channels + header/trailer
   */
  MAXFADCWORDS = nfadc * (4 + blockLevel * (4 + 16 * (1 + (ptw / 2))) + 18);

  /*  Enable FADC */
  faGEnable(0, 0);

  /* Interrupts/Polling enabled after conclusion of rocGo() */
}

void
rocEnd()
{

  /* FADC Disable */
  faGDisable(0);

  /* FADC Event status - Is all data read out */
  faGStatus(0);

  tiStatus(0);

  printf("rocEnd: Ended after %d events\n",tiGetIntCount());

}

void
rocTrigger(int arg)
{
  int ifa, stat, nwords, dCnt;
  unsigned int datascan, scanmask;
  int roCount = 0, blockError = 0;

  roCount = tiGetIntCount();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  dCnt = tiReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    {
      dma_dabufp += dCnt;
    }

  /* fADC250 Readout */
  BANKOPEN(FADC_BANK ,BT_UI4,0);

  /* Mask of initialized modules */
  scanmask = faScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faGBlockReady(scanmask, 100);
  stat = (datascan == scanmask);

  if(stat)
    {
      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  nwords = faReadBlock(faSlot(ifa), dma_dabufp, MAXFADCWORDS, 1);

	  /* Check for ERROR in block read */
	  blockError = faGetBlockError(1);

	  if(blockError)
	    {
	      printf("ERROR: Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		     faSlot(ifa), roCount, nwords);

	      if(nwords > 0)
		dma_dabufp += nwords;
	    }
	  else
	    {
	      dma_dabufp += nwords;
	    }
	}
    }
  else
    {
      printf("ERROR: Event %d: Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }
  BANKCLOSE;


  /* Check for SYNC Event */
  if(tiGetSyncEventFlag() == 1)
    {
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  printf("%s: ERROR: TI Data available (%d) after readout in SYNC event \n",
		 __func__, davail);

	  while(tiBReady())
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

      for(ifa = 0; ifa < nfadc; ifa++)
	{
	  davail = faBready(faSlot(ifa));
	  if(davail > 0)
	    {
	      printf("%s: ERROR: fADC250 Data available after readout in SYNC event \n",
		     __func__, davail);

	      while(faBready(faSlot(ifa)))
		{
		  vmeDmaFlush(faGetA32(faSlot(ifa)));
		}
	    }
	}
    }

}

void
rocCleanup()
{

  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faGReset(1);

}

/*
  Local Variables:
  compile-command: "make -k fadc_list.so"
  End:
 */
