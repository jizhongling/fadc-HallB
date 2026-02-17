/* Module: fadcFirmwareTools.c
 *
 * Description: FADC250 Firmware Tools Library
 *              Firmware specific functions.
 *
 * Author:
 *        Bryan Moffit
 *        JLab Data Acquisition Group
 *
 * Revision History:
 *      Revision 1.0   2011/07/18  moffit
 *         - Initial version for FADC250 v2
 *
 * SVN: $Rev$
 *
 */

#include <ctype.h>

#define        FA_FPGAID_MASK     0xFFFFF000
#define        FA_FPGAID_CTRL     0xf2501000
#define        FA_FPGAID_PROC     0xf2502000
#define        FA_FPGAID_REV_MASK 0x00000FFF

#define        MSC_MAX_SIZE    8000000
unsigned int   MCS_FPGAID = -1;            /* FPGA ID defined in the MCS file */
unsigned int   MSC_arraySize = 0;          /* Size of the array holding the firmware */
unsigned char  MSC_ARRAY[MSC_MAX_SIZE];    /* The array holding the firmware */
char          *MSC_filename_LX110 = "LX110_firmware.dat"; /* Default firmware for LX110 */
char          *MSC_filename_FX70T = "FX70T_firmware.dat"; /* Default firmware for FX70T */
int            MSC_loaded = 0;             /* 1(0) if firmware loaded (not loaded) */
VOIDFUNCPTR    faUpdateWatcherRoutine = NULL;
faUpdateWatcherArgs_t faUpdateWatcherArgs;

/*************************************************************
 * fadcFirmwareLoad
 *   - main routine to load up firmware for FADC with specific id
 *
 *  NOTE: Make call to
 *     fadcFirmwareSetFilename(...);
 *   if not using firmware from the default
 */

int
fadcFirmwareLoad(int id, int chip, int pFlag)
{
  faUpdateWatcherArgs_t updateArgs;
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(chip<0 || chip>2)
    {
      printf("%s: ERROR:  Invalid chip parameter %d\n",
	     __FUNCTION__,chip);
      return ERROR;
    }

  if(chip==2) /* Fix for discrepancy between Linux and vxWorks implementation */
    chip=FADC_FIRMWARE_LX110;

  /* Perform a hardware and software reset */
  FALOCK;
  vmeWrite32(&FAp[id]->reset, 0xFFFF);
  FAUNLOCK;
  taskDelay(60);

  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Check if ready \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  /* Check if FADC is Ready */
  if(fadcFirmwareTestReady(id, 60, pFlag) != OK)
    {
      printf("%s: ERROR: FADC %2d not ready after reset\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  /* Check if FADC has write access to SRAM from U90 JTAG DIP-switches */
  if(fadcFirmwareCheckSRAM(id) != OK)
    {
      printf("%s: ERROR: FADC %2d not configured to read/write PROM.\n\tCheck U90 DIP Switches.\n",
	     __FUNCTION__,id);
      updateArgs.step = FA_ARGS_SHOW_STRING;
      sprintf(updateArgs.title, "ERROR: FADC %2d FAILED PROM READ/WRITE TEST\n", id);
      fadcFirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  /* Data to SRAM */
  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading SRAM with data \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  fadcFirmwareDownloadConfigData(id);
  if(fadcFirmwareVerifyDownload(id) != OK)
    {
      printf("%s: ERROR: FADC %2d Failed data verification at SRAM\n",
	     __FUNCTION__,id);
      return ERROR;
    }


  /* SRAM TO PROM */
  taskDelay(1);
  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading PROM with SRAM data \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  FALOCK;
  if(chip==FADC_FIRMWARE_LX110)
    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_SRAM_TO_PROM1);
  else if(chip==FADC_FIRMWARE_FX70T)
    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_SRAM_TO_PROM2);
  FAUNLOCK;
  taskDelay(1);

  if(fadcFirmwareTestReady(id, 60000, pFlag) != OK) /* Wait til it's done */
    {
      printf("%s: ERROR: FADC %2d ready timeout SRAM -> PROM\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  /* PROM TO SRAM (For verification) */
  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading SRAM with PROM data \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  fadcFirmwareZeroSRAM(id);

  FALOCK;
  if(chip==FADC_FIRMWARE_LX110)
    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_PROM1_TO_SRAM);
  else if(chip==FADC_FIRMWARE_FX70T)
    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_PROM2_TO_SRAM);
  FAUNLOCK;
  taskDelay(1);

  if(fadcFirmwareTestReady(id, 60000, pFlag) != OK) /* Wait til it's done */
    {
      printf("%s: ERROR: FADC %2d ready timeout PROM -> SRAM\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  /* Compare SRAM to Data Array */
  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Verifying data \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  if(fadcFirmwareVerifyDownload(id) != OK)
    {
      printf("%s: ERROR: FADC %d PROM data not verified\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  /* PROM to FPGA (Reboot FPGA) */
  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Rebooting FPGA \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  FALOCK;
  if(chip==FADC_FIRMWARE_LX110)
    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_REBOOT_FPGA1);
  else if(chip==FADC_FIRMWARE_FX70T)
    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_REBOOT_FPGA2);
  FAUNLOCK;
  taskDelay(1);

  if(fadcFirmwareTestReady(id, 60000, pFlag) != OK) /* Wait til it's done */
    {
      printf("%s: ERROR: FADC %2d ready timeout PROM -> FPGA\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Done programming FADC %2d\n", id);
  fadcFirmwareUpdateWatcher(updateArgs);

  return OK;

}

static unsigned int passed[FA_MAX_BOARDS+1], stepfail[FA_MAX_BOARDS+1];

int
fadcFirmwarePassedMask()
{
  unsigned int retMask=0;
  int id, ifadc;

  for(ifadc=0; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(passed[id]==1)
        retMask |= (1<<id);
    }

  return retMask;
}

/*************************************************************
 * fadcFirmwareGLoad
 *   - load up firmware for all initialized modules
 *
 *  NOTE: Make call to
 *     fadcFirmwareSetFilename(...);
 *   if not using firmware from the default
 */
int
fadcFirmwareGLoad(int chip, int pFlag)
{
  int ifadc=0, id=0, step=0;
  faUpdateWatcherArgs_t updateArgs;

  /*   unsigned int passed[FA_MAX_BOARDS+1], stepfail[FA_MAX_BOARDS+1]; */

  if(chip<0 || chip>2)
    {
      printf("%s: ERROR:  Invalid chip parameter %d\n",
	     __FUNCTION__,chip);
      return ERROR;
    }

  if(chip==2) /* Fix for discrepancy between Linux and vxWorks implementation */
    chip=FADC_FIRMWARE_LX110;

  /* Perform a hardware and software reset */
  step=0;
  FALOCK;
  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if((id<=0) || (id>21) || (FAp[id] == NULL))
	{
	  printf("%s: ERROR : ADC in slot %d is not initialized \n",
		 __FUNCTION__,id);
	  passed[id] = 0;
	  stepfail[id] = step;
	}
      else
	{
	  passed[id] = 1;
	  vmeWrite32(&FAp[id]->reset, 0xFFFF);
	}
    }
  FAUNLOCK;
  taskDelay(60);

  /* Check if FADC is Ready */
  step=1;

  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Check if ready \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(fadcFirmwareTestReady(id, 60, pFlag) != OK)
	{
	  printf("%s: ERROR: FADC %2d not ready after reset\n",
		 __FUNCTION__,id);
	  passed[id] = 0;
	  stepfail[id] = step;
	}
    }

  /* Check if FADC has write access to SRAM from U90 JTAG DIP-switches */
  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(fadcFirmwareCheckSRAM(id) != OK)
	{
	  printf("%s: ERROR: FADC %2d not configured to read/write PROM.\n\tCheck U90 DIP Switches.\n",
		 __FUNCTION__,id);
	  passed[id] = 0;
	  stepfail[id] = step;
	  updateArgs.step = FA_ARGS_SHOW_STRING;
	  sprintf(updateArgs.title, "ERROR: FADC %2d FAILED PROM READ/WRITE TEST\n", id);
	  fadcFirmwareUpdateWatcher(updateArgs);
	}
    }


  /* Data to SRAM */
  step=2;

  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading SRAM with data \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];

      updateArgs.step = FA_ARGS_SHOW_ID;
      updateArgs.id   = id;
      fadcFirmwareUpdateWatcher(updateArgs);

      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  fadcFirmwareDownloadConfigData(id);

	  if(fadcFirmwareVerifyDownload(id) != OK)
	    {
	      printf("%s: ERROR: FADC %2d Failed data verification at SRAM\n",
		     __FUNCTION__,id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	  else
	    {
	      updateArgs.step = FA_ARGS_SHOW_DONE;
	      fadcFirmwareUpdateWatcher(updateArgs);
	    }
	}
    }

  /* SRAM TO PROM */
  step=3;
  taskDelay(1);

  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading PROM with SRAM data \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  FALOCK;
  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  if(chip==FADC_FIRMWARE_LX110)
	    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_SRAM_TO_PROM1);
	  else if(chip==FADC_FIRMWARE_FX70T)
	    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_SRAM_TO_PROM2);
	}
    }
  FAUNLOCK;
  taskDelay(1);

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  if(fadcFirmwareTestReady(id, 60000, pFlag) != OK) /* Wait til it's done */
	    {
	      printf("%s: ERROR: FADC %2d ready timeout SRAM -> PROM\n",
		     __FUNCTION__,id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	}
    }

  /* PROM TO SRAM (For verification) */
  step=4;

  updateArgs.step = FA_ARGS_SHOW_STRING;
  fadcFirmwareUpdateWatcher(updateArgs);
  sprintf(updateArgs.title,"Loading SRAM with PROM data \n");

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  fadcFirmwareZeroSRAM(id);
	  FALOCK;
	  if(chip==FADC_FIRMWARE_LX110)
	    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_PROM1_TO_SRAM);
	  else if(chip==FADC_FIRMWARE_FX70T)
	    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_PROM2_TO_SRAM);
	  FAUNLOCK;
	}
    }

  taskDelay(1);

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  if(fadcFirmwareTestReady(id, 60000, pFlag) != OK) /* Wait til it's done */
	    {
	      printf("%s: ERROR: FADC %2d ready timeout PROM -> SRAM\n",
		     __FUNCTION__,id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	}
    }

  /* Compare SRAM to Data Array */
  step=5;

  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Verifying data \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];

      updateArgs.step = FA_ARGS_SHOW_ID;
      updateArgs.id   = id;
      fadcFirmwareUpdateWatcher(updateArgs);

      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  if(fadcFirmwareVerifyDownload(id) != OK)
	    {
	      printf("%s: ERROR: FADC %d PROM data not verified\n",
		     __FUNCTION__,id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	  else
	    {
	      updateArgs.step = FA_ARGS_SHOW_DONE;
	      fadcFirmwareUpdateWatcher(updateArgs);
	    }
	}
    }

  /* PROM to FPGA (Reboot FPGA) */
  step=6;

  updateArgs.step = FA_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Rebooting FPGA \n");
  fadcFirmwareUpdateWatcher(updateArgs);

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  FALOCK;
	  if(chip==FADC_FIRMWARE_LX110)
	    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_REBOOT_FPGA1);
	  else if(chip==FADC_FIRMWARE_FX70T)
	    vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_REBOOT_FPGA2);
	  FAUNLOCK;
	}
    }
  taskDelay(1);

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  if(fadcFirmwareTestReady(id, 60000, pFlag) != OK) /* Wait til it's done */
	    {
	      printf("%s: ERROR: FADC %2d ready timeout PROM -> FPGA\n",
		     __FUNCTION__,id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	}
    }

  for(ifadc=0 ; ifadc<nfadc; ifadc++)
    {
      id = fadcID[ifadc];
      if(passed[id]) /* Skip the ones that have previously failed */
	{
	  updateArgs.step = FA_ARGS_SHOW_STRING;
	  sprintf(updateArgs.title, "Done programming FADC %2d\n", id);
	  fadcFirmwareUpdateWatcher(updateArgs);
	}
      else
	{
	  printf("%s: FAILED programming FADC %2d at step %d\n",
		 __FUNCTION__,id,stepfail[id]);
	}
    }

  return OK;

}

void
fadcFirmwareDownloadConfigData(int id)
{
  unsigned int ArraySize;
  unsigned int ByteCount, ByteIndex, ByteNumber;
  unsigned int Word32Bits;
#ifdef DEBUG
  unsigned int value;
#endif

  if(MSC_loaded != 1)
    {
      printf("%s: ERROR : Firmware was not loaded\n",
	     __FUNCTION__);
      return;
    }

  ArraySize = MSC_arraySize;
  ByteIndex = 0;

  /* write SRAM address register */
  /* start at 0 and increment address after write to mem1 data register */
  FALOCK;
  vmeWrite32(&FAp[id]->mem_adr, 0x80000000);
#ifdef DEBUG
  value = vmeRead32(&FAp[id]->mem_adr);
#endif
  FAUNLOCK;
#ifdef DEBUG
  printf("%s: FADC %2d memory address at start of writes = 0x%08x\n\n",
	 __FUNCTION__,id,value);
#endif
  taskDelay(1);			/* wait */

/*   printf("Download Config Data... \n"); */
  for (ByteCount = 0; ByteCount < ArraySize; ByteCount += 4)
    {
      Word32Bits = 0;
      for (ByteNumber = 0; ByteNumber < 4; ++ByteNumber)
	{
	  Word32Bits = (MSC_ARRAY[ByteIndex] << (8 * ByteNumber)) | Word32Bits;
	  ++ByteIndex;
	  if(ByteIndex>MSC_MAX_SIZE)
	    printf("**** TOO BIG! ****\n");
	}

      /* write 32-bit data word to  mem1 data register */
      FALOCK;
      vmeWrite32(&FAp[id]->mem1_data, Word32Bits);
      FAUNLOCK;
    }

#ifdef DEBUG
  FALOCK;
  value = vmeRead32(&FAp[id]->mem_adr);
  FAUNLOCK;
  printf("%s: FADC %2d memory address after write = 0x%08x\n\n",
	 __FUNCTION__,id,value);
#endif
  taskDelay(1);			/* wait */

}


int
fadcFirmwareVerifyDownload (int id)
{
  unsigned int ArraySize;
  unsigned int ByteCount, ByteIndex, ByteNumber;
  unsigned int ExpWord32Bits, RdWord32Bits;
  int ErrorCount=0, stopPrint=0;
#ifdef DEBUG
  unsigned int value;
#endif

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(MSC_loaded != 1)
    {
      printf("%s: ERROR : Firmware was not loaded\n",
	     __FUNCTION__);
      return ERROR;
    }

  ArraySize = MSC_arraySize;
  ByteIndex = 0;

  /* write SRAM address register */
  /* start at 0 and increment address after read from mem1 data register */
  FALOCK;
  vmeWrite32(&FAp[id]->mem_adr, 0x0000 | FA_MEM_ADR_INCR_MEM1);
#ifdef DEBUG
  value = vmeRead32(&FAp[id]->mem_adr);
#endif
  FAUNLOCK;
#ifdef DEBUG
  printf("%s: FADC %2d memory address at start of read = 0x%08x\n\n",
	 __FUNCTION__,id,value);
#endif
  taskDelay(1);			/* wait */

  for (ByteCount = 0; ByteCount < ArraySize; ByteCount += 4)
    {
      /* get expected value */
      ExpWord32Bits = 0;
      for (ByteNumber = 0; ByteNumber < 4; ++ByteNumber)
	{
	  ExpWord32Bits = (MSC_ARRAY[ByteIndex] << (8 * ByteNumber)) | ExpWord32Bits;
	  ++ByteIndex;
	}

      /* read 32-bit data word from mem1 data register */
      FALOCK;
      RdWord32Bits = (unsigned int)vmeRead32(&FAp[id]->mem1_data);
      FAUNLOCK;

#ifdef DEBUG
      if(ByteCount<40)
	printf("RdWord32Bits = 0x%08x\n",RdWord32Bits);
#endif

      /* test if read value = expected value */
      if (RdWord32Bits != ExpWord32Bits)
	{
	  ErrorCount++;
	  if(!stopPrint)
	    printf("%s: ERROR: FADC %2d ByteCount %8d  Expect %08X  Read %08X\n",
		   __FUNCTION__,id,
		   ByteCount, ExpWord32Bits, RdWord32Bits);
	  if( ErrorCount==2 )
	    {
	      printf("%s: Further errors for FADC %2d will not be displayed\n",
		     __FUNCTION__,id);
/* 	      getchar(); */
	      stopPrint=1;
	    }
/* 	  if( ErrorCount>1000 ) */
/* 	    stopPrint=1; */
	}
    }

#ifdef DEBUG
  FALOCK;
  value = vmeRead32(&FAp[id]->mem_adr);
  FAUNLOCK;
  printf("%s: memory address after read = 0x%08x\n\n",
	 __FUNCTION__,value);
#endif
  if(ErrorCount)
    printf("%s: ErrorCount = %d\n",__FUNCTION__,ErrorCount);
  taskDelay(1);			/* wait */

  if(ErrorCount)
    return ERROR;

  return OK;
}

int
fadcFirmwareTestReady(int id, int n_try, int pFlag)
{
  int ii;
  int result;
  faUpdateWatcherArgs_t updateArgs;
  unsigned int value = 0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  result = ERROR;

  updateArgs.step = FA_ARGS_SHOW_ID;
  updateArgs.id   = id;
  fadcFirmwareUpdateWatcher(updateArgs);

  for(ii = 0; ii < n_try; ii++)	/* poll for ready bit */
    {
      updateArgs.step = FA_ARGS_SHOW_PROGRESS;
      fadcFirmwareUpdateWatcher(updateArgs);

      taskDelay(1);		/* wait */
      FALOCK;
      value = vmeRead32(&FAp[id]->prom_reg1);
      FAUNLOCK;

      if( value == 0xFFFFFFFF)
	continue;

      if(value & FA_PROMREG1_READY)
	{
	  result = OK;
	  break;
	}
    }
  updateArgs.step = FA_ARGS_SHOW_DONE;
  fadcFirmwareUpdateWatcher(updateArgs);

  if(pFlag)
    {
      if( ii == n_try )		/* failed to detect ready asserted */
	printf("%s: FADC %2d NOT READY after %d wait cycles (1/60 sec)\n",
	       __FUNCTION__,id,n_try);
      else
	printf("%s: FADC %2d READY after %d wait cycles (1/60 sec)\n",
	       __FUNCTION__,id,(ii + 1));
    }

  return result;
}

int
fadcFirmwareZeroSRAM (int id)
{
  int ii, value=0, value_1=0, value_2=0;
  int ErrorCount=0, stopPrint=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  /* set address = 0; allow increment on mem2 access */
  FALOCK;
  vmeWrite32(&FAp[id]->mem_adr, 0x0000 | FA_MEM_ADR_INCR_MEM2);

  for( ii = 0; ii < 0x80000; ii++) 	/* write ZERO to entire memory */
    {
      vmeWrite32(&FAp[id]->mem1_data, 0);
      vmeWrite32(&FAp[id]->mem2_data, 0);
    }

  /* reset address = 0; allow increment on mem2 access */
  vmeWrite32(&FAp[id]->mem_adr, 0x0000 | FA_MEM_ADR_INCR_MEM2);

  FAUNLOCK;

  /* read and test expected memory data */
  for( ii = 0; ii < 0x80000; ii++)
    {
      FALOCK;
      value_1 = vmeRead32(&FAp[id]->mem1_data);
      value_2 = vmeRead32(&FAp[id]->mem2_data);
      FAUNLOCK;

      if( (value_1 != 0) || (value_2 != 0) )
	{
	  ErrorCount++;
	  FALOCK;
	  value = vmeRead32(&FAp[id]->mem_adr) & 0xFFFFF;
	  FAUNLOCK;
	  if(!stopPrint)
	    {
	      printf("%s: ERROR: FADC %2d  address = %8X    mem1 read = %8X    mem2 read = %8X\n",
		     __FUNCTION__,id,
		     value, value_1, value_2);
	      taskDelay(1);		/* wait */
	    }
	  if(ErrorCount==1)
	    {
	      printf("%s: Further errors for FADC %2d will not be displayed\n",
		     __FUNCTION__,id);
	      stopPrint=1;
	    }
	}
    }

  if(ErrorCount)
    return ERROR;

  return OK;
}

int
fadcFirmwareCheckSRAM (int id)
{
  int NonZeroCount=0;
  unsigned int ByteCount;
  unsigned int RdWord32Bits;
#ifdef DEBUG
  unsigned int value;
#endif
  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  fadcFirmwareZeroSRAM(id);

  FALOCK;
  vmeWrite32(&FAp[id]->prom_reg1,FA_PROMREG1_PROM2_TO_SRAM);
  FAUNLOCK;
  taskDelay(1);

  if(fadcFirmwareTestReady(id, 60000, 0) != OK) /* Wait til it's done */
    {
      printf("%s: ERROR: FADC %2d ready timeout PROM -> SRAM\n",
	     __FUNCTION__,id);
      return ERROR;
    }

  FALOCK;
  vmeWrite32(&FAp[id]->mem_adr, 0x0000 | FA_MEM_ADR_INCR_MEM1);
#ifdef DEBUG
  value = vmeRead32(&FAp[id]->mem_adr);
#endif
  FAUNLOCK;
#ifdef DEBUG
  printf("%s: FADC %2d memory address at start of read = 0x%08x\n\n",
	 __FUNCTION__,id,value);
#endif
  taskDelay(1);			/* wait */

  for (ByteCount = 0; ByteCount < MSC_arraySize; ByteCount += 4)
    {
      FALOCK;
      RdWord32Bits = (unsigned int)vmeRead32(&FAp[id]->mem1_data);
      FAUNLOCK;

      if (RdWord32Bits != 0)
	{
	  NonZeroCount++;
	  break;
	}
    }

  if(NonZeroCount == 0)
    {
      return ERROR;
    }

  return OK;
}

void
fadcFirmwareSetFilename(char *filename, int chip)
{
  if(chip == FADC_FIRMWARE_LX110)
    MSC_filename_LX110 = filename;
  else if (chip == FADC_FIRMWARE_FX70T)
    MSC_filename_FX70T = filename;

}


int
fadcFirmwareReadFile(char *filename)
{
  unsigned int arraySize;

/* #define DEBUGFILE */
#ifdef DEBUGFILE
  int ichar=0;
#endif
  FILE *arrayFile=NULL;
  arrayFile=fopen(filename,"r");

  if(arrayFile==NULL)
    {
      printf("%s: ERROR opening file (%s) for reading\n",
	     __FUNCTION__,filename);
      return ERROR;
    }

  /* First 32bits is the size of the array */
  fread(&arraySize,sizeof(unsigned int),1,arrayFile);

#ifdef VXWORKS
  /* Made this file in Linux... so byte swap it for VXWORKS */
  MSC_arraySize = LONGSWAP(arraySize);
#else
  MSC_arraySize = arraySize;
#endif

  if(MSC_arraySize>MSC_MAX_SIZE)
    {
      printf("%s: ERROR: Firmware size (%d) from %s greater than MAX allowed (%d)\n",
	     __FUNCTION__,MSC_arraySize,filename,MSC_MAX_SIZE);
      return ERROR;
    }


#ifdef DEBUGFILE
  printf("MSC_arraySize = %d\n",MSC_arraySize);
#endif

  fread(&MSC_ARRAY,MSC_arraySize,1,arrayFile);

  fclose(arrayFile);

#ifdef DEBUGFILE
  for(ichar=0; ichar<16*10; ichar++)
    {
      if((ichar%16) == 0)
	printf("\n");
      printf("0x%02x ",MSC_ARRAY[ichar]);
    }
  printf("\n\n");
#endif
  MSC_loaded = 1;

  printf("%s: Reading Firmware from %s\n",
	 __FUNCTION__,filename);

  return OK;
}

int
hex2num(char c)
{
  c = toupper(c);

  if(c > 'F')
    return 0;

  if(c >= 'A')
    return 10 + c - 'A';

  if((c > '9') || (c < '0') )
    return 0;

  return c - '0';
}

int
fadcFirmwareGetFpgaID(int pflag)
{
  if(pflag)
    {
      printf("%s: FPGA ID = 0x%x (%d)\n",
	     __FUNCTION__,MCS_FPGAID, MCS_FPGAID);
    }

  return MCS_FPGAID;
}

int
fadcFirmwareChipFromFpgaID(int pflag)
{
  int rval=0;
  unsigned int id;

  id = MCS_FPGAID & FA_FPGAID_MASK;

  switch(id)
    {
    case FA_FPGAID_PROC:
      rval = FADC_FIRMWARE_LX110;
      break;
    case FA_FPGAID_CTRL:
      rval = FADC_FIRMWARE_FX70T;
      break;
    case -1:
    default:
      rval = ERROR;
    }

  if(pflag)
    {
      if(rval!=ERROR)
	{
	  printf("%s: ID = 0x%08x FPGA Chip = %d\n",
		 __FUNCTION__,MCS_FPGAID,rval);
	}
      else
	{
	  printf("%s: ID (0x%08x) does not match any available FPGAs for this program\n",
		 __FUNCTION__,MCS_FPGAID);
	}
    }

  return rval;
}

int
fadcFirmwareRevFromFpgaID(int pflag)
{
  int rval=0;

  rval = MCS_FPGAID & FA_FPGAID_REV_MASK;

  if(pflag)
    {
      printf("%s: Rev = 0x%x\n",__FUNCTION__,rval);
    }
  return rval;
}

int
fadcFirmwareReadMcsFile(char *filename)
{
  FILE *mscFile=NULL;
  char ihexLine[200], *pData;
  int len=0, datalen=0, byte_shift;
  unsigned int nbytes=0, line=0, hiChar=0, loChar=0;
  unsigned int readMSC=0;
#ifdef DEBUGFILE
  int ichar, thisChar[0];
#endif

  mscFile = fopen(filename,"r");
  if(mscFile==NULL)
    {
      perror("fopen");
      printf("%s: ERROR opening file (%s) for reading\n",
	     __FUNCTION__,filename);
      return ERROR;
    }

  while(!feof(mscFile))
    {
      /* Get the current line */
      if(!fgets(ihexLine, sizeof(ihexLine), mscFile))
	break;

      /* Get the the length of this line */
      len = strlen(ihexLine);

      if(len >= 5)
	{
	  /* Check for the start code */
	  if(ihexLine[0] != ':')
	    {
	      printf("%s: ERROR parsing file at line %d\n",
		     __FUNCTION__,line);
	      return ERROR;
	    }

	  /* Get the byte count */
	  hiChar = hex2num(ihexLine[1]);
	  loChar = hex2num(ihexLine[2]);
	  datalen = (hiChar)<<4 | loChar;

	  if(strncmp("00",&ihexLine[7], 2) == 0) /* Data Record */
	    {
	      pData = &ihexLine[9]; /* point to the beginning of the data */
	      while(datalen--)
		{
		  hiChar = hex2num(*pData++);
		  loChar = hex2num(*pData++);
		  MSC_ARRAY[readMSC] =
		    ((hiChar)<<4) | (loChar);
		  if(readMSC>=MSC_MAX_SIZE)
		    {
		      printf("%s: ERROR: TOO BIG!\n",__FUNCTION__);
		      return ERROR;
		    }
		  readMSC++;
		  nbytes++;
		}
	    }
	  else if(strncmp("01",&ihexLine[7], 2) == 0) /* End of File, contains FPGA ID */
	    {
	      byte_shift=24;
	      MCS_FPGAID=0;
	      pData = &ihexLine[9]; /* point to the beginning of the data */
	      while(datalen--)
		{
		  if(byte_shift<0)
		    {
		      printf("%s: ERROR: FPGA ID too large!\n",__FUNCTION__);
		      return ERROR;
		    }
		  hiChar = hex2num(*pData++);
		  loChar = hex2num(*pData++);

		  MCS_FPGAID |= ((hiChar<<4) | (loChar))<<byte_shift;
#ifdef DEBUGFILE
		  printf("%2d: MCS_FPGAID = 0x%08x\n",datalen,MCS_FPGAID);
#endif
		  byte_shift -= 8;
		}
	    }
	}
      line++;
    }

  MSC_arraySize = readMSC;

#ifdef DEBUGFILE
  printf("MSC_arraySize = %d\n",MSC_arraySize);

  for(ichar=0; ichar<16*10; ichar++)
    {
      if((ichar%16) == 0)
	printf("\n");
      printf("0x%02x ",MSC_ARRAY[ichar]);
    }
  printf("\n\n");
#endif
  MSC_loaded = 1;

  fclose(mscFile);
  return OK;
}

int
fadcFirmwareAttachUpdateWatcher(VOIDFUNCPTR routine, faUpdateWatcherArgs_t arg)
{

  if(routine)
    {
      faUpdateWatcherRoutine = routine;
      faUpdateWatcherArgs = arg;
    }
  else
    {
      faUpdateWatcherRoutine = NULL;
      memset(&faUpdateWatcherArgs, 0, sizeof(faUpdateWatcherArgs_t));
    }

  return OK;
}

void
fadcFirmwareUpdateWatcher(faUpdateWatcherArgs_t arg)
{
  faUpdateWatcherArgs_t rArg;
  static int step1_ticks = 0;

  if((arg.step >= FA_ARGS_SHOW_ID) && (arg.step < FA_ARGS_LAST))
    rArg = arg;
  else
    rArg = faUpdateWatcherArgs;

  if(faUpdateWatcherRoutine != NULL)
    {
      (*faUpdateWatcherRoutine) (rArg);
    }
  else
    {
      switch(rArg.step)
	{
	case FA_ARGS_SHOW_ID:
	  step1_ticks=0;
	  printf("%2d: ", arg.id);
	  fflush(stdout);
	  break;

	case FA_ARGS_SHOW_PROGRESS:
	  step1_ticks++;
	  if((step1_ticks % 180) == 0)
	    {
	      printf("+");
	      fflush(stdout);
	    }
	  break;

	case FA_ARGS_SHOW_DONE:
	  step1_ticks=0;
	  printf(" Done\n");
	  fflush(stdout);
	  break;

	case FA_ARGS_SHOW_STRING:
	  printf("%s", arg.title);
	  fflush(stdout);
	  break;

	}

    }
}
