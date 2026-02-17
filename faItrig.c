/*********************************************
 *
 *  FADC Internal Trigger FADC Configuration and Control
 *  Routines.
 */

#ifndef EIEIO
#define EIEIO
#endif

unsigned int
faItrigStatus(int id, int sFlag)
{
  unsigned int status, config, twidth, wMask, wWidth, cMask, sum_th;
  unsigned int itrigCnt, trigOut;
  int vers, disabled, mode;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      printf("faItrigStatus: ERROR : FADC in slot %d is not initialized \n",id);
      return(ERROR);
    }

  /* Express Time in ns - 4ns/clk  */
  FALOCK;
  status   =  vmeRead32(&FAp[id]->hitsum_status)&0xffff;
  config   =  vmeRead32(&FAp[id]->hitsum_cfg)&0xffff;
  twidth   =  (vmeRead32(&FAp[id]->hitsum_trig_width)&0xffff)*FA_ADC_NS_PER_CLK;
  wMask    =  vmeRead32(&FAp[id]->hitsum_window_bits)&0xffff;
  wWidth   =  (vmeRead32(&FAp[id]->hitsum_window_width)&0xffff)*FA_ADC_NS_PER_CLK;
  cMask    =  vmeRead32(&FAp[id]->hitsum_coin_bits)&0xffff;
  sum_th   =  vmeRead32(&FAp[id]->hitsum_sum_thresh)&0xffff;
  itrigCnt =  vmeRead32(&FAp[id]->internal_trig_scal);
  trigOut  =  vmeRead32(&FAp[id]->ctrl1)&FA_ITRIG_OUT_MASK;
  FAUNLOCK;

  vers     = status&FA_ITRIG_VERSION_MASK;
  mode     = config&FA_ITRIG_MODE_MASK;
  if(mode == FA_ITRIG_SUM_MODE) /* If Sum mode then Live trigger always enabled */
    disabled = 0;
  else
    disabled = config&FA_ITRIG_ENABLE_MASK;


  printf("\n FADC Internal Trigger (HITSUM) Configuration: \n");
  printf("  (Mode: 0-Table 1-Coin 2-Window 4-Sum)\n");
  if(disabled)
    printf("     Hitsum Status      = 0x%04x    Config = 0x%04x   (Mode = %d - Disabled)\n",status, config, mode);
  else
    printf("     Hitsum Status      = 0x%04x    Config = 0x%04x   (Mode = %d - Enabled)\n",status, config, mode);
  printf("     Window  Input Mask = 0x%04x    Width = %5d ns\n", wMask, wWidth);
  printf("     Coin    Input Mask = 0x%04x \n",cMask);
  printf("     Sum Mode Threshold = %d\n",sum_th);
  if(trigOut == FA_ITRIG_OUT_FP)
    printf("     Trigger Out  Width =  %5d ns (Front panel output)\n", twidth);
  else
    printf("     Trigger Out  Width =  %5d ns (Output disabled)\n", twidth);
  printf("     Internal Triggers (Live) = %d\n",itrigCnt);

  return(config);
}

/************************************************************
 *
 *  Setup Internal Triggering
 *   
 *   Four Modes of Operation (tmode)
 *     0) Table Mode
 *     1) Coincidence Mode
 *     2) Window Mode
 *     3) INVALID
 *     4) Sum Mode
 *
 *   wMask     = Mask of 16 channels to be enabled for Window Mode
 *   wWidth    = Width of trigger window before latching (in clocks)
 *   cMask     = Mask of 16 channels to be enabled for Coincidence Mode
 *   sumThresh = 10-12 bit threshold for Sum trigger to be latched 
 *   tTable    = pointer to trigger table (65536 values) to be loaded
 */
int
faItrigSetMode(int id, int tmode, unsigned int wWidth, unsigned int wMask,
	       unsigned int cMask, unsigned int sumThresh, unsigned int *tTable)
{
  int ii;
  unsigned int config, stat, wTime;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      printf("faItrigSetMode: ERROR : FADC in slot %d is not initialized \n",id);
      return(ERROR);
    }

  /* Make sure we are not enabled or running */
  FALOCK;
  config = vmeRead32(&FAp[id]->hitsum_cfg)&FA_ITRIG_CONFIG_MASK;
  FAUNLOCK;
  if((config&FA_ITRIG_ENABLE_MASK) == 0) 
    {
      printf("faItrigSetMode: ERROR: Internal triggers are enabled - Disable first\n");
      return(ERROR);
    }

  if((tmode==FA_ITRIG_UNDEF_MODE)||(tmode>FA_ITRIG_SUM_MODE)) 
    {
      printf("faItrigSetMode: ERROR: Trigger mode (%d) out of range (tmode = 0-4)\n",tmode);
      return(ERROR);
    }

  /* Check if we need to load a trigger table */
  if(tTable != NULL) 
    {
      printf("faItrigSetMode: Loading trigger table from address 0x%lx \n",(unsigned long) tTable);
      FALOCK;
      vmeWrite32(&FAp[id]->s_adr, FA_SADR_AUTO_INCREMENT);
      vmeWrite32(&FAp[id]->hitsum_pattern, 0);  /* Make sure address 0 is not a valid trigger */
      for(ii=1;ii<=0xffff;ii++) 
	{
	  if(tTable[ii]) 
	    vmeWrite32(&FAp[id]->hitsum_pattern, 1);
	  else
	    vmeWrite32(&FAp[id]->hitsum_pattern, 0);
	}
      FAUNLOCK;
    }

  switch(tmode) 
    {
    case FA_ITRIG_SUM_MODE:
      /* Load Sum Threshhold if in range */
      FALOCK;
      if((sumThresh > 0)&&(sumThresh <= 0xffff)) 
	{
	  vmeWrite32(&FAp[id]->hitsum_sum_thresh, sumThresh);
	}
      else
	{
	  printf("faItrigSetMode: ERROR: Sum Threshold out of range (0<st<=0xffff)\n");
	  FAUNLOCK;
	  return(ERROR);
	}
      stat = (config&~FA_ITRIG_MODE_MASK) | FA_ITRIG_SUM_MODE;
      vmeWrite32(&FAp[id]->hitsum_cfg, stat);
      FAUNLOCK;
      printf("faItrigSetMode: Configure for SUM Mode (Threshold = 0x%x)\n",sumThresh);
      break;

    case FA_ITRIG_COIN_MODE:
      /* Set Coincidence Input Channels */
      FALOCK;
      if((cMask > 0)&&(cMask <= 0xffff)) 
	{
	  vmeWrite32(&FAp[id]->hitsum_coin_bits, cMask);
	}
      else
	{
	  printf("faItrigSetMode: ERROR: Coincidence channel mask out of range (0<cc<=0xffff)\n");
	  FAUNLOCK;
	  return(ERROR);
	}
      stat = (config&~FA_ITRIG_MODE_MASK) | FA_ITRIG_COIN_MODE;
      vmeWrite32(&FAp[id]->hitsum_cfg, stat);
      FAUNLOCK;
      printf("faItrigSetMode: Configure for COINCIDENCE Mode (channel mask = 0x%x)\n",cMask);
      break;

    case FA_ITRIG_WINDOW_MODE:
      /* Set Trigger Window width and channel mask */
      FALOCK;
      if((wMask > 0)&&(wMask <= 0xffff)) 
	{
	  vmeWrite32(&FAp[id]->hitsum_window_bits, wMask);
	}
      else
	{
	  printf("faItrigSetMode: ERROR: Trigger Window channel mask out of range (0<wc<=0xffff)\n");
	  FAUNLOCK;
	  return(ERROR);
	}
      if((wWidth > 0)&&(wWidth <= FA_ITRIG_MAX_WIDTH)) 
	{
	  vmeWrite32(&FAp[id]->hitsum_window_width, wWidth);
	  wTime = 4*wWidth;
	}
      else
	{
	  printf("faItrigSetMode: ERROR: Trigger Window width out of range (0<ww<=0x200)\n");
	  FAUNLOCK;
	  return(ERROR);
	}
      stat = (config&~FA_ITRIG_MODE_MASK) | FA_ITRIG_WINDOW_MODE;
      vmeWrite32(&FAp[id]->hitsum_cfg, stat);
      FAUNLOCK;
      printf("faItrigSetMode: Configure for Trigger WINDOW Mode (channel mask = 0x%x, width = %d ns)\n",
	     wMask,wTime);
      break;

    case FA_ITRIG_TABLE_MODE:
      FALOCK;
      stat = (config&~FA_ITRIG_MODE_MASK) | FA_ITRIG_TABLE_MODE;
      vmeWrite32(&FAp[id]->hitsum_cfg, stat);
      FAUNLOCK;
      printf("faItrigSetMode: Configure for Trigger TABLE Mode\n");
    }
  
  return(OK);
}

/************************************************************
 *
 *  Setup Internal Trigger Table 
 *    16 input channels can be latched to create a 16 bit
 *  lookup address (0x0001 - 0xffff) in memory. The value 0 or 1
 *  at that memory address determines if a trigger pulse will be
 *  generated (this is for Window or Table mode only)
 *
 *   table = pointer to an array of 65536 values (1 or 0) that
 *           will define a valid trigger or not.
 *      (if = NULL, then the default table is loaded - all
 *       input combinations will generate a trigger)
 */
int
faItrigInitTable(int id, unsigned int *table)
{
  int ii;
  unsigned int config;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      printf("faItrigInitTable: ERROR : FADC in slot %d is not initialized \n",id);
      return(ERROR);
    }

  /* Check and make sure we are not running */
  FALOCK;
  config = vmeRead32(&FAp[id]->hitsum_cfg);
  if((config&FA_ITRIG_ENABLE_MASK) !=  FA_ITRIG_DISABLED) 
    {
      printf("faItrigInitTable: ERROR: Cannot update Trigger Table while trigger is Enabled\n");
      FAUNLOCK;
      return(ERROR);
    }


  if(table == NULL) 
    { 
      /* Use default Initialization - all combinations of inputs will be a valid trigger */
      vmeWrite32(&FAp[id]->s_adr, FA_SADR_AUTO_INCREMENT);
      vmeWrite32(&FAp[id]->hitsum_pattern, 0);  /* Make sure address 0 is not a valid trigger */
      for(ii=1;ii<=0xffff;ii++) 
	{
	  vmeWrite32(&FAp[id]->hitsum_pattern, 1);
	}
      
    }
  else
    {  /* Load specified table into hitsum FPGA */
      
      vmeWrite32(&FAp[id]->s_adr, FA_SADR_AUTO_INCREMENT);
      vmeWrite32(&FAp[id]->hitsum_pattern, 0);  /* Make sure address 0 is not a valid trigger */
      for(ii=1;ii<=0xffff;ii++) 
	{
	  if(table[ii]) 
	    vmeWrite32(&FAp[id]->hitsum_pattern, 1);
	  else
	    vmeWrite32(&FAp[id]->hitsum_pattern, 0);
	}
      
    }
  FAUNLOCK;
  
  return(OK);
}




int
faItrigSetHBwidth(int id, unsigned short hbWidth, unsigned short hbMask)
{
  int ii;
  unsigned int config, hbval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      printf("faItrigSetHBwidth: ERROR : FADC in slot %d is not initialized \n",id);
      return(ERROR);
    }

  /* Check and make sure we are not running */
  FAUNLOCK;
  config = vmeRead32(&FAp[id]->hitsum_cfg);
  if((config&FA_ITRIG_ENABLE_MASK) !=  FA_ITRIG_DISABLED) 
    {
      printf("faItrigSetHBwidth: ERROR: Cannot set HB widths while trigger is Enabled\n");
      FAUNLOCK;
      return(ERROR);
    }
  
  if(hbWidth>FA_ITRIG_MAX_HB_WIDTH) hbWidth = FA_ITRIG_MAX_HB_WIDTH;
  if(hbMask==0)   hbMask  = 0xffff;  /* Set all Channels */
  
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++) 
    {
      if((1<<ii)&hbMask) 
	{
	  vmeWrite32(&FAp[id]->s_adr, ii);                  /* Set Channel to Read/Write*/
	  hbval  =  vmeRead32(&FAp[id]->hitsum_hit_info)&~FA_ITRIG_HB_WIDTH_MASK;
	  hbval  =  hbval | hbWidth;
	  vmeWrite32(&FAp[id]->hitsum_hit_info, hbval);  /* Set Value */
	}
    }
  FAUNLOCK;
  
  return(OK);
}

unsigned int
faItrigGetHBwidth(int id, unsigned int chan)
{
  unsigned int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      logMsg("faItrigGetHBwidth: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0xffffffff);
    }
  
  if(chan >= FA_MAX_ADC_CHANNELS) 
    {
      logMsg("faItrigGetHBwidth: ERROR : Channel # out of range (0-15)\n",0,0,0,0,0,0);
      return(0xffffffff);
    }
  
  FALOCK;
  vmeWrite32(&FAp[id]->s_adr, chan);             /* Set Channel */
  EIEIO;    
  rval = vmeRead32(&FAp[id]->hitsum_hit_info)&FA_ITRIG_HB_WIDTH_MASK;  /* Get Value */
  FAUNLOCK;
  
  return(rval);
}

int
faItrigSetHBdelay(int id, unsigned short hbDelay, unsigned short hbMask)
{
  int ii;
  unsigned int config, hbval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      printf("faItrigSetHBdelay: ERROR : FADC in slot %d is not initialized \n",id);
      return(ERROR);
    }
  
  /* Check and make sure we are not running */
  FALOCK;
  config = vmeRead32(&FAp[id]->hitsum_cfg);
  if((config&FA_ITRIG_ENABLE_MASK) !=  FA_ITRIG_DISABLED) 
    {
      printf("faItrigSetHBdelay: ERROR: Cannot set HB delays while trigger is Enabled\n");
      FAUNLOCK;
      return(ERROR);
    }
  
  
  if(hbDelay>FA_ITRIG_MAX_HB_DELAY) hbDelay = FA_ITRIG_MAX_HB_DELAY;
  if(hbMask==0)   hbMask  = 0xffff;  /* Set all Channels */
  
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++) 
    {
      if((1<<ii)&hbMask) 
	{
	  vmeWrite32(&FAp[id]->s_adr, ii);                  /* Set Channel */
	  hbval  =  vmeRead32(&FAp[id]->hitsum_hit_info)&~FA_ITRIG_HB_DELAY_MASK;
	  hbval |=  (hbDelay<<8);
	  vmeWrite32(&FAp[id]->hitsum_hit_info, hbval);  /* Set Value */
	}
    }
  FAUNLOCK;
  
  return(OK);
}

unsigned int
faItrigGetHBdelay(int id, unsigned int chan)
{
  unsigned int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      logMsg("faItrigGetHBdelay: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0xffffffff);
    }
  
  if(chan>15) 
    {
      logMsg("faItrigGetHBdelay: ERROR : Channel # out of range (0-15)\n",0,0,0,0,0,0);
      return(0xffffffff);
    }
     
  FALOCK;
  vmeWrite32(&FAp[id]->s_adr, chan);             /* Set Channel */
  EIEIO;    
  rval = (vmeRead32(&FAp[id]->hitsum_hit_info)&FA_ITRIG_HB_DELAY_MASK)>>8;  /* Get Value */
  FAUNLOCK;

  return(rval);
}


void
faItrigPrintHBinfo(int id)
{
  int ii;
  unsigned int hbval[16], wval, dval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      printf("faItrigPrintHBinfo: ERROR : FADC in slot %d is not initialized \n",id);
      return;
    }
  
  FALOCK;
  vmeWrite32(&FAp[id]->s_adr, ii);
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++) 
    {
      vmeWrite32(&FAp[id]->s_adr, ii);
      hbval[ii] = vmeRead32(&FAp[id]->hitsum_hit_info)&FA_ITRIG_HB_INFO_MASK;  /* Get Values */
    }
  FAUNLOCK;
  
  printf(" HitBit (width,delay) in nsec for FADC Inputs in slot %d:",id);
  for(ii=0;ii<FA_MAX_ADC_CHANNELS;ii++) 
    {
      wval = ((hbval[ii]&FA_ITRIG_HB_WIDTH_MASK)+1)*FA_ADC_NS_PER_CLK;
      dval = (((hbval[ii]&FA_ITRIG_HB_DELAY_MASK)>>8)+7)*FA_ADC_NS_PER_CLK;
      if((ii%4)==0) printf("\n");
      printf("Chan %2d: %4d,%3d  ",(ii+1),wval,dval);
    }
  printf("\n");
  
}

unsigned int
faItrigSetOutWidth(int id, unsigned short itrigWidth)
{
  unsigned int retval=0;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      logMsg("faItrigSetOutWidth: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(0xffffffff);
    }
  
  if(itrigWidth>FA_ITRIG_MAX_WIDTH) itrigWidth = FA_ITRIG_MAX_WIDTH;
  
  FALOCK;
  if(itrigWidth)
    vmeWrite32(&FAp[id]->hitsum_trig_width, itrigWidth);
  
  EIEIO;
  retval = vmeRead32(&FAp[id]->hitsum_trig_width)&0xffff;
  FAUNLOCK;
  
  return(retval);
}

void
faItrigEnable(int id, int eflag)
{
  unsigned int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      logMsg("faItrigEnable: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }
  
  FALOCK;
  rval = vmeRead32(&FAp[id]->hitsum_cfg);
  rval &= ~(FA_ITRIG_DISABLED);
  
  vmeWrite32(&FAp[id]->hitsum_cfg, rval);

  if(eflag) 
    {  /* Enable Live trigger to Front Panel Output */
      vmeWrite32(&FAp[id]->ctrl1,  vmeRead32(&FAp[id]->ctrl1) 
		 | (FA_ENABLE_LIVE_TRIG_OUT | FA_ENABLE_TRIG_OUT_FP));
    }
  FAUNLOCK;
  
}

void
faItrigDisable(int id, int dflag)
{
  unsigned int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      logMsg("faItrigDisable: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }
  
  FALOCK;
  rval = vmeRead32(&FAp[id]->hitsum_cfg);
  rval |= FA_ITRIG_DISABLED;
  
  vmeWrite32(&FAp[id]->hitsum_cfg, rval);
  
  if(dflag) 
    {  /* Disable Live trigger to Front Panel Output */
      rval = vmeRead32(&FAp[id]->ctrl1);
      rval &= ~(FA_ENABLE_LIVE_TRIG_OUT | FA_ENABLE_TRIG_OUT_FP);
      vmeWrite32(&FAp[id]->ctrl1, rval);
    }
  FAUNLOCK;
  
}


int
faItrigGetTableVal(int id, unsigned short pMask)
{
  int rval;

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      logMsg("faItrigGetTableVal: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return(ERROR);
    }
  
  FALOCK;
  vmeWrite32(&FAp[id]->s_adr, pMask);
  EIEIO; /* Make sure write comes before read */
  rval = vmeRead32(&FAp[id]->hitsum_pattern)&0x1;
  FAUNLOCK;

  return(rval);
}

void
faItrigSetTableVal(int id, unsigned short tval, unsigned short pMask)
{

  if(id==0) id=fadcID[0];

  if((id<=0) || (id>21) || (FAp[id] == NULL)) 
    {
      logMsg("faItrigSetTableVal: ERROR : FADC in slot %d is not initialized \n",id,0,0,0,0,0);
      return;
    }
  
  FALOCK;
  vmeWrite32(&FAp[id]->s_adr, pMask);
  if(tval)
    vmeWrite32(&FAp[id]->hitsum_pattern, 1);
  else
    vmeWrite32(&FAp[id]->hitsum_pattern, 0);
  FAUNLOCK;
  
}

