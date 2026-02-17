#ifndef __FADC250CONFIG_H
#define __FADC250CONFIG_H

#ifndef FADC250_CONFIG_GET_ENV
#define FADC250_CONFIG_GET_ENV "FADC250_PARAMS"
#endif

/****************************************************************************
 *
 *  fadc250Config.h  -  configuration library header file for fADC250 board
 *
 *  SP, 07-Nov-2013
 *
 */


#define ADD_TO_STRING				\
  len1 = strlen(str);				\
  len2 = strlen(sss);				\
  if((len1+len2) < length) strcat(str,sss);	\
  else return(len1)

#define CLOSE_STRING				\
  len1 = strlen(str);				\
  return(len1)

#define FNLEN     128       /* length of config. file name */
#define STRLEN    250       /* length of str_tmp */
#define ROCLEN     80       /* length of ROC_name */
#define NCHAN      16


/** FADC250 configuration parameters **/
typedef struct {
  int f_rev;
  int b_rev;
  int b_ID;

  int          mode;
  int          compression;
  int          vxsReadout;
  unsigned int winOffset;
  unsigned int winWidth;
  unsigned int nsb;
  unsigned int nsa;
  unsigned int npeak;

  unsigned int chDisMask;
  unsigned int trigMask;
  unsigned int trigWidth;
  unsigned int trigMinTOT;
  unsigned int trigMinMult;
  unsigned int thr[NCHAN];
  unsigned int dac[NCHAN];
  unsigned int delay[NCHAN];
  float        ped[NCHAN];
  unsigned int thrIgnoreMask;
  unsigned int invertMask;
  unsigned int playbackDisableMask;
  float gain[NCHAN];
  unsigned int trigMode[NCHAN];

} FADC250_CONF;


/* functions */

#ifdef	__cplusplus
extern "C" {
#endif

void fadc250SetExpid(char *string);
void fadc250GetParamsForOffline(float ped[6][22][16], int tet[6][22][16], float gain[6][22][16], int nsa[6][22], int nsb[6][22]);
void fadc250Sethost(char *host);
void fadc250InitGlobals();
int fadc250ReadConfigFile(char *filename);
int fadc250DownloadAll();
int fadc250Config(char *fname);
void fadc250Mon(int slot);
int fadc250UploadAll(char *string, int length);

#ifdef	__cplusplus
}
#endif


#endif // __FADC250CONFIG_H
