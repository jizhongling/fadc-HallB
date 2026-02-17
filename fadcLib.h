/******************************************************************************
*
*  fadcLib.h  - Driver library header file for readout of the JLAB 250MHz FLASH
*                ADC using a VxWorks 5.5 or later based single board
*                computer.
*
*  Author: David Abbott
*          Jefferson Lab Data Acquisition Group
*          June 2007
*
*  Revision  1.0 - Initial Revision
*                    - Supports up to 20 FADC boards in a Crate
*                    - Programmed I/O and Block reads
*
*  Revision  1.1 - Updated for new firmware (0x0111 or later)
*                  Supports FADC Signal Distribution Module
*
*  Revision  1.2 - Updated to support Internal (HITSUM) triggering
*                  and Moller firmware revisions of the FADC
*
*  Revision  2.0 - Updated to support FADC V2.
*
*/

#ifndef __FADCLIB__
#define __FADCLIB__

#define CLAS12

#define FA_BOARD_ID       0xfadc0000
#define FA_MAX_BOARDS             20
#define FA_MAX_ADC_CHANNELS       16
#define FA_MAX_DATA_PER_CHANNEL  251
#define FA_MAX_A32_MEM      0x800000   /* 8 Meg */
#define FA_MAX_A32MB_SIZE   0x800000  /*  8 MB */
#define FA_VME_INT_LEVEL           3
#define FA_VME_INT_VEC          0xFA

#define FA_SUPPORTED_CTRL_FIRMWARE 0xB3,0xBA
#define FA_SUPPORTED_CTRL_FIRMWARE_NUMBER 2
#define FA_SUPPORTED_PROC_FIRMWARE 0x1B06
#define FA_SUPPORTED_PROC_FIRMWARE_NUMBER 1


struct fadc_struct
{
  /* 0x0000 */ volatile unsigned int version;
  /* 0x0004 */ volatile unsigned int csr;
  /* 0x0008 */ volatile unsigned int ctrl1;
  /* 0x000C */ volatile unsigned int ctrl2;
  /* 0x0010 */ volatile unsigned int blk_level;
  /* 0x0014 */ volatile unsigned int intr;
  /* 0x0018 */ volatile unsigned int adr32;
  /* 0x001C */ volatile unsigned int adr_mb;
  /* 0x0020 */ volatile unsigned int s_adr;
  /* 0x0024 */ volatile unsigned int delay;
  /* 0x0028 */ volatile unsigned int itrig_cfg;
  /* 0x002C */ volatile unsigned int reset;
  /* 0x0030 */ volatile unsigned int trig_scal;
  /* 0x0034 */ volatile unsigned int ev_count;
  /* 0x0038 */ volatile unsigned int blk_count;
  /* 0x003C */ volatile unsigned int blk_fifo_count;
  /* 0x0040 */ volatile unsigned int blk_wrdcnt_fifo;
  /* 0x0044 */ volatile unsigned int internal_trig_scal;
  /* 0x0048 */ volatile unsigned int ram_word_count;
  /* 0x004C */ volatile unsigned int dataflow_status;
  /* 0x0050 */ volatile unsigned short dac[16];
  /* 0x0070 */ volatile unsigned int status[4];
  /* 0x0080 */ volatile unsigned int aux;
  /* 0x0084 */ volatile unsigned int trigger_control;
  /* 0x0088 */ volatile unsigned int trig21_delay;
  /* 0x008C */ volatile unsigned int mem_adr;
  /* 0x0090 */ volatile unsigned int mem1_data;
  /* 0x0094 */ volatile unsigned int mem2_data;
  /* 0x0098 */ volatile unsigned int prom_reg1;
  /* 0x009C */ volatile unsigned int prom_reg2;
  /* 0x00A0 */ volatile unsigned int berr_module_scal;
  /* 0x00A4 */ volatile unsigned int berr_crate_scal;
  /* 0x00A8 */ volatile unsigned int proc_words_scal;
  /* 0x00AC */ volatile unsigned int aux_scal2;
  /* 0x00B0 */ volatile unsigned int header_scal;
  /* 0x00B4 */ volatile unsigned int trig2_scal;
  /* 0x00B8 */ volatile unsigned int trailer_scal;
  /* 0x00BC */ volatile unsigned int syncreset_scal;
  /* 0x00C0 */ volatile unsigned int busy_level;
  /* 0x00C4 */ volatile unsigned int gen_evt_header;
  /* 0x00C8 */ volatile unsigned int gen_evt_data;
  /* 0x00CC */ volatile unsigned int gen_evt_trailer;
  /* 0x00D0 */ volatile unsigned int mgt_status;
  /* 0x00D4 */ volatile unsigned int mgt_ctrl;
  /* 0x00D8 */ volatile unsigned int reserved_ctrl[2];
  /* 0x00E0 */ volatile unsigned int scaler_ctrl;
  /* 0x00E4 */ volatile unsigned int serial_number[3];
  /* 0x00F0 */ volatile unsigned int scaler_interval;
  /* 0x00F4 */ volatile unsigned int spare_ctrl[(0xFC-0xF4)>>2];
  /* 0x00FC */ volatile unsigned int system_monitor;

  /* 0x0100 */ volatile unsigned int adc_status[3];
  /* 0x010C */ volatile unsigned int adc_config[4];
  /* 0x011C */ volatile unsigned int adc_ptw;
  /* 0x0120 */ volatile unsigned int adc_pl;
  /* 0x0124 */ volatile unsigned int adc_nsb;
  /* 0x0128 */ volatile unsigned int adc_nsa;
  /* 0x012C */ volatile unsigned short adc_thres[16];
  /* 0x014C */ volatile unsigned int ptw_last_adr;
  /* 0x0150 */ volatile unsigned int ptw_max_buf;
  /* 0x0154 */ volatile unsigned int adc_test_data;

#ifdef CLAS12
  /* 0x0158 */ volatile unsigned short adc_pedestal[16];
  /* 0x0178 */ volatile unsigned short adc_delay[16];
  /* 0x0198 */ volatile unsigned int adc_gain[16];
  /* 0x01d8 */ volatile unsigned int hitbit_trig_mask;
  /* 0x01dc */ volatile unsigned int hitbit_trig_width;
  /* 0x01e0 */ volatile unsigned int hitbit_cfg;
  /* 0x01e4 */ volatile unsigned int spare_adc[(0x200-0x1e4)>>2];

  /* 0x0200 */ volatile unsigned short la_ctrl;
  /* 0x0202 */ volatile unsigned short la_status;
  /* 0x0204 */ volatile unsigned short adc_scaler_ctrl;
  /* 0x0206 */ volatile unsigned short spare_adc0[(0x220-0x206)>>1];
  /* 0x0220 */ volatile unsigned short la_cmp_mode0[16];
  /* 0x0240 */ volatile unsigned short la_cmp_thr0[16];
  /* 0x0260 */ volatile unsigned short adc_accumulator0[16];
  /* 0x0280 */ volatile unsigned short adc_accumulator1[16];
  /* 0x02A0 */ volatile unsigned short adc_accumulator2[16];
  /* 0x02C0 */ volatile unsigned short la_data[13];
  /* 0x02DA */ volatile unsigned short spare_adc1[(0x300-0x2DA)>>1];
#else
  /* 0x0158 */ volatile unsigned int adc_pedestal[16];
  /* 0x0198 */ volatile unsigned int spare_adc[(0x200-0x198)>>2];

  /* 0x0200 */ volatile unsigned int hitsum_status;
  /* 0x0204 */ volatile unsigned int hitsum_cfg;
  /* 0x0208 */ volatile unsigned int hitsum_hit_width;
  /* 0x020C */ volatile unsigned int hitsum_trig_delay;
  /* 0x0210 */ volatile unsigned int hitsum_trig_width;
  /* 0x0214 */ volatile unsigned int hitsum_window_bits;
  /* 0x0218 */ volatile unsigned int hitsum_window_width;
  /* 0x021C */ volatile unsigned int hitsum_coin_bits;
  /* 0x0220 */ volatile unsigned int hitsum_pattern;
  /* 0x0224 */ volatile unsigned int hitsum_fifo;
  /* 0x0228 */ volatile unsigned int hitsum_sum_thresh;
  /* 0x022C */ volatile unsigned int spare_hitsum[(0x300-0x22C)>>2];
#endif


  /* 0x0300 */ volatile unsigned int scaler[16];
  /* 0x0340 */ volatile unsigned int time_count;
  /* 0x0344 */ volatile unsigned int spare_scaler[(0x400-0x344)>>2];

  /* 0x0400 */ volatile unsigned int testBit;
  /* 0x0404 */ volatile unsigned int clock250count;
  /* 0x0408 */ volatile unsigned int syncp0count;
  /* 0x040C */ volatile unsigned int trig1p0count;
  /* 0x0410 */ volatile unsigned int trig2p0count;

#ifdef CLAS12
  /* 0x0414 */ volatile unsigned int spare_adc_1[(0x500-0x414)>>2];
  /* 0x0500 */ volatile unsigned int gtx_ctrl;
  /* 0x0504 */ volatile unsigned int /*spare_gtx1*/state_csr; /* for Ed */
  /* 0x0508 */ volatile unsigned int /*trx_ctrl*/state_value; /* for Ed */
  /* 0x050C */ volatile unsigned int spare_gtx3;
  /* 0x0510 */ volatile unsigned int gtx_status;
  /* 0x0514 */ volatile unsigned int spare_gtx4;
  /* 0x0518 */ volatile unsigned int gtx_la_ctrl;
  /* 0x051C */ volatile unsigned int gtx_la_status;
  /* 0x0520 */ volatile unsigned int gtx_la_data[2];
  /* 0x0528 */ volatile unsigned int gtx_la_mode;
  /* 0x052C */ volatile unsigned int gtx_la_thr;
  /* 0x0530 */ volatile unsigned int gtx_la_en[2];
  /* 0x0538 */ volatile unsigned int gtx_la_val[2];
  /* 0x0540 */ volatile unsigned int spare_gtx[(0x580-0x540)>>2];
  /* 0x0580 */ volatile unsigned int hist_ctrl;
  /* 0x0584 */ volatile unsigned int hist_time;
  /* 0x0588 */ volatile unsigned int hist_data[16];
#endif

};

struct fadc_data_struct {
  unsigned int new_type;
  unsigned int type;
  unsigned int slot_id_hd;
  unsigned int slot_id_tr;
  unsigned int n_evts;
  unsigned int blk_num;
  unsigned int n_words;
  unsigned int evt_num_1;
  unsigned int evt_num_2;
  unsigned int time_now;
  unsigned int time_1;
  unsigned int time_2;
  unsigned int time_3;
  unsigned int time_4;
  unsigned int chan;
  unsigned int width;
  unsigned int valid_1;
  unsigned int adc_1;
  unsigned int valid_2;
  unsigned int adc_2;
  unsigned int over;
  unsigned int adc_sum;
  unsigned int pulse_num;
  unsigned int thres_bin;
  unsigned int quality;
  unsigned int integral;
  unsigned int time;
  unsigned int chan_a;
  unsigned int source_a;
  unsigned int chan_b;
  unsigned int source_b;
  unsigned int group;
  unsigned int time_coarse;
  unsigned int time_fine;
  unsigned int vmin;
  unsigned int vpeak;
  unsigned int trig_type_int;	/* next 4 for internal trigger data */
  unsigned int trig_state_int;	/* e.g. helicity */
  unsigned int evt_num_int;
  unsigned int err_status_int;
};


struct fadc_sdc_struct {
  volatile unsigned short csr;
  volatile unsigned short ctrl;
  volatile unsigned short busy_enable;
  volatile unsigned short busy_status;
};


/* FADC Special Board IDs */

/* Define CSR Bits */
#define  FA_CSR_EVENT_AVAILABLE                    0x1
#define  FA_CSR_BLOCK_LEVEL_FLAG                   0x2
#define  FA_CSR_BLOCK_READY                        0x4
#define  FA_CSR_BERR_STATUS                        0x8
#define  FA_CSR_TOKEN_STATUS                      0x10
#define  FA_CSR_FIFO1_EMPTY                      0x800
#define  FA_CSR_FIFO1_ALMOST_EMPTY              0x1000
#define  FA_CSR_FIFO1_HALF_FULL                 0x2000
#define  FA_CSR_FIFO1_ALMOST_FULL               0x4000
#define  FA_CSR_FIFO1_FULL                      0x8000
#define  FA_CSR_SOFT_PULSE_TRIG2              0x100000
#define  FA_CSR_SOFT_CLEAR                    0x200000
#define  FA_CSR_DATA_STREAM_SCALERS           0x400000
#define  FA_CSR_FORCE_EOB_INSERT              0x800000
#define  FA_CSR_FORCE_EOB_SUCCESS            0x1000000
#define  FA_CSR_FORCE_EOB_FAILED             0x2000000
#define  FA_CSR_ERROR_MASK                  0x0c000000
#define  FA_CSR_ERROR_CLEAR                 0x08000000
#define  FA_CSR_SYNC                        0x10000000
#define  FA_CSR_TRIGGER                     0x20000000
#define  FA_CSR_SOFT_RESET                  0x40000000
#define  FA_CSR_HARD_RESET                  0x80000000

/* Define Init Flag bits for Clock Source */
#define FA_SOURCE_INT         0x00
#define FA_SOURCE_SDC         0x10
#define FA_SOURCE_VXS         0x20
#define FA_SOURCE_MASK        0x30

/* Define Control2 Bits */
#define FA_CTRL_GO               0x01
#define FA_CTRL_ENABLE_TRIG      0x02
#define FA_CTRL_ENABLE_SRESET    0x04
#define FA_CTRL_ENABLE_INT_TRIG  0x08
#define FA_CTRL_ENABLE_MASK      0x07
#define FA_CTRL_ENABLED          0x07

#define FA_CTRL_COMPRESS_DISABLE 0x000
#define FA_CTRL_COMPRESS_VERIFY  0x240
#define FA_CTRL_COMPRESS_ENABLE  0x280
#define FA_CTRL_COMPRESS_MASK    0x2C0

#define FA_CTRL_VXS_RO_ENABLE    0x400

/* Define CTRL1 Bits */
#define FA_REF_CLK_INTERNAL        0x0
#define FA_REF_CLK_FP              0x1
#define FA_REF_CLK_P0              0x2
#define FA_REF_CLK_MASK            0x3
#define FA_ENABLE_INTERNAL_CLK     0x8

#define FA_TRIG_FP                0x00
#define FA_TRIG_FP_ISYNC          0x10
#define FA_TRIG_P0                0x20
#define FA_TRIG_P0_ISYNC          0x30
#define FA_TRIG_VME_PLAYBACK      0x50
#define FA_TRIG_VME               0x60
#define FA_TRIG_INTERNAL          0x70
#define FA_TRIG_MASK              0x70
#define FA_ENABLE_SOFT_TRIG       0x80

#define FA_SRESET_FP             0x000
#define FA_SRESET_FP_ISYNC       0x100
#define FA_SRESET_P0             0x200
#define FA_SRESET_P0_ISYNC       0x300
#define FA_SRESET_VME            0x600
#define FA_SRESET_NONE           0x700
#define FA_SRESET_MASK           0x700
#define FA_ENABLE_SOFT_SRESET    0x800

#define FA_ENABLE_LIVE_TRIG_OUT  0x1000
#define FA_ENABLE_TRIG_OUT_FP    0x2000
#define FA_ENABLE_TRIG_OUT_P0    0x4000
#define FA_TRIGOUT_MASK          0x7000

#define FA_ENABLE_BLKLVL_INT      0x40000
#define FA_ENABLE_BERR           0x100000
#define FA_ENABLE_MULTIBLOCK     0x200000
#define FA_FIRST_BOARD           0x400000
#define FA_LAST_BOARD            0x800000
#define FA_ENABLE_DEBUG         0x2000000
#define FA_MB_TOKEN_VIA_P0     0x10000000
#define FA_MB_TOKEN_VIA_P2     0x20000000

/* Define Control 2 Bits */
#define FA_GO                         0x1
#define FA_ENABLE_EXT_TRIGGER         0x2
#define FA_ENABLE_EXT_SRESET          0x4
#define FA_ENABLE_INT_TRIGGER         0x8
#define FA_ENABLE_STREAM_MODE        0x10

#define FA_DEBUG_XFER_INFIFO_BFIFO       0x1000
#define FA_DEBUG_XFER_BFIFO_OFIFO        0x2000

/* Define Reset Bits */
#define FA_RESET_CNTL_FPGA              0x1
#define FA_RESET_ADC_FPGA1              0x2
#define FA_RESET_ADC_FIFO1            0x100
#define FA_RESET_DAC                  0x800
#define FA_RESET_EXT_RAM_PT          0x1000
#define FA_RESET_TOKEN              0x10000
#define FA_RESET_ALL                 0xFFFF

/* Define trig_scal bits */
#define FA_TRIG_SCAL_RESET          (1<<31)

/* Define trigger_control bits */
#define FA_TRIGCTL_TRIGSTOP_EN      (1<<31)
#define FA_TRIGCTL_MAX2_MASK     0x00FF0000
#define FA_TRIGCTL_BUSY_EN          (1<<15)
#define FA_TRIGCTL_MAX1_MASK     0x000000FF

/* Define trig21 delay bits */
#define FA_TRIG21_DELAY_MASK     0x00000FFF

/* Define MGT Control bits */
#define FA_MGT_RESET                    0x0
#define FA_RELEASE_MGT_RESET            0x1
#define FA_MGT_FRONT_END_TO_CTP         0x2
#define FA_MGT_ENABLE_DATA_ALIGNMENT    0x4

#define FA_INT_ACTIVE             0x1000000
#define FA_INT_IACKSTAT_WAIT      0x2000000
#define FA_INT_IACKSTAT_ONBT      0x4000000


/* Define RAM Bits */
#define FA_RAM_DATA_MASK       0x000fffff
#define FA_RAM_FULL            0x00100000
#define FA_RAM_EMPTY           0x00200000
#define FA_RAM_ADR_INCREMENT   0x00100000

/* Define Bit Masks */
#define FA_VERSION_MASK        0x000000ff
#define FA_BOARD_MASK          0xffff0000
#define FA_CSR_MASK            0x0fffffff
#define FA_CONTROL_MASK        0xffffffff
#define FA_CONTROL2_MASK       0xc00432ff
#define FA_EVENT_COUNT_MASK    0xffffff
#define FA_BLOCK_COUNT_MASK    0xfffff
#define FA_BLOCK_LEVEL_MASK    0xffff
#define FA_INT_ENABLE_MASK     0xc0000
#define FA_INT_VEC_MASK        0xff
#define FA_INT_LEVEL_MASK      0x700
#define FA_SLOT_ID_MASK        0x1f0000
#define FA_DAC_VALUE_MASK      0x0fff
#define FA_THR_VALUE_MASK      0x0fff
#define FA_THR_INVERT_MASK     0x4000
#define FA_THR_IGNORE_MASK     0x8000
#define FA_PLAYBACK_DIS_MASK   0x2000

#define FA_REF_CLK_SEL_MASK        0x7
#define FA_TRIG_SEL_MASK          0x70
#define FA_SRESET_SEL_MASK       0x700

#define FA_BUSY_LEVEL_MASK     0xfffff
#define FA_FORCE_BUSY       0x80000000

#define FA_A32_ENABLE        0x1
#define FA_AMB_ENABLE        0x1
#define FA_A32_ADDR_MASK     0xff80   /* 8 MB chunks */
#define FA_AMB_MIN_MASK      0xff80
#define FA_AMB_MAX_MASK      0xff800000

#define FA_SADR_AUTO_INCREMENT   0x10000
#define FA_PPG_WRITE_VALUE       0x8000
#define FA_PPG_SAMPLE_MASK       0x1fff
#define FA_PPG_MAX_SAMPLES       512
#define FA_PPG_ENABLE            0x80

/* Define system_monitor Bits/Masks */
#define FA_SYSMON_CTRL_TEMP_MASK        0x3FF
#define FA_SYSMON_FPGA_CORE_V_MASK 0x001FF800
#define FA_SYSMON_FPGA_AUX_V_MASK  0xFFC00000

/* Define MGT Status Bits/Masks */
#define FA_MGT_GTX1_LANE1_UP      0x1
#define FA_MGT_GTX1_LANE2_UP      0x2
#define FA_MGT_GTX1_CHANNEL_UP    0x4
#define FA_MGT_GTX1_HARD_ERROR    0x8
#define FA_MGT_GTX1_SOFT_ERROR    0x10
#define FA_MGT_GTX2_LANE1_UP      0x20
#define FA_MGT_GTX2_LANE2_UP      0x40
#define FA_MGT_GTX2_CHANNEL_UP    0x80
#define FA_MGT_GTX2_HARD_ERROR    0x100
#define FA_MGT_GTX2_SOFT_ERROR    0x200
#define FA_MGT_SUM_DATA_VALID     0x400
#define FA_MGT_RESET_ASSERTED     0x800


/* Define default ADC Processing prarmeters
   values are in clock cycles 4ns/cycle */

#define FA_ADC_NS_PER_CLK      4

#define FA_MAX_PROC_MODE                       10
#define FA_ADC_PROC_MASK        0x7
#define FA_ADC_PROC_ENABLE      0x8
#define FA_ADC_PROC_MODE_WINDOW   1
#define FA_ADC_PROC_MODE_PEAK     2
#define FA_ADC_PROC_MODE_SUM      3
#define FA_ADC_PROC_MODE_TIME     7

/* begin from Bryan for trigger_control */
#define FA_SUPPORTED_MODES                   9,10
#define FA_SUPPORTED_NMODES                     2

extern const char *fa_mode_names[FA_MAX_PROC_MODE+1];

#define FA_ADC_CONFIG0_CHAN_MASK         0x0F00
#define FA_ADC_CONFIG0_CHAN_READ_ENABLE (1<<15)

#define FA_ADC_STATUS1_TRIG_RCV_DONE (1<<15)
#define FA_ADC_STATUS1_TRIGNUM_MASK  0x0FFF

#define FA_ADC_STATUS2_CHAN_DATA_MASK    0x3FFF

#define FA_ADC_CONFIG1_NSAT_MASK  0x0C00
#define FA_ADC_CONFIG1_TNSAT_MASK 0x3000

#define FA_ADC_CONFIG3_TPT_MASK   0x0FFF

#define FA_ADC_CONFIG6_MNPED_MASK   0x00003C00
#define FA_ADC_CONFIG6_PMAXPED_MASK 0x000003FF

#define FA_ADC_CONFIG7_NPED_MASK    0x00003C00
#define FA_ADC_CONFIG7_MAXPED_MASK  0x000003FF
/* end from Bryan for trigger_control */


#define FA_ADC_VERSION_MASK  0x7fff
#define FA_ADC_PLAYBACK_MODE 0x0080

#define FA_ADC_PEAK_MASK     0x0070
#define FA_ADC_CHAN_MASK     0xffff

#define FA_ADC_DEFAULT_PL     50
#define FA_ADC_DEFAULT_PTW    50
#define FA_ADC_DEFAULT_NSB     5
#define FA_ADC_DEFAULT_NSA    10
#define FA_ADC_DEFAULT_NP      4
#define FA_ADC_DEFAULT_THRESH  0
#define FA_ADC_DEFAULT_NPED    4
#define FA_ADC_DEFAULT_MAXPED  1
#define FA_ADC_DEFAULT_NSAT    1
#define FA_ADC_DEFAULT_MNPED   4
#define FA_ADC_DEFAULT_TNSA   10
#define FA_ADC_DEFAULT_TNSAT   1
#define FA_ADC_DEFAULT_TPT     0

#define FA_ADC_MAX_PL       2000
#define FA_ADC_MAX_PTW       512
#define FA_ADC_MAX_NSB       500
#define FA_ADC_MAX_NSA       500
#define FA_ADC_MAX_NP          1
#define FA_ADC_MAX_THRESH   1023
#define FA_ADC_MAX_NPED       15
#define FA_ADC_MAX_MAXPED   1023
#define FA_ADC_MAX_NSAT        4
#define FA_ADC_MAX_MNPED      15
#define FA_ADC_MAX_TNSA       63
#define FA_ADC_MAX_TNSAT       4
#define FA_ADC_MAX_TPT      4095

#define FA_ADC_MIN_PL          1
#define FA_ADC_MIN_PTW         1
#define FA_ADC_MIN_NSB         1
#define FA_ADC_MIN_NSA         1
#define FA_ADC_MIN_NP          1
#define FA_ADC_MIN_THRESH      0
#define FA_ADC_MIN_NPED        4
#define FA_ADC_MIN_MAXPED      0
#define FA_ADC_MIN_NSAT        1
#define FA_ADC_MIN_MNPED       4
#define FA_ADC_MIN_TNSA        2
#define FA_ADC_MIN_TNSAT       2
#define FA_ADC_MIN_TPT         0

#define FA_ADC_NSB_READBACK_MASK    0x0000000F
#define FA_ADC_NSB_NEGATIVE         (1<<3)

#define FA_ADC_NSA_READBACK_MASK    0x000001FF
#define FA_ADC_TNSA_MASK            0x00007E00

#define FA_ADC_TPT_MASK    0xFFF

#define FA_ADC_PEDESTAL_MASK   0xffff
#define FA_ADC_DELAY_MASK      0x1ff

/* Define parameters for HITSUM (Trigger) FPGA */
#define FA_ITRIG_VERSION_MASK     0x1fff
#define FA_ITRIG_CONFIG_MASK      0x001F
#define FA_ITRIG_ENABLE_MASK      0x0010
#define FA_ITRIG_P2OUT_MASK       0x0008
#define FA_ITRIG_MODE_MASK        0x0007

#define FA_ITRIG_TABLE_MODE       1
#define FA_ITRIG_WINDOW_MODE      2
#define FA_ITRIG_COIN_MODE        3
#define FA_ITRIG_SUM_MODE         4

#define FA_ITRIG_DISABLED         0x0010
#define FA_ITRIG_HB_P2OUT         0x0008

#define FA_ITRIG_MIN_WIDTH        0x01  /* 4 nsec */
#define FA_ITRIG_MAX_WIDTH        0x100  /* 1.024 microsec */
#define FA_ITRIG_MAX_DELAY        0xf00  /* 15.360 microsec */

#define FA_ITRIG_WINDOW_MAX_WIDTH 0x200  /* 2.048 microsec */


/* Define ADC Data Types and Masks */

#define FA_DATA_TYPE_DEFINE       0x80000000
#define FA_DATA_TYPE_MASK         0x78000000
#define FA_DATA_SLOT_MASK         0x07c00000

#define FA_DATA_BLOCK_HEADER      0x00000000
#define FA_DATA_BLOCK_TRAILER     0x08000000
#define FA_DATA_EVENT_HEADER      0x10000000
#define FA_DATA_TRIGGER_TIME      0x18000000
#define FA_DATA_WINDOW_RAW        0x20000000
#define FA_DATA_WINDOW_SUM        0x28000000
#define FA_DATA_PULSE_RAW         0x30000000
#define FA_DATA_PULSE_INTEGRAL    0x38000000
#define FA_DATA_PULSE_TIME        0x40000000
#define FA_DATA_STREAM            0x48000000
#define FA_DATA_INVALID           0x70000000
#define FA_DATA_FILLER            0x78000000
#define FA_DUMMY_DATA             0xf800fafa

#define FA_DATA_BLKNUM_MASK       0x0000003f
#define FA_DATA_WRDCNT_MASK       0x003fffff
#define FA_DATA_TRIGNUM_MASK      0x07ffffff

/* Define Firmware registers Data types and Masks */
#define FA_PROMREG1_SRAM_TO_PROM1    0x0
#define FA_PROMREG1_SRAM_TO_PROM2    0x1
#define FA_PROMREG1_PROM1_TO_SRAM    0x3
#define FA_PROMREG1_PROM2_TO_SRAM    0x4
#define FA_PROMREG1_GET_ID1          0x6
#define FA_PROMREG1_GET_ID2          0x7
#define FA_PROMREG1_ERASE_PROM1      0x9
#define FA_PROMREG1_ERASE_PROM2      0xA
#define FA_PROMREG1_REBOOT_FPGA1     0xC
#define FA_PROMREG1_REBOOT_FPGA2     0xD
#define FA_PROMREG1_READY            (1<<31)

#define FA_MEM_ADR_INCR_MEM1      (1<<31)
#define FA_MEM_ADR_INCR_MEM2      (1<<30)

/* Define Scaler Control bits */
#define FA_SCALER_CTRL_ENABLE     (1<<0)
#define FA_SCALER_CTRL_LATCH      (1<<1)
#define FA_SCALER_CTRL_RESET      (1<<2)
#define FA_SCALER_CTRL_MASK        (0x7)

/* Define Scaler Interval Mask */
#define FA_SCALER_INTERVAL_MASK   0x0000FFFF

/* Define Serial Number bits and masks */
#define FA_SERIAL_NUMBER_ACDI               0x41434449
#define FA_SERIAL_NUMBER_ADV_ASSEM_MASK     0xFF000000
#define FA_SERIAL_NUMBER_ADV_ASSEM          0x42000000
#define FA_SERIAL_NUMBER_ACDI_BOARDID_MASK  0x0000FFFF
#define FA_SERIAL_NUMBER_ADV_MNFID1         0x42323135
#define FA_SERIAL_NUMBER_ADV_MNFID2_MASK    0xFFFF0000
#define FA_SERIAL_NUMBER_ADV_MNFID2         0x39350000
#define FA_SERIAL_NUMBER_ADV_BOARDID2_MASK  0x000000FF


/* Define FADC Signal Distribution card bits */

#define FA_SDC_BOARD_ID    0x0200
#define FA_SDC_BOARD_MASK  0xfffe
#define FA_SDC_ADR_MASK    0xffc0

#define FASDC_BUSY_MASK   0x00ff

#define FASDC_CSR_BUSY           0x1
#define FASDC_CSR_TRIG          0x20
#define FASDC_CSR_SRESET        0x40
#define FASDC_CSR_INIT          0x80
#define FASDC_CSR_MASK        0x00e1

#define FASDC_CTRL_CLK_EXT               0x1
#define FASDC_CTRL_NOSYNC_TRIG          0x10
#define FASDC_CTRL_ENABLE_SOFT_TRIG     0x20
#define FASDC_CTRL_NOSYNC_SRESET       0x100
#define FASDC_CTRL_ENABLE_SOFT_SRESET  0x200
#define FASDC_CTRL_ASSERT_SOFT_BUSY   0x8000
#define FASDC_CTRL_MASK               0x8331

/* Definitions for FADC Firmware Tools */
#define        FADC_FIRMWARE_LX110    0
#define        FADC_FIRMWARE_FX70T    1

#define FA_CTRL1_SYSTEM_TEST_MODE   (1<<31)

#define FA_TESTBIT_TRIGOUT          (1<<0)
#define FA_TESTBIT_BUSYOUT          (1<<1)
#define FA_TESTBIT_SDLINKOUT        (1<<2)
#define FA_TESTBIT_TOKENOUT         (1<<3)
#define FA_TESTBIT_STATBITB         (1<<8)
#define FA_TESTBIT_TOKENIN          (1<<9)
#define FA_TESTBIT_CLOCK250_STATUS  (1<<15)

#define FA_CLOCK250COUNT_RESET   (0<<0)
#define FA_CLOCK250COUNT_START   (1<<0)
#define FA_SYNCP0COUNT_RESET     (0<<0)
#define FA_TRIG1P0COUNT_RESET    (0<<0)
#define FA_TRIG2P0COUNT_RESET    (0<<0)

/* faInit initialization flag bits */
#define FA_INIT_SOFT_SYNCRESET      (0<<0)
#define FA_INIT_EXT_SYNCRESET       (1<<0)
#define FA_INIT_SOFT_TRIG           (0<<1)
#define FA_INIT_FP_TRIG             (1<<1)
#define FA_INIT_VXS_TRIG            (2<<1)
#define FA_INIT_INT_CLKSRC          (0<<4)
#define FA_INIT_FP_CLKSRC           (1<<4)
#define FA_INIT_VXS_CLKSRC          (2<<4)
#define FA_INIT_P2_CLKSRC           ((1<<4) | (2<<4))
#define FA_INIT_SKIP                (1<<16)
#define FA_INIT_USE_ADDRLIST        (1<<17)
#define FA_INIT_SKIP_FIRMWARE_CHECK (1<<18)

/* fadcBlockError values */
#define FA_BLOCKERROR_NO_ERROR          0
#define FA_BLOCKERROR_TERM_ON_WORDCOUNT 1
#define FA_BLOCKERROR_UNKNOWN_BUS_ERROR 2
#define FA_BLOCKERROR_ZERO_WORD_COUNT   3
#define FA_BLOCKERROR_DMADONE_ERROR     4
#define FA_BLOCKERROR_NTYPES            5

/* Number of regs to read in faDebugOutput */
#define FA_DEBUG_OUTPUT_NREGS 57

/* Function Prototypes */
int faInit (unsigned int addr, unsigned int addr_inc, int nadc, int iFlag);
int faSlot(unsigned int i);
int faSetClockSource(int id, int clkSrc);
void faStatus(int id, int sflag);
void faGStatus(int sflag);
unsigned int faGetFirmwareVersions(int id, int pflag);
int faSetProcMode(int id, int pmode, unsigned int PL, unsigned int PTW,
	          unsigned int NSB, unsigned int NSA, unsigned int NP, int bank);
void faGSetProcMode(int pmode, unsigned int PL, unsigned int PTW,
	          unsigned int NSB, unsigned int NSA, unsigned int NP, int bank);
void faSetNormalMode(int id, int opt);
int faSetPPG(int id, int pmode, unsigned short *sdata, int nsamples);
void faPPGEnable(int id);
void faPPGDisable(int id);
int faReadBlock(int id, volatile unsigned int *data, int nwrds, int rflag);
int  faGetBlockError(int pflag);
int faPrintBlock(int id, int rflag);
void faClear(int id);
void faClearError(int id);
unsigned int faReadCSR(int id);
void faReset(int id, int iFlag);
void faGReset(int iFlag);
void faSoftReset(int id, int cflag);
void faResetToken(int id);
int  faTokenStatus(int id);
int  faGTokenStatus();
unsigned int faGetTokenStatus(int pflag);
void faSetCalib(int id, unsigned short sdelay, unsigned short tdelay);
int  faSetChannelDisable(int id, int channel);
void faChanDisable(int id, unsigned short cmask);
int  faSetChannelDisableMask(int id, unsigned short cmask);
int  faSetChannelEnable(int id, int channel);
int  faSetChannelEnableMask(int id, unsigned short enMask);
int  faGetChannelMask(int id, int type);
unsigned int faGetChanMask(int id);
void faEnableSyncReset(int id);
void faEnable(int id, int eflag, int bank);
void faGEnable(int eflag, int bank);
void faDisable(int id, int eflag);
void faGDisable(int eflag);
void faTrig(int id);
void faGTrig();
void faTrig2(int id);
void faGTrig2();
int  faSetTrig21Delay(int id, int delay);
int  faGetTrig21Delay(int id);
int  faEnableInternalPlaybackTrigger(int id);
void faSync(int id);
int faDready(int id,int dflag);
int faBready(int id);
unsigned int faGBready();
unsigned int faGBlockReady(unsigned int slotmask, int nloop);
unsigned int faScanMask();
int faBusyLevel(int id, unsigned int val, int bflag);
int faBusy(int id);
void faEnableSoftTrig(int id);
void faDisableSoftTrig(int id);
void faEnableSoftSync(int id);
void faDisableSoftSync(int id);
void faEnableClk(int id);
void faDisableClk(int id);
void faEnableTriggerOut(int id, int output);
void faEnableBusError(int id);
void faDisableBusError(int id);
void faEnableMultiBlock(int tflag);
void faDisableMultiBlock();
int faSetBlockLevel(int id, int level);
void faGSetBlockLevel(int level);
int faSetClkSource(int id, int source);
int faSetTrigSource(int id, int source);
int faSetSyncSource(int id, int source);
void faEnableFP(int id);
int faSetTrigOut(int id, int trigout);
unsigned int faGetTriggerCount(int id);
int faResetTriggerCount(int id);
int faSetThreshold(int id, unsigned short tvalue, unsigned short chmask);
int faPrintThreshold(int id);
int faSetDAC(int id, unsigned short dvalue, unsigned short chmask);
void faPrintDAC(int id);
unsigned int faGetChannelDAC(int id, unsigned int chan);
int faSetChannelPedestal(int id, unsigned int chan, unsigned int ped);
int faGetChannelPedestal(int id, unsigned int chan);
int faLive(int id, int sflag);
void faDataDecode(unsigned int data);

int faSetMGTSettings(int id, int txpre, int txswing, int rxequ);
int faSetMGTTestMode(int id, unsigned int mode);
int faSyncResetMode(int id, unsigned int mode);
/* FADC scaler routine prototypes */
int faReadScalers(int id, volatile unsigned int *data, unsigned int chmask, int rflag);
int faPrintScalers(int id, int rflag);
int faClearScalers(int id);
int faLatchScalers(int id);
int faEnableScalers(int id);
int faDisableScalers(int id);

unsigned int faGetA32(int id);
unsigned int faGetA32M();
unsigned int faGetMinA32MB(int id);
unsigned int faGetMaxA32MB(int id);

/* FADC Internal Trigger Routine prototypes */
int faItrigBurstConfig(int id, unsigned int ntrig,
		   unsigned int burst_window, unsigned int busy_period);
unsigned int faItrigControl(int id, unsigned short itrig_width, unsigned short itrig_dt);
unsigned int faItrigStatus(int id, int sFlag);
int faItrigSetMode(int id, int tmode, unsigned int wMask, unsigned int wWidth,
	           unsigned int cMask, unsigned int sumThresh, unsigned int *tTable);
int faItrigInitTable(int id, unsigned int *table);
int faItrigSetHBwidth(int id, unsigned short hbWidth, unsigned short hbMask);
unsigned int faItrigGetHBwidth(int id, unsigned int chan);
void faItrigPrintHBwidth(int id);
unsigned int faItrigOutConfig(int id, unsigned short itrigDelay, unsigned short itrigWidth);
void faItrigEnable(int id);
void faItrigDisable(int id);
int faItrigGetTableVal (int id, unsigned short pMask);

/* FADC Firmware Tools Prototypes */
int  fadcFirmwareLoad(int id, int chip, int pFlag);
int  fadcFirmwareGLoad(int chip, int pFlag);
void fadcFirmwareDownloadConfigData(int id);
int  fadcFirmwareVerifyDownload (int id);
int  fadcFirmwareTestReady(int id, int n_try, int pFlag);
int  fadcFirmwareZeroSRAM (int id);
int  fadcFirmwareCheckSRAM (int id);
void fadcFirmwareSetFilename(char *filename, int chip);
int  fadcFirmwareReadFile(char *filename);
int  fadcFirmwareGetFpgaID(int pflag);
int  fadcFirmwareChipFromFpgaID(int pflag);
int  fadcFirmwareRevFromFpgaID(int pflag);
int  fadcFirmwareReadMcsFile(char *filename);
enum faArgs_enum
  {
    FA_ARGS_SHOW_ID,
    FA_ARGS_SHOW_PROGRESS,
    FA_ARGS_SHOW_DONE,
    FA_ARGS_SHOW_STRING,
    FA_ARGS_LAST
  };

typedef struct faUpdateWatcherArgs_struct
{
  int step; /* 0: show id, 1: show progress, 2: show complete */
  int id;   /* slot id */
  char title[80];
} faUpdateWatcherArgs_t;
int  fadcFirmwareAttachUpdateWatcher(VOIDFUNCPTR routine, faUpdateWatcherArgs_t arg);
void fadcFirmwareUpdateWatcher(faUpdateWatcherArgs_t arg);

void faTestSetSystemTestMode(int id, int mode);
void faTestSetTrigOut(int id, int mode);
void faTestSetBusyOut(int id, int mode);
void faTestSetSdLink(int id, int mode);
void faTestSetTokenOut(int id, int mode);
int  faTestGetStatBitB(int id);
int  faTestGetTokenIn(int id);
int  faTestGetClock250CounterStatus(int id);
unsigned int faTestGetClock250Counter(int id);
unsigned int faTestGetSyncCounter(int id);
unsigned int faTestGetTrig1Counter(int id);
unsigned int faTestGetTrig2Counter(int id);
void faTestResetClock250Counter(int id);
void faTestResetSyncCounter(int id);
void faTestResetTrig1Counter(int id);
void faTestResetTrig2Counter(int id);
unsigned int faTestGetTestBitReg(int id);
int  faTestSystemClock(int id, int pflag);

int  faGetSerialNumber(int id, char **rval, int snfix);
int  faSetScalerBlockInterval(int id, unsigned int nblock);
int  faGetScalerBlockInterval(int id);
int  faForceEndOfBlock(int id, int scalers);
void faGForceEndOfBlock(int scalers);

/* FLASH SDC prototypes */
int faSDC_Config(unsigned short cFlag, unsigned short bMask);
void faSDC_Status(int sFlag);
void faSDC_Enable(int nsync);
void faSDC_Disable();
void faSDC_Sync();
void faSDC_Trig();
int faSDC_Busy();


/*sergey*/
int faGetProcMode(int id, int *pmode, unsigned int *PL, unsigned int *PTW,
				  unsigned int *NSB, unsigned int *NSA, unsigned int *NP);
int faGetNfadc();
int faId(unsigned int slot);
int faSetThresholdAll(int id, unsigned short tvalue[16]);
int faSetPedestal(int id, unsigned int wvalue);
int faPrintPedestal(int id);
#ifdef CLAS12
int faSetTriggerProcessingMode(int id, unsigned int chan, unsigned int mask);
int faGetTriggerProcessingMode(int id, unsigned int chan);
int faSetChannelDelay(int id, unsigned int chan, unsigned int delay);
int faGetChannelDelay(int id, unsigned int chan);
int faInvert(int id, unsigned short chmask);
unsigned int faGetInvertMask(int id);
int faResetMGT(int id, int reset);
int faGetMGTChannelStatus(int id);
int faSetChannelGain(int id, unsigned int chan, float gain);
float faGetChannelGain(int id, unsigned int chan);
int faGLoadChannelPedestals(char *fname, int updateThresholds);
int faThresholdIgnore(int id, unsigned short chmask);
unsigned int faGetThresholdIgnoreMask(int id);
int faPlaybackDisable(int id, unsigned short chmask);
unsigned int faGetPlaybackDisableMask(int id);
int faSetHitbitTrigMask(int id, unsigned short chmask);
unsigned int faGetHitbitTrigMask(int id);
int faSetHitbitTrigWidth(int id, unsigned short width);
unsigned int faGetHitbitTrigWidth(int id);

int faGSetHitbitMinTOT(unsigned short width);
int faGSetHitbitMinMultiplicity(unsigned short mult);
int faSetHitbitMinTOT(int id, unsigned short width);
int faSetHitbitMinMultiplicity(int id, unsigned short mult);
int faGetHitbitMinTOT(int id);
int faGetHitbitMinMultiplicity(int id);

/*Andrea*/
int faSetDelayAll(int id,unsigned int delay);
int faSetGlobalDelay(unsigned int delay);

void faSetCompression(int id, int opt);
int faGetCompression(int id);

void faSetVXSReadout(int id, int opt);
void faGSetVXSReadout(int opt);
int faGetVXSReadout(int id);

typedef struct
{
  double avg;
  double rms;
  double min;
  double max;
} fa250Ped;

int faMeasureChannelPedestal(int id, unsigned int chan, fa250Ped *ped);

int faGetChThreshold(int id, int ch);
int faSetChThreshold(int id, int ch, int threshold);

void faSetA32BaseAddress(unsigned int addr);


#endif

int  faCalcMaxUnAckTriggers(int mode, int ptw, int nsa, int nsb, int np);
int  faSetTriggerStopCondition(int id, int trigger_max);
int  faSetTriggerBusyCondition(int id, int trigger_max);
int  faSetTriggerPathSamples(int id, unsigned int TNSA, unsigned int TNSAT);
void faGSetTriggerPathSamples(unsigned int TNSA, unsigned int TNSAT);
int  faSetTriggerPathThreshold(int id, unsigned int TPT);
void faGSetTriggerPathThreshold(unsigned int TPT);

/* for Ed */
int  faArmStatesStorage(int id);
int  faDisarmStatesStorage(int id);
int  faReadStatesStorage(int id);
/* for Ed */

#endif /* __FADCLIB__ */
