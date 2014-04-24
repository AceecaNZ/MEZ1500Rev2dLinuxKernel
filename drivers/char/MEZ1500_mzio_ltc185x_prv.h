#ifndef __ADC1_H__
#define __ADC1_H__ 1

#define TimerFreq		100000 // Hz
#define Timer1ms		TimerFreq/1000 
#define Timer10ms		TimerFreq/100 
#define Timer100ms	TimerFreq/10 
#define Timer1000ms	TimerFreq 

#ifndef ON
  #define ON  true
#endif

#ifndef OFF
  #define OFF false
#endif

// Bit Definitions
#ifndef BIT0
  #define  BIT0  0x00000001
  #define  BIT1  0x00000002
  #define  BIT2  0x00000004
  #define  BIT3  0x00000008
  #define  BIT4  0x00000010
  #define  BIT5  0x00000020
  #define  BIT6  0x00000040
  #define  BIT7  0x00000080
#endif
#ifndef BIT8
  #define  BIT8  0x00000100
  #define  BIT9  0x00000200
  #define  BIT10 0x00000400
  #define  BIT11 0x00000800
  #define  BIT12 0x00001000
  #define  BIT13 0x00002000
  #define  BIT14 0x00004000
  #define  BIT15 0x00008000
#endif
#ifndef BIT16
  #define  BIT16 0x00010000
  #define  BIT17 0x00020000
  #define  BIT18 0x00040000
  #define  BIT19 0x00080000
  #define  BIT20 0x00100000
  #define  BIT21 0x00200000
  #define  BIT22 0x00400000
  #define  BIT23 0x00800000
#endif
#ifndef BIT24
  #define  BIT24 0x01000000
  #define  BIT25 0x02000000
  #define  BIT26 0x04000000
  #define  BIT27 0x08000000
  #define  BIT28 0x10000000
  #define  BIT29 0x20000000
  #define  BIT30 0x40000000
  #define  BIT31 0x80000000
#endif

// CON bits
#define bGPCON_Input        0   // Input bits
#define bGPCON_Output       1   // Output bits
#define bGPCON_Dedicated    2   // Dedicated bits
#define bGPCON_Funct1       2   // Dedicated function 1
#define bGPCON_Funct2       3   // Dedicated function 2
#define bGPCON_Reserved     3   // Reserve bits
#define bGPCON_AllBits      3   // All the bits

// DAT bit masks
#define bDAT_CRNT_CN1_EN_N_CAM_DATA0        BIT0
#define bDAT_CRNT_CN2_EN_N_CAM_DATA1        BIT1
#define bDAT_CRNT_CN3_EN_N_CAM_DATA2        BIT2
#define bDAT_CLIM_EN_N_CAM_DATA3            BIT3
#define bDAT_CLIM_FLT_N_CAM_DATA4           BIT4
#define bDAT_ADC_RD_EN_N_CAM_DATA5          BIT5
#define bDAT_ADC_CNV_START_CAM_DATA6        BIT6
#define bDAT_ADC_CNV_BSY_N_CAM_DATA7        BIT7
#define bDAT_UNUSED_CAM_PCLK                BIT8
#define bDAT_UNUSED_CAM_VSYNC               BIT9
#define bDAT_UNUSED_CAM_HREF                BIT10
#define bDAT_UNUSED_CAM_CLK_OUT             BIT11
#define bDAT_UNUSED_CAM_RST                 BIT12

// CON bit field masks
#define bmaskCON_CRNT_CN1_EN_N_CAM_DATA0    (bGPCON_AllBits << 0)
#define bmaskCON_CRNT_CN2_EN_N_CAM_DATA1    (bGPCON_AllBits << 2)
#define bmaskCON_CRNT_CN3_EN_N_CAM_DATA2    (bGPCON_AllBits << 4)
#define bmaskCON_CLIM_EN_N_CAM_DATA3        (bGPCON_AllBits << 6)
#define bmaskCON_CLIM_FLT_N_CAM_DATA4       (bGPCON_AllBits << 8)
#define bmaskCON_ADC_RD_EN_N_CAM_DATA5      (bGPCON_AllBits << 10)
#define bmaskCON_ADC_CNV_START_CAM_DATA6    (bGPCON_AllBits << 12)
#define bmaskCON_ADC_CNV_BSY_N_CAM_DATA7    (bGPCON_AllBits << 14)
#define bmaskCON_UNUSED_CAM_PCLK            (bGPCON_AllBits << 16)
#define bmaskCON_UNUSED_CAM_VSYNC           (bGPCON_AllBits << 18)
#define bmaskCON_UNUSED_CAM_HREF            (bGPCON_AllBits << 20)
#define bmaskCON_UNUSED_CAM_CLK_OUT         (bGPCON_AllBits << 22)
#define bmaskCON_UNUSED_CAM_RST             (bGPCON_AllBits << 24)

// CON bit masks
#define bCON_CRNT_CN1_EN_N_CAM_DATA0        (bGPCON_Output << 0)
#define bCON_CRNT_CN2_EN_N_CAM_DATA1        (bGPCON_Output << 2)
#define bCON_CRNT_CN3_EN_N_CAM_DATA2        (bGPCON_Output << 4)
#define bCON_CLIM_EN_N_CAM_DATA3            (bGPCON_Output << 6)
#define bCON_CLIM_FLT_N_CAM_DATA4           (bGPCON_Input  << 8)
#define bCON_ADC_RD_EN_N_CAM_DATA5          (bGPCON_Output << 10)
#define bCON_ADC_CNV_START_CAM_DATA6        (bGPCON_Output << 12)
#define bCON_ADC_CNV_BSY_N_CAM_DATA7        (bGPCON_Input  << 14)
#define bCON_UNUSED_CAM_PCLK                (bGPCON_Output << 16)
#define bCON_UNUSED_CAM_VSYNC               (bGPCON_Output << 18)
#define bCON_UNUSED_CAM_HREF                (bGPCON_Output << 20)
#define bCON_UNUSED_CAM_CLK_OUT             (bGPCON_Output << 22)
#define bCON_UNUSED_CAM_RST                 (bGPCON_Output << 24)

// InitAdc outputs (Bits not listed are inputs)
#define bGPCON_CAM_init                     bCON_CRNT_CN1_EN_N_CAM_DATA0  |\
                                            bCON_CRNT_CN2_EN_N_CAM_DATA1  |\
                                            bCON_CRNT_CN3_EN_N_CAM_DATA2  |\
                                            bCON_CLIM_EN_N_CAM_DATA3      |\
                                            bCON_ADC_RD_EN_N_CAM_DATA5    |\
                                            bCON_ADC_CNV_START_CAM_DATA6  |\
                                            bCON_UNUSED_CAM_PCLK          |\
                                            bCON_UNUSED_CAM_VSYNC         |\
                                            bCON_UNUSED_CAM_HREF          |\
                                            bCON_UNUSED_CAM_CLK_OUT       |\
                                            bCON_UNUSED_CAM_RST

// InitAdc output states (1 = high)

#define bGPDAT_CAM_init                     bDAT_CRNT_CN1_EN_N_CAM_DATA0  |\
                                            bDAT_CRNT_CN2_EN_N_CAM_DATA1  |\
                                            bDAT_CRNT_CN3_EN_N_CAM_DATA2  |\
                                            bDAT_CLIM_EN_N_CAM_DATA3      |\
                                            bDAT_ADC_RD_EN_N_CAM_DATA5

// InitAdc pull-up states (1 = off)
#define bGPUP_CAM_init                      bDAT_CRNT_CN1_EN_N_CAM_DATA0  |\
                                            bDAT_CRNT_CN2_EN_N_CAM_DATA1  |\
                                            bDAT_CRNT_CN3_EN_N_CAM_DATA2  |\
                                            bDAT_CLIM_EN_N_CAM_DATA3      |\
                                            bDAT_CLIM_FLT_N_CAM_DATA4     |\
                                            bDAT_ADC_RD_EN_N_CAM_DATA5    |\
                                            bDAT_ADC_CNV_START_CAM_DATA6  |\
                                            bDAT_ADC_CNV_BSY_N_CAM_DATA7  |\
                                            bDAT_UNUSED_CAM_PCLK          |\
                                            bDAT_UNUSED_CAM_VSYNC         |\
                                            bDAT_UNUSED_CAM_HREF          |\
                                            bDAT_UNUSED_CAM_CLK_OUT       |\
                                            bDAT_UNUSED_CAM_RST

#define CNx     0     // 5V_LIM - Master control for CN1, CN2 & CN3 
#define CN1     1     // 5V_CN1
#define CN2     2     // 5V_CN2
#define CN3     3     // 5V_CN3

// ADC bit masks
#define ADC_SLEEP_                  BIT0
#define ADC_NAP_                    BIT1
#define ADC_GAIN                    BIT2
#define ADC_UNI                     BIT3
#define ADC_SELECT0                 BIT4
#define ADC_SELECT1                 BIT5
#define ADC_ODD_SIGN                BIT6
#define ADC_SGL_DIFF                BIT7

// Multiplexer Channel Selection (Single-ended)
#define ADC_SINGLE_ENDED_INPUT1             ADC_SGL_DIFF

#define ADC_SINGLE_ENDED_INPUT2             ADC_SGL_DIFF  |\
                                            ADC_ODD_SIGN

#define ADC_SINGLE_ENDED_INPUT3             ADC_SGL_DIFF  |\
                                            ADC_SELECT0

#define ADC_SINGLE_ENDED_INPUT4             ADC_SGL_DIFF  |\
                                            ADC_ODD_SIGN  |\
                                            ADC_SELECT0

#define ADC_SINGLE_ENDED_INPUT5             ADC_SGL_DIFF  |\
                                            ADC_SELECT1

#define ADC_SINGLE_ENDED_INPUT6             ADC_SGL_DIFF  |\
                                            ADC_ODD_SIGN  |\
                                            ADC_SELECT1

#define ADC_SINGLE_ENDED_INPUT7             ADC_SGL_DIFF  |\
                                            ADC_SELECT1   |\
                                            ADC_SELECT0

#define ADC_SINGLE_ENDED_INPUT8             ADC_SGL_DIFF  |\
                                            ADC_ODD_SIGN  |\
                                            ADC_SELECT1   |\
                                            ADC_SELECT0

// Multiplexer Channel Selection (Differential)
#define ADC_DIFFERENTIAL_EVEN_INPUT1_2      0

#define ADC_DIFFERENTIAL_EVEN_INPUT3_4      ADC_SELECT0

#define ADC_DIFFERENTIAL_EVEN_INPUT5_6      ADC_SELECT1

#define ADC_DIFFERENTIAL_EVEN_INPUT7_8      ADC_SELECT1   |\
                                            ADC_SELECT0

#define ADC_DIFFERENTIAL_ODD_INPUT1_2       ADC_ODD_SIGN

#define ADC_DIFFERENTIAL_ODD_INPUT3_4       ADC_ODD_SIGN  |\
                                            ADC_SELECT0

#define ADC_DIFFERENTIAL_ODD_INPUT5_6       ADC_ODD_SIGN  |\
                                            ADC_SELECT1

#define ADC_DIFFERENTIAL_ODD_INPUT7_8       ADC_ODD_SIGN  |\
                                            ADC_SELECT1   |\
                                            ADC_SELECT0

// Input Range Selection
#define ADC_5V_BI                           0
#define ADC_5V_UNI                          ADC_UNI
#define ADC_10V_BI                          ADC_GAIN
#define ADC_10V_UNI                         ADC_UNI | ADC_GAIN

// Power Down Selection
#define ADC_POWER_ON                        0
#define ADC_NAP                             ADC_NAP_
#define ADC_SLEEP                           ADC_SLEEP_


#endif  // __ADC1_H__
