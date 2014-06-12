#ifndef __MEZ1500_MZIO_LTC185x_H__
#define __MEZ1500_MZIO_LTC185x_H__ 1

// MZIO LTC185x IOCTL routines
// Init/Deinit routines
#define MZIO_LTC185x_INIT                   _IOWR(0,0,int)
#define MZIO_LTC185x_DEINIT                 _IOWR(0,1,int)

// Channel setup routines, note SET_PERIOD is in 10us increments, 0=infinity
#define MZIO_LTC185x_CHANNEL_SETUP          _IOWR(0,10,int)   // Use *ChConfigData as arg

#define MZIO_LTC185x_5VCn1_Enable           _IOWR(0,130,int)
#define MZIO_LTC185x_5VCn2_Enable           _IOWR(0,131,int)
#define MZIO_LTC185x_5VCn3_Enable           _IOWR(0,132,int)

#define MZIO_LTC185x_START                  _IOWR(0,140,int)
#define MZIO_LTC185x_STOP                   _IOWR(0,141,int)

#define MZIO_LTC185x_READ_BUFFER            _IOWR(0,150,int)
#define MZIO_LTC185x_SET_READ_FILENAME      _IOWR(0,151,int)

// ADC Channel setup
#define LTC185x_ChSetup_Enabled             0x80000000    // Set to enable, clear to disable
#define LTC185x_ChSetup_OddEven             0x00400000    // set for Odd, clear for even, differential channels only
#define LTC185x_ChSetup_UniBi               0x00800000    // Set to be unipolar, clear to be bipolar
#define LTC185x_ChSetup_Gain                0x00040000    // Set to be 0-5V/-+5V, clear to be 0-10V/-+10V

// ADC Channel select
#define Chn0                                0
#define Chn1                                1
#define Chn2                                2
#define Chn3                                3
#define Chn4                                4
#define Chn5                                5
#define Chn6                                6
#define Chn7                                7
#define Chn01                               8
#define Chn23                               9
#define Chn45                               10
#define Chn67                               11
#define ChnMax                              Chn67

typedef struct {
  unsigned short  ch;             // channel
  unsigned short  *buf;           // user space buffer, if NULL will return number of sample available
  unsigned int    numSamples;     // number of bytes to read
  unsigned int    *overun;        // set to 1 if the buffer has been overrun
} ReadBufferData;

typedef struct {
  unsigned short  ch;             // channel
  unsigned int   	config;         // the config tuple for the channel
  unsigned long   period;         // the sampling period of the channel in us, minimum is 50us
} ChConfigData;

#define ChSampleSize 170        // Makes a 4096 page
//#define ChSampleSize 10
typedef struct {
  unsigned short  Ch0Buf[ChSampleSize];
  unsigned short  Ch1Buf[ChSampleSize];
  unsigned short  Ch2Buf[ChSampleSize];
  unsigned short  Ch3Buf[ChSampleSize];
  unsigned short  Ch4Buf[ChSampleSize];
  unsigned short  Ch5Buf[ChSampleSize];
  unsigned short  Ch6Buf[ChSampleSize];
  unsigned short  Ch7Buf[ChSampleSize];
  unsigned short  Ch01Buf[ChSampleSize];
  unsigned short  Ch23Buf[ChSampleSize];
  unsigned short  Ch45Buf[ChSampleSize];
  unsigned short  Ch67Buf[ChSampleSize];
} BufData;


#define Ch0Filename "/tmp/LTC185x_Ch0.bin"
#define bufferSampleSize  1000
#define filesize          (bufferSampleSize * sizeof(int));

struct mmap_tmpFile_info {
  char *data; /* the data */
  int reference;       /* how many times it is mmapped */
};


#endif  // __MEZ1500_MZIO_LTC185x_H__
