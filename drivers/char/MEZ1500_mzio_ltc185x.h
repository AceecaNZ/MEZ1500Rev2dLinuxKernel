#ifndef __MEZ1500_MZIO_LTC185x_H__
#define __MEZ1500_MZIO_LTC185x_H__ 1

// MZIO LTC185x IOCTL routines
// Init/Deinit routines
#define MZIO_LTC185x_INIT							   		_IOWR(0,0,int)  
#define MZIO_LTC185x_DEINIT						      _IOWR(0,1,int)  

// Channel setup routines, note SET_PERIOD is in 10us increments, 0=infinity
#define MZIO_LTC185x_CH0SE_SETUP			    	_IOWR(0,10,int)
#define MZIO_LTC185x_CH0SE_SET_PERIOD	      _IOWR(0,11,int)
#define MZIO_LTC185x_CH0SE_SET_BUFFER			  _IOWR(0,12,int)
                                                           
#define MZIO_LTC185x_CH1SE_SETUP  		      _IOWR(0,20,int)
#define MZIO_LTC185x_CH1SE_SET_PERIOD	      _IOWR(0,21,int)
                                                           
#define MZIO_LTC185x_CH2SE_SETUP			      _IOWR(0,30,int)
#define MZIO_LTC185x_CH2SE_SET_PERIOD	      _IOWR(0,31,int)
                                                           
#define MZIO_LTC185x_CH3SE_SETUP			      _IOWR(0,40,int)
#define MZIO_LTC185x_CH3SE_SET_PERIOD	      _IOWR(0,41,int)
                                                           
#define MZIO_LTC185x_CH4SE_SETUP			      _IOWR(0,50,int)
#define MZIO_LTC185x_CH4SE_SET_PERIOD	      _IOWR(0,51,int)
                                                           
#define MZIO_LTC185x_CH5SE_SETUP			      _IOWR(0,60,int)
#define MZIO_LTC185x_CH5SE_SET_PERIOD	      _IOWR(0,61,int)
                                                           
#define MZIO_LTC185x_CH6SE_SETUP			      _IOWR(0,70,int)
#define MZIO_LTC185x_CH6SE_SET_PERIOD	      _IOWR(0,71,int)
                                                           
#define MZIO_LTC185x_CH7SE_SETUP			      _IOWR(0,80,int)
#define MZIO_LTC185x_CH7SE_SET_PERIOD	      _IOWR(0,81,int)
                                                           
#define MZIO_LTC185x_CH01DE_SETUP			      _IOWR(0,90,int)
#define MZIO_LTC185x_CH01SE_SET_PERIOD      _IOWR(0,91,int)
                                                           
#define MZIO_LTC185x_CH23DE_SETUP			      _IOWR(0,100,int)
#define MZIO_LTC185x_CH23SE_SET_PERIOD      _IOWR(0,101,int)
                                                           
#define MZIO_LTC185x_CH45DE_SETUP			      _IOWR(0,110,int)
#define MZIO_LTC185x_CH45SE_SET_PERIOD      _IOWR(0,111,int)
                                                           
#define MZIO_LTC185x_CH67DE_SETUP			      _IOWR(0,120,int)
#define MZIO_LTC185x_CH67SE_SET_PERIOD      _IOWR(0,121,int)
                                                           
#define MZIO_LTC185x_5VCn1_Enable			      _IOWR(0,130,int)
#define MZIO_LTC185x_5VCn2_Enable			      _IOWR(0,131,int)
#define MZIO_LTC185x_5VCn3_Enable			      _IOWR(0,132,int)
                                                           
#define MZIO_LTC185x_START						      _IOWR(0,140,int)
#define MZIO_LTC185x_STOP							      _IOWR(0,141,int)
                                                           
#define MZIO_LTC185x_READ							      _IOWR(0,150,int)
#define MZIO_LTC185x_SET_READ_FILENAME	    _IOWR(0,151,int)

// ADC Channel setup
#define LTC185x_ChSetup_Enabled				    	0x80000000		// Set to enable, clear to disable                                       
#define LTC185x_ChSetup_OddEven				      0x00400000		// set for Odd, clear for even, differential channels only               
#define LTC185x_ChSetup_UniBi					      0x00800000		// Set to be unipolar, clear to be bipolar                               
#define LTC185x_ChSetup_Gain					      0x00040000		// Set to be 0-5V/-+5V, clear to be 0-10V/-+10V                          

typedef struct { 
  int *bufferStart;							// start of the user space buffer
  unsigned long	bufferSize;			// size of user space buffer (wrP will wrap around)
  unsigned int *numSamplesP;		// pointer to user space to track bytes written into the user space buffer
} ChBufferData;

#define Ch0Filename	"/tmp/LTC185x_Ch0.bin"
#define bufferSampleSize	1000
#define filesize					(bufferSampleSize * sizeof(int));

struct mmap_tmpFile_info {
	char *data;	/* the data */
	int reference;       /* how many times it is mmapped */  	
};


#endif  // __MEZ1500_MZIO_LTC185x_H__