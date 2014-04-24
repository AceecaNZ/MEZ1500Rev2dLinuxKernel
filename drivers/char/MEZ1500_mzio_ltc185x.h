// MZIO LTC185x IOCTL routines
#define MZIO_LTC185x_INIT							_IOWR(0,0,int)
#define MZIO_LTC185x_DEINIT						_IOWR(0,1,int)
#define MZIO_LTC185x_SETUP_CH0SE			_IOWR(0,2,int) 
#define MZIO_LTC185x_SETUP_CH1SE			_IOWR(0,3,int) 
#define MZIO_LTC185x_SETUP_CH2SE			_IOWR(0,4,int) 
#define MZIO_LTC185x_SETUP_CH3SE			_IOWR(0,5,int) 
#define MZIO_LTC185x_SETUP_CH4SE			_IOWR(0,6,int) 
#define MZIO_LTC185x_SETUP_CH5SE			_IOWR(0,7,int) 
#define MZIO_LTC185x_SETUP_CH6SE			_IOWR(0,8,int) 
#define MZIO_LTC185x_SETUP_CH7SE			_IOWR(0,9,int) 
#define MZIO_LTC185x_SETUP_CH01DE			_IOWR(0,10,int)
#define MZIO_LTC185x_SETUP_CH23DE			_IOWR(0,11,int)
#define MZIO_LTC185x_SETUP_CH45DE			_IOWR(0,12,int)
#define MZIO_LTC185x_SETUP_CH67DE			_IOWR(0,13,int)
#define MZIO_LTC185x_5VCn1_Enable			_IOWR(0,14,int)
#define MZIO_LTC185x_5VCn2_Enable			_IOWR(0,15,int)
#define MZIO_LTC185x_5VCn3_Enable			_IOWR(0,16,int)
#define MZIO_LTC185x_START						_IOWR(0,17,int)
#define MZIO_LTC185x_STOP							_IOWR(0,18,int)

// ADC Channel setup 
#define LTC185x_ChSetup_Enabled				0x8000		// Set to enable, clear to disable
#define LTC185x_ChSetup_OddEven				0x0040		// set for Odd, clear for even, differential channels only
#define LTC185x_ChSetup_UniBi					0x0080		// Set to be unipolar, clear to be bipolar
#define LTC185x_ChSetup_Gain					0x0004		// Set to be 0-5V/-+5V, clear to be 0-10V/-+10V




