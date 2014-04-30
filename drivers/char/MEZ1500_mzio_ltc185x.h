// MZIO LTC185x IOCTL routines
// Init/Deinit routines
#define MZIO_LTC185x_INIT							   	_IOWR(0,0,int)       
#define MZIO_LTC185x_DEINIT						    _IOWR(0,1,int)       
                       
// Channel setup routines, note SET_PERIOD is in 10us increments, 0=infinity                                        
#define MZIO_LTC185x_CH0SE_SETUP			    _IOWR(0,10,int)       
#define MZIO_LTC185x_CH0SE_SET_PERIOD	    _IOWR(0,11,int)      
                                                               
#define MZIO_LTC185x_CH1SE_SETUP  		    _IOWR(0,12,int)       
#define MZIO_LTC185x_CH1SE_SET_PERIOD	    _IOWR(0,13,int)      
                                                               
#define MZIO_LTC185x_CH2SE_SETUP			    _IOWR(0,14,int)       
#define MZIO_LTC185x_CH2SE_SET_PERIOD	    _IOWR(0,15,int)      
                                                               
#define MZIO_LTC185x_CH3SE_SETUP			    _IOWR(0,16,int)       
#define MZIO_LTC185x_CH3SE_SET_PERIOD	    _IOWR(0,17,int)      
                                                               
#define MZIO_LTC185x_CH4SE_SETUP			    _IOWR(0,18,int)       
#define MZIO_LTC185x_CH4SE_SET_PERIOD	    _IOWR(0,19,int)      
                                                               
#define MZIO_LTC185x_CH5SE_SETUP			    _IOWR(0,20,int)       
#define MZIO_LTC185x_CH5SE_SET_PERIOD	    _IOWR(0,21,int)      
                                                               
#define MZIO_LTC185x_CH6SE_SETUP			    _IOWR(0,22,int)       
#define MZIO_LTC185x_CH6SE_SET_PERIOD	    _IOWR(0,23,int)      
                                                               
#define MZIO_LTC185x_CH7SE_SETUP			    _IOWR(0,24,int)       
#define MZIO_LTC185x_CH7SE_SET_PERIOD	    _IOWR(0,25,int)      
                                                               
#define MZIO_LTC185x_CH01DE_SETUP			    _IOWR(0,26,int)      
#define MZIO_LTC185x_CH01SE_SET_PERIOD    _IOWR(0,27,int)      
                                                               
#define MZIO_LTC185x_CH23DE_SETUP			    _IOWR(0,28,int)      
#define MZIO_LTC185x_CH23SE_SET_PERIOD    _IOWR(0,29,int)      
                                                               
#define MZIO_LTC185x_CH45DE_SETUP			    _IOWR(0,30,int)      
#define MZIO_LTC185x_CH45SE_SET_PERIOD    _IOWR(0,31,int)      
                                                               
#define MZIO_LTC185x_CH67DE_SETUP			    _IOWR(0,32,int)      
#define MZIO_LTC185x_CH67SE_SET_PERIOD    _IOWR(0,33,int)      
                                                               
#define MZIO_LTC185x_5VCn1_Enable			    _IOWR(0,40,int)      
#define MZIO_LTC185x_5VCn2_Enable			    _IOWR(0,41,int)      
#define MZIO_LTC185x_5VCn3_Enable			    _IOWR(0,42,int)      
                                                               
#define MZIO_LTC185x_START						    _IOWR(0,50,int)      
#define MZIO_LTC185x_STOP							    _IOWR(0,51,int)      
                                                                                                                         
// ADC Channel setup                                                                                                     
#define LTC185x_ChSetup_Enabled				    0x80000000		// Set to enable, clear to disable                               
#define LTC185x_ChSetup_OddEven				    0x00400000		// set for Odd, clear for even, differential channels only       
#define LTC185x_ChSetup_UniBi					    0x00800000		// Set to be unipolar, clear to be bipolar                       
#define LTC185x_ChSetup_Gain					    0x00040000		// Set to be 0-5V/-+5V, clear to be 0-10V/-+10V                  
                                                                                                                         

