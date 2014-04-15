// MZIO IOCTL routines
#define MZIO_GPIO_SET_HIGH		_IOWR(0,0,int)
#define MZIO_GPIO_SET_LOW			_IOWR(0,1,int)
#define MZIO_GPIO_SET_DIR_OUT	_IOWR(0,2,int)
#define MZIO_GPIO_SET_DIR_IN	_IOWR(0,3,int)
#define MZIO_GPIO_SET_PU_OFF	_IOWR(0,4,int)
#define MZIO_GPIO_SET_PU_ON		_IOWR(0,5,int)
#define MZIO_GPIO_GET					_IOWR(0,6,int)
#define MZIO_DRIVER_DEBUG			_IOWR(0,7,int)
#define MZIO_CAMIF_SET_CFG		_IOWR(0,8,int)
#define MZIO_CAMIF_GET_CFG		_IOWR(0,9,int)
#define MZIO_CAMIF_SET_DAT		_IOWR(0,10,int)
#define MZIO_CAMIF_GET_DAT		_IOWR(0,11,int)
#define MZIO_CAMIF_SET_UP	  	_IOWR(0,12,int)
#define MZIO_CAMIF_GET_UP 		_IOWR(0,13,int)

// MZIO GPIO definitions
#define MZIO_MOD_RESET				0
#define MZIO_MOD_PWR					1
#define MZIO_CAMIF_DAT0  			2
#define MZIO_CAMIF_DAT1  			3
#define MZIO_CAMIF_DAT2  			4
#define MZIO_CAMIF_DAT3  			5
#define MZIO_CAMIF_DAT4  			6
#define MZIO_CAMIF_DAT5  			7
#define MZIO_CAMIF_DAT6  			8
#define MZIO_CAMIF_DAT7  			9
#define MZIO_5V_MZ_ENn  			10
#define MZIO_3V3_EXT_ENn			11


