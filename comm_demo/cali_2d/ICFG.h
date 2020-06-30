#ifndef __ICAL_CONFIG_H__
#define __ICAL_CONFIG_H__

#define LOG_TAG "QMCX983D"
#define PI	3.1415926535897932f

//#define ICAL_DBG

#ifdef ICAL_DBG
	#ifdef  _WIN32
	#define  D(...)  printf(__VA_ARGS__);printf("\n");
	#define  E(...)  printf(__VA_ARGS__);printf("\n");
	#else
	#define  D(...)  ALOGD(__VA_ARGS__)
	#define  E(...)  ALOGE(__VA_ARGS__)
	#endif
#else
	#define  D(...)  do{}while(0)
	#define  E(...)  do{}while(0)
#endif

/********************Calibration***************************/
#define uT							     31.25

//2D
#define ELLIPSE_2D
#define DEFAULT_NORMAL			        (50 *uT)
#define ONLY_UPDATE_UPPER_LIMIT			(85 * uT)
#define ONLY_UPDATE_LOWER_LIMIT			(30 * uT)

#define DEFAULT_NORMAL_2D				(DEFAULT_NORMAL * 0.6)
#define FIRST_CALIB_NUM_2D			    (16-1)
#define CALIB_INTERVAL_NUM_2D			16
#define MAXNUM_BEFORE_ROLLBACK_2D       48
#define STANDOFF_CHKNUM_2D		        24
#define ONLY_UPDATE_UPPER_LIMIT_2D		(ONLY_UPDATE_UPPER_LIMIT *0.6)
#define ONLY_UPDATE_LOWER_LIMIT_2D		(ONLY_UPDATE_LOWER_LIMIT *0.6)
#define VD_XY_DIFF_MAX_2D				(3*uT)
#define RD_DIFF_MAX_2D					(3*uT)
#define X_Y_MAX_MIN_DIFF_2D				(50 *uT *0.6)
#define MAX_DIFF_RR_VAR_2D				(DEFAULT_NORMAL_2D*1.2)

#endif //__ICAL_CONFIG_H__

