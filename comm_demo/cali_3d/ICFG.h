/*
 * ICFG.h
 *
 *  Created on: 2016Äê6ÔÂ26ÈÕ
 *      Author: sky
 */

#ifndef ICFG_H_
#define ICFG_H_

#define VER_CHK "\n\n - v 3.2\n\n\n"


#define LOG_TAG "QMCX983D"

//#define ICAL_DBG

#ifdef ICAL_DBG
  #ifdef  _WIN32
#define  D(...)  printf(__VA_ARGS__);printf("\n");
#define  E(...)  printf(__VA_ARGS__);printf("\n");
  #else
#define  D(...)  ALOGD(__VA_ARGS__);
#define  E(...)  ALOGE(__VA_ARGS__);
  #endif
#else
  #define  D(...)  ((void)0);
  #define  E(...)  ((void)0);
#endif

#define PI	3.1415926535897932f

#define USE_OFFSET_AVG
/********************Calibration***************************/
#define MAXIMUM_DATA_SAMPLES			50

//#define MAGNETIC_DIPOLE         		 50
#define uT							     31.25

//3D
#define DEFAULT_NORMAL			        (50 *uT)

#define FIRST_CALIB_NUM					(32-1)
#define MAXNUM_BEFORE_ROLLBACK		    40

#define CALIB_INTERVAL_NUM_DEFAULT		16
#define STANDOFF_CHKNUM_3D				16
#define ONLY_UPDATE_UPPER_LIMIT			(55 * uT)
#define ONLY_UPDATE_LOWER_LIMIT			(35 * uT)
#define WRITE_UPPER_LIMIT				(43 * uT)
#define WRITE_LOWER_LIMIT				(53 * uT)
#define ONLY_UPDATE_DIFFSAMECOUNT		(2)
#define X_Y_MAX_MIN_DIFF				(50 *uT)
#define VD_DIFF_MAX						(5*uT)
#define VD_XY_DIFF_MAX					(5*uT)
#define RD_DIFF_MAX						(5*uT)

#endif /* ICFG_H_ */
