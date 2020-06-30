#ifndef __INTELLIGENT_CALIBRATION_2D_H__
#define __INTELLIGENT_CALIBRATION_2D_H__

/*************************************************************************
*	The purpose of this Library is to provide the ability to do active
*	magnetic field calibration as a background process that doesn't
*	involve active (conscious) participation by the User.
*
*	As the environment that the magnetic sensors are in chances, it
*	is necessary to correct out disturbances that might interfere with
*	the true nature of the Earth's magnetic field. These disturbances
*	result in errors being injected into Heading/Navigation calculations
*	and thus erroneous Heading information. This library attempts to
*	correct out these disturbances by generating calibration (correction)
*	coefficients that are applied to the raw magnetic data before being
*	used for navigation purposes.
*
*	Using the calibration library is meant to be simple and operate
*	more or less in the background. On system power up, call any one of
*	three initialization routines that will put the calibration system
*	into start mode. Once this bas been done, then data collection can
*	begin at the discretion of the User/platform or when data is
*	available. The library tests the data for quality and if it passes,
*	then the data is added to a 24 sample buffer. When enough samples
*	(24) have been collected, then the calibration compute function can
*	be called to try and generate new coefficients to correct out the
*	new localized disturbances. This process is meant to be continuous
*	until the system power is turned off or the system chooses to stop
*	generating new coefficients.
*
*	The general flow of operation is something like this:
*
*					ical_Init(...);
*					:
*					:
*					Loop:
*						ical_collectDataPoint(...);
*						if (ical_readyCheck())
*							rs = ical_computeCalibration(pCoef);
*							if (rs == ICAL_SUCCESS)
*								ofsX = pCoef->Offset[0];
*								ofsY = pCoef->Offset[1];
*								ofsZ = pCoef->Offset[2];
*								Kxx	 = pCoef->Matrix[0][0];
*								Kyy	 = pCoef->Matrix[1][1];
*								Kzz	 = pCoef->Matrix[2][2];
*					End Loop:
*					:
*					:
*					:
*					// Some where else in your code
*					:
*					calMagX	= (rawMagX - ofsX) * Kxx;
*					calMagY	= (rawMagY - ofsY) * Kyy;
*					calMagZ	= (rawMagZ - ofsZ) * Kzz;
*
*					heading = genHeading(calMagX,calMagY,calMagZ,....);
*
*
*************************************************************************/
//#include <stdlib.h>
#ifdef  _WIN32
#include <time.h>
#include <windows.h> 
#include <stdio.h>
#include <stdint.h>
//#include "qst_sensors.h"

#else
#include <dlfcn.h>
#include <cutils/properties.h>
#include <cutils/log.h>
#include <sys/time.h>
#endif 
#include <math.h>
//#include "kal_release.h"

/********************************************************************
							Definitions
*******************************************************************/
#define ICAL2D_SUCCESS		 1
#define ICAL2D_FAILURE		-1

#define bEnable_ELLIPSE_2D 1

#define SENSOR_TYPE_MAGNETIC_FIELD					2
#define SENSOR_TYPE_ACCELEROMETER                    1


// Internal Type Definitions
typedef char INT8;
typedef unsigned char UINT8;
typedef short INT16;
typedef unsigned short UINT16;
typedef long INT32;
typedef unsigned long UINT32;
typedef float REAL;


typedef signed char int8_t;
typedef unsigned char   uint8_t;
typedef short  int16_t;
typedef unsigned short  uint16_t;
typedef int  int32_t;
typedef unsigned   uint32_t;
typedef long long  int64_t;
typedef unsigned long long   uint64_t;

#ifndef  _WIN32
struct timeval {
  long tv_sec;
  long tv_usec;
};
#endif
typedef struct
{
	int	Offset[2];		// Magnetic Hard Iron x, y, z offsets
	int rr;
} TRANSFORM_2D_T;


typedef struct
{
	 REAL T;
	 REAL E_2D_ofsX;
	 REAL E_2D_ofsY;
	 REAL CosT;
	 REAL SinT;
	 REAL R1;
	 REAL R2;
	 int rr;
}EllipsePram;

int CUSTOMER_SET_EllipsePram(EllipsePram *pram);

void CUSTOMER_GET_CLIABPRAM(int *accuracy, EllipsePram *pram);

int process(REAL *mMagneticData);

/********************************************************************
							Funtion Prototypes
********************************************************************/

void mcal(short rok);

/************************************************************************
*	This function is used to initialize or reset the Magnetic Field
*	Calibration background running algorithm. Always call this
*	function at either power-up or when it is desired to do a reset
*	to flush the system clean of consecutive high anomally calibrations.
*
*	Inputs:
*
*	currentTimeMSec - Initialize the algorithm with the current time
*					  in milliseconds.
*
*	Normalizer		- This value is used to normalize the calibration
*					  coefficients to the range of the collected data.
*					  Example: data in range +/512 should use 512,
*					  +/1024 should use 1024, etc. Default is 4096.
*
*	Note 1:	One of the data decrimination factors is time and thus the
*			need for an initial value. Time wise, data is seperated
*			by a half second. Data with intervals shorter than this
*			will be rejected.
*
*/
void ical2D_Init(long currentTimeMSec,unsigned short Normalizer);

void EllipseRemovePoint();

void ical2D_shiftData(int length);

/*************************************************************************
*	This function is used to pass raw (uncalibrated) magnetic data into the
*	calibration's internal data storage buffers. It also does a quality
*	check to determine of the 3D data point is good to use.
*
*	Inputs:
*
*	rawMagX:			- Raw magnetic data value (+- range) sensed from the X axes.
*
*	rawMagY:			- Raw magnetic data value (+- range) sensed from the Y axes.
*
*	rawMagZ:			- Raw magnetic data value (+- range) sensed from the Z axes.
*
*	currentTimeMsec:	- Time stamp in milliseconds for the data point. (NEEDED).
*
*
*	Return Value:		- 0: Data point was rejected.	1: Data point was used.
*
*
*	Note 1:				- The function ical_readCheck() should always be called
*						  after this function is called to determine if enough
*						  data has been collected to generate calibration
*						  coefficients.
*
*/
short ical2D_collectDataPoint(short rawMagX, short rawMagY, long currentTimeMsec);


/*************************************************************************
*	This function is used to generated magnetic calibration coefficients. It
*	should be called after the ical_readyCheck() function has indicated that
*	enough qualified data (24) have been collected.
*
*	Output:
*
*		xfp:	- Structure that contains both the magnetic Hard Iron offsets
*				  and the Soft Iron gain corrections.
*
*		Offsets - Index range: 0-2, which corresponds to X, Y, and Z.
*
*		Matrix	- Main diagonal contains the gains of interest.
*				  00,11,22 correspond to Kxx, Kyy, and Kzz.
*
*	Return Values:
*
*		ICAL_SUCCESS				- Calibration coefficients were generated.
*		ICAL_NO_OUTPUT_STRUCTURE	- No return (output) argument was provided.
*		ICAL_BAD_DATA				- Data set used was degenerate (no good).
*		ICAL_HARDIRON_FAILURE		- Solution failed due to over powering
*									  Hard Iron effects.
*		ICAL_FAILED_TO_CONVERGE		- Solution failed to converge for whatever
*									  reason.
*
*	Note 1:		- Usage: User must apply the coefficients to correct the raw
*				  magnetic data.
*
*				  MagX	= (rawMagX - ofsX) * Kxx
*				  MagY	= (rawMagY - ofsY) * Kyy
*				  MagZ	= (rawMagZ - ofsZ) * Kzz
*
*/
short ical2D_computeCalibration(TRANSFORM_2D_T* xfp);


/************************************************************************
*	This function is used to determine if enough data has been collected
*	to try and generate calibration coefficients. It is important
*	to call this function after every call of the ical_collectDataPoint()
*	function.
*
*	Return Value:	1 - Enough (24) data samples have been collected to
*						try and generated calibration coefficients.
*
*					0 - Insufficient data has been collected by the
*						algorithm for the generation of calbration
*						coefficients.
*
*/
short ical2D_readyCheck();

void ical2D_clearFarPoint(int rr, int ofx, int ofy);

#endif //__INTELLIGENT_CALIBRATION_2D_H__
