/*
 * ICAL.c
 *
 *  Created on: 2016Äê6ÔÂ17ÈÕ
 *      Author: sky
 */

/*
*	Module:	MagFieldCal.c
*	Name:	Tom Judd & Bruce R. Graham
*	Date:	7/28/2010
*
*	Description: This library provides an algorithm that can
*	generate correction coefficients for both Hard and Soft
*	Iron distrubances by continually running in the background,
*	filtering raw 3D data points for a set of 24 data samples to
*	work with. Once a solution has been generated, it is outputted
*	for the User to apply to raw magnetic data. After every
*	generation of calibration coefficients, the algorithm starts
*	looking for data to build up a new sample set to generate
*	another solution. This process will continue as long as the
*	system in which it is embedded in is running.
*
*	Version: 1.0
*/

#include "ICAL.h"
#include "ICFG.h"

#ifdef  _WIN32
#include <time.h>
#include <windows.h> 
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#else
#include <dlfcn.h>
#include <cutils/properties.h>
#include <cutils/log.h>
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#endif

// Internal Definitions
#define ENS3D_STANDOFF		0.6					// Minimum distance between samples in radians

#define __AAA__ 1
#ifdef __AAA__
#define PARA_3D            	4
#elif __ABC__
#define PARA_3D            	6
#endif

#define DIM_A				(PARA_3D)
// Internal Structural Type Definitions
typedef struct
{
   INT16 mag[MAXIMUM_DATA_SAMPLES][3];
} CAL_ENSEMBLE_T;

typedef struct
{
   INT16 trialMag[3];
   INT16 vecDiff[3];
   INT16 nsamps;		//Count of collection data
   INT16 standoffCounts;
   INT32 sampleTime;
} ICAL_MGR_T;

typedef struct
{
   REAL MatA[DIM_A][DIM_A];
   REAL invMatA[DIM_A][DIM_A];

} MA_INVERT_T;

#ifndef false
	#define false 		0
#endif
#ifndef true
	#define true 		1
#endif

// Internal Variable Definitions
static ICAL_MGR_T			icalMgr;
static CAL_ENSEMBLE_T		icalEns;
static MA_INVERT_T			ma;

static REAL					*Jpt[PARA_3D];
static REAL					JTJ[PARA_3D][PARA_3D];
static REAL					JTK[PARA_3D];
static REAL					Jout[PARA_3D];
static REAL					magX[MAXIMUM_DATA_SAMPLES] = {0};
static REAL					magY[MAXIMUM_DATA_SAMPLES] = {0};
static REAL					magZ[MAXIMUM_DATA_SAMPLES] = {0};
static REAL					magXX[MAXIMUM_DATA_SAMPLES] = {0};
static REAL					magYY[MAXIMUM_DATA_SAMPLES] = {0};
static REAL					magZZ[MAXIMUM_DATA_SAMPLES] = {0};
static REAL					rr[MAXIMUM_DATA_SAMPLES]   = {0};

REAL Gain = 0;

// 3D
static TRANSFORM_T seedCal = {0};
static TRANSFORM_T newCal = {0};
char *wkCoefficients = "data/misc/calib.dat";
INT16 ofsX = 0, ofsY = 0, ofsZ = 0;
INT16 Kxx = 1562, Kyy = 1562, Kzz = 1562;

static INT16 AvgMagR = (INT16)DEFAULT_NORMAL;
INT8 lastCalibCount = 0;

static INT8 lastStatus  = ACCURACY_UNRELIABLE;
static INT8 curAccuracy = ACCURACY_UNRELIABLE;
INT16 lastR = 0;
INT16 rd = 0;
INT8 diffSameCount = 0;
INT16 IsFirstCalAfterReboot = 1;

static UINT8 nCheckHorizontal = 0;
static UINT8 IsHorizontal = 0;

#ifdef USE_OFFSET_AVG
#define OFS_AVG_N    4
INT16 ofsX_old = 0, ofsY_old = 0, ofsZ_old = 0;
INT16 offset_buff[3][4] = {{0,0,0,0}};
INT8 offset_avgN = 0;
#endif
REAL ipitch = 0;
REAL iroll = 0;
/////////////////////////////////////////////////////////////////////
///////////////// Internal Function Prototypes //////////////////////
/////////////////////////////////////////////////////////////////////
INT64 get_time_in_nanosec(void);
void clear_offset(void);
void vec_diff(INT16 *v0, INT16 *v1, INT16 *diff);
INT32 vec_magnitude(INT16* vec);

INT16 ical_qualify(long currentTimeMsec, INT16 rawMagX, INT16 rawMagY, INT16 rawMagZ);
void ical_addSample(INT16* mag, INT16 sampIX);
void ical_shiftData(INT16 length);

INT16 em_invert(INT16 size);
void  em_average(INT16(*xyz)[3], REAL *avg, INT16 *iavg, INT16 npts);
void  em_demean(INT16(*xyz)[3], INT16 *avg, INT16 npts);
INT16 em_center(INT16(*xyz)[3], INT16 *outrxyz, INT16 npts);
int storeCalibrationToFile(TRANSFORM_T theCal, char* fileName);
int getCalibrationFromFile(TRANSFORM_T* pCal, char* fileName);
////////////////////////////////////////////////////////////////////
//////////////// START OF EXTERNAL LIBRARY CODE ////////////////////
////////////////////////////////////////////////////////////////////

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

#ifdef _WIN32
INT64 get_time_in_nanosec(void) {
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	return (sys.wSecond * 1000LL + sys.wMilliseconds) * 1000000LL;
}
#else
INT64 get_time_in_nanosec(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000000000LL + tv.tv_usec * 1000LL;
}
#endif

void *mMemset(void *ptr, INT16 value, UINT16 num)
{
	INT8 *xs = (INT8 *)ptr;
	while (num --)
		*xs++ = value;
	return ptr;
}

REAL mFabs(REAL a)
{
	return (a<0) ? -a : a;
}

INT16 mSabs(INT16 a)
{
	return (a<0) ? -a : a;
}


REAL mSqrt(REAL a)
{
	REAL x = a, x_ = 0;
    while(mFabs(x_ - x) > 1e-10)
    {
        x_ = x;
        x = 0.5*(x+a/x);
    }
    return x;
}

/*
	more faster than iteration
*/
REAL InvSqrt(REAL x)
{
	REAL xhalf = 0.5f*x;
	int i = *(int*)&x; // get bits for floating VALUE 
	i = 0x5f375a86- (i>>1); // gives initial guess y0
	x = *(REAL*)&i; // convert bits BACK to float
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
	x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy

	return 1/x;
}

void ical_Init(long currentTimeMSec)
{
	D("ical_Init");
	// Zero the managers
	mMemset(&icalMgr, 0, sizeof(ICAL_MGR_T));
	mMemset(&icalEns, 0, sizeof(CAL_ENSEMBLE_T));

	// Initialize the sample time tick
	icalMgr.sampleTime	= currentTimeMSec;

	// Initialize the stand off counts
	icalMgr.standoffCounts	= (INT16)(ENS3D_STANDOFF * 562.5);

	IsFirstCalAfterReboot = 1;

	lastCalibCount= 0;

#ifdef USE_OFFSET_AVG
	clear_offset();
#endif

}

void removePointAt(UINT8 num)
{
	UINT8 i=0;
	//D("ical_clearFarPoint, removePointAt - %d", num);
	if(num>=icalMgr.nsamps || num==-1)
		return;

	for(i=0; i<icalMgr.nsamps;i++ )
	{
		if(i>num)
		{
			icalEns.mag[i-1][0] = icalEns.mag[i][0];
			icalEns.mag[i-1][1] = icalEns.mag[i][1];
			icalEns.mag[i-1][2] = icalEns.mag[i][2];
		}
	}
	icalMgr.nsamps--;
}

/*
	delete 2 points,max value and min value from the offset
*/
void ical_clearFarPoint(int rr,int ofx, int ofy, int ofz)
{
	UINT8 i=0;
	D("ical_clearFarPoint, N = %d, INT -> [%d, %d, %d]", icalMgr.nsamps, ofx, ofy, ofz);
	if(icalMgr.nsamps > FIRST_CALIB_NUM){
		UINT8 maxIndex = 0;
		UINT8 minIndex = 0;

		REAL maxVar, minVar;
		maxVar = minVar =mSqrt(	(icalEns.mag[0][0]-ofx)*(icalEns.mag[0][0]-ofx)+
								(icalEns.mag[0][1]-ofy)*(icalEns.mag[0][1]-ofy)+
								(icalEns.mag[0][2]-ofz)*(icalEns.mag[0][2]-ofz));

		for(i=1; i<icalMgr.nsamps;i++ ){
			REAL var = mSqrt(	(icalEns.mag[i][0]-ofx)*(icalEns.mag[i][0]-ofx)+
								(icalEns.mag[i][1]-ofy)*(icalEns.mag[i][1]-ofy)+
								(icalEns.mag[i][2]-ofz)*(icalEns.mag[i][2]-ofz));
			if(var>maxVar){
				maxVar=var;
				maxIndex=i;
			}
			if(var<minVar){
				minVar=var;
				minIndex=i;
			}
		//	D("ical_clearFarPoint, n = %d, var = %f,  _[%d, %d, %d]", i, var, icalEns.mag[i][0], icalEns.mag[i][1], icalEns.mag[i][2]);
		}
		D("ical_clearFarPoint_ maxVar = %f _(%d), minVar = %f _(%d)", maxVar, maxIndex, minVar, minIndex);
		/*add @20160711 by andy*/
		if(maxIndex > minIndex){
			removePointAt(maxIndex);
			removePointAt(minIndex);
		}else{
			removePointAt(minIndex);
			removePointAt(maxIndex);
		}
	}
}

//Romove add point > 1.2 rr
void ical_clearFarPoint2(int rr, int ofx, int ofy, int ofz)
{
	INT16 i=0, j=0;
	INT16 tmpIdx[MAXIMUM_DATA_SAMPLES] = {0};
	INT16 tmpCounter = 0;
	//D("ical_clearFarPoint2, N = %d, INT -> [%d, %d, %d]", icalMgr.nsamps, ofx, ofy, ofz);
	if(icalMgr.nsamps > FIRST_CALIB_NUM-4)
	{
		for(i=0; i<icalMgr.nsamps;i++ )
		{
			REAL var = mSqrt(	(icalEns.mag[i][0]-ofx)*(icalEns.mag[i][0]-ofx)+
								(icalEns.mag[i][1]-ofy)*(icalEns.mag[i][1]-ofy)+
								(icalEns.mag[i][2]-ofz)*(icalEns.mag[i][2]-ofz));
			if(var>rr*1.2)
			{
				tmpIdx[tmpCounter] = i;
				tmpCounter++;
			}
		}
		for(j=0;j<tmpCounter;j++)
		{
			if(j ==0)
				removePointAt(tmpIdx[j]);
			else
				removePointAt(tmpIdx[j]-j);
		}
	}
}

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
INT16 ical_collectDataPoint(INT16 rawMagX, INT16 rawMagY, INT16 rawMagZ, long currentTimeMsec)
{
	if (0 == ical_qualify(currentTimeMsec, rawMagX, rawMagY, rawMagZ))
	{
		E("ical_collectDataPoint ical_qualify fail!");
		return false;
	}
	ical_addSample(&icalMgr.trialMag[0], icalMgr.nsamps);
	return true;
}

INT16 ComputeRR(INT16 *irxyz)
{
	int i =0;
	int ofs_X = irxyz[1];
	int ofs_Y = irxyz[2];
	int ofs_Z = irxyz[3];
	int K1 =irxyz[4];
	int K2 =irxyz[5];
	int K3 =irxyz[6];
	REAL RR = 0;
	REAL RR_SUM = 0;
	REAL tempXYZ[3] = {0};
	for(i=0; i<icalMgr.nsamps; i++)
	{
//	   		D("ical_3D ComputeCalibration %d \t- [%d, %d, %d]\n", i, icalEns.mag[i][0], icalEns.mag[i][1], icalEns.mag[i][2]);
	   		tempXYZ[0] =  (icalEns.mag[i][0] - ofs_X) * K1 / DEFAULT_NORMAL;
	   		tempXYZ[1] =  (icalEns.mag[i][1] - ofs_Y) * K2 / DEFAULT_NORMAL;
	   		tempXYZ[2] =  (icalEns.mag[i][2] - ofs_Z) * K3 / DEFAULT_NORMAL;

	   		RR = tempXYZ[0] *tempXYZ[0] + tempXYZ[1]*tempXYZ[1]+ tempXYZ[2]*tempXYZ[2];
	   		RR = mSqrt(RR);
//	   		D("ICAL_3D sqrt(RR) = %f\n",RR);
	   		RR_SUM +=RR;
	}
	RR = RR_SUM / (icalMgr.nsamps);
	D("ICAL_3D RR = %d\n",(int)RR);
	return (int)RR;
}

INT16 ical_computeCalibration(TRANSFORM_T* xfp)
{
	INT16	irxyz[7] = {0};
   INT16 ii;
   if (!xfp)
	   return ICAL_FALSE;

	for(ii=0; ii<icalMgr.nsamps; ii++){
		D("ICAL_3D ComputeCalibration %d \t- [%d, %d, %d]\n", ii, icalEns.mag[ii][0], icalEns.mag[ii][1], icalEns.mag[ii][2]);
	}

   if (1 != em_center(&icalEns.mag[0], &irxyz[0], icalMgr.nsamps))
   		return ICAL_FALSE;
	
  D("ical_3D  Hard_Iron - {%d, %d, %d, %d}, (%d)", irxyz[0], irxyz[1], irxyz[2], irxyz[3], icalMgr.nsamps);
	xfp->Offset[0] = irxyz[1];
	xfp->Offset[1] = irxyz[2];
	xfp->Offset[2] = irxyz[3];
	
	xfp->Matrix[0] = 0;
	xfp->Matrix[1] = 0;
	xfp->Matrix[2] = 0;	

#if __AAA__
	xfp->Matrix[0] = 1;
	xfp->Matrix[1] = 1;
	xfp->Matrix[2] = 1;
	xfp->rr = (int)irxyz[0];
#else
	xfp->Matrix[0] = irxyz[4] ;
	xfp->Matrix[1] = irxyz[5] ;
	xfp->Matrix[2] = irxyz[6] ;
	xfp->rr = ComputeRR(irxyz);
#endif

	D("ICAL_3D offset-%d [%d,%d,%d],scal [%d,%d,%d],R = %d",icalMgr.nsamps,xfp->Offset[0],xfp->Offset[1],xfp->Offset[2],xfp->Matrix[0],xfp->Matrix[1],xfp->Matrix[2],xfp->rr);
	return ICAL_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
////////////////// START OF INTERNAL LIBRARY CODE ////////////////////
//////////////////////////////////////////////////////////////////////

/*************************************************************************
*	This function is used to qualify a magnetic data point in both time and
*	direction/displacement.
*
*	Inputs:
*
*		currentTimeMsec		- Time stamp in milliseconds for 3D magnetic data point.
*
*		rawMagX				- Raw magnetic field strength in the X axis direction.
*
*		rawMagY				- Raw magnetic field strength in the Y axis direction.
*
*		rawMagZ				- Raw magnetic field strength in the Z axis direction.
*
*
*	Return Value:			- 0: Data is rejected.	1: Data is valid.
*
*/
INT16 ical_qualify(long currentTimeMsec, INT16 rawMagX, INT16 rawMagY, INT16 rawMagZ)
{
	INT16 ii;

	//avoid mcal timestap not sync with event timestamp
	if (icalMgr.nsamps == 0)
	{
		icalMgr.trialMag[0]		= rawMagX;
		icalMgr.trialMag[1]		= rawMagY;
		icalMgr.trialMag[2]		= rawMagZ;
		icalMgr.sampleTime		= currentTimeMsec;		
		return true;		
	}

	/*
		check the absolute value between currentTimeStamp and the TimeStamp of the last points ,should be more than 15ms
	*/

	if (currentTimeMsec - icalMgr.sampleTime < 15 && currentTimeMsec - icalMgr.sampleTime > 0)
	{
		D("ical_qualify, return 0 - 1");
		return false;
	}
	D("ical_qualify, raw: %d %d %d \n",rawMagX,rawMagY,rawMagZ);
	icalMgr.trialMag[0]		= rawMagX;
	icalMgr.trialMag[1]		= rawMagY;
	icalMgr.trialMag[2]		= rawMagZ;
	icalMgr.sampleTime		= currentTimeMsec;

	ii  = icalMgr.nsamps;

	/* 
		check the distance between the last point collected and current point 
	*/
	vec_diff(&(icalMgr.trialMag[0]), &(icalEns.mag[ii-1][0]), &(icalMgr.vecDiff[0]));
	D("ical_qualify3D, vec = %d, standoffCounts = %d", vec_magnitude(&icalMgr.vecDiff[0]), icalMgr.standoffCounts);

	if (vec_magnitude(&icalMgr.vecDiff[0]) < icalMgr.standoffCounts )
	{
		D("ical_qualify, return 0 - 2");
		return false;
	}


	/* 
		check the distance between all points collected and current point 
	*/
	while (ii > 0)
	{
		ii--;
		vec_diff(&(icalMgr.trialMag[0]), &(icalEns.mag[ii][0]), &(icalMgr.vecDiff[0]));
		D("ical_qualify3D, vec = %d, standoffCounts = %d", vec_magnitude(&icalMgr.vecDiff[0]), icalMgr.standoffCounts);

		if(vec_magnitude(&icalMgr.vecDiff[0]) < (icalMgr.standoffCounts * 0.5) )
		{
			D("ical_qualify, vecDiff too small ! return 0 - 4");
			return false;
		}

		if(vec_magnitude(&icalMgr.vecDiff[0]) > 6000 ) //6000
		{
			D("ical_qualify, vec too big ! return 0 - 3");
			ical_Init(currentTimeMsec);
			return false;
        }
	}
	
	return true;
}

/*************************************************************************
*	This function simply adds the magnetic data values to the
*	internal data buffers and increments index counter.
*
*	Inputs:
*
*		mag		- Pointer to array containing magnetic data.
*
*		acc		- Pointer to array containing usage flags.
*
*		sampIX	- Current location to place data into array.
*
*/
void ical_addSample(INT16* mag, INT16 sampIX)
{
	INT16 ii;
	for (ii = 0;ii < 3;ii++)
		icalEns.mag[sampIX][ii] = mag[ii];
	icalMgr.nsamps	= icalMgr.nsamps + 1;

	// Circle around if exceed buffer size
	if (icalMgr.nsamps >= MAXIMUM_DATA_SAMPLES)
		icalMgr.nsamps = 0;
}

/*
 *
 * Left shift data
 *
 * */
void ical_shiftData(INT16 length)
{
	INT16 i = 0;
	//D("\n\n********ical_shiftData, nsamps = %d, length = %d", icalMgr.nsamps, length);
	if(icalMgr.nsamps<=length || length>MAXNUM_BEFORE_ROLLBACK)
		return;

	for(i=0; i<(icalMgr.nsamps - length); i++){
		icalEns.mag[i][0] = icalEns.mag[i+length][0];
		icalEns.mag[i][1] = icalEns.mag[i+length][1];
		icalEns.mag[i][2] = icalEns.mag[i+length][2];
	}

	icalMgr.nsamps -= length;

	for(i=0; i<(MAXIMUM_DATA_SAMPLES - icalMgr.nsamps); i++){
		icalEns.mag[i+icalMgr.nsamps][0] = icalEns.mag[i+icalMgr.nsamps][1] = icalEns.mag[i+icalMgr.nsamps][2] = 0;
	}

}

/////////////////////////////////////////////////////////////////////
///////////////////// EM MATH FUNCTIONS /////////////////////////////
/////////////////////////////////////////////////////////////////////

// function to invert a matrix by Gaussian elimination
//
//    Maximum Matrix size: DIM_A
//
//    MatA is reduced to identity matrix
//    invMatA contains inverse matrix
//
//   Return Values:
//		   0 = Successful inversion
//        -1 = Error: Singular Matrix
//
INT16 em_invert(INT16 size)
{
   REAL alpha, beta;
   INT16 ii, jj, kk;

   if( (size < 2) || (size > DIM_A))
	   return(false);
   // initialize the reduction matrix--> unity matrix
   for(ii = 0; ii < size; ii++)
   {
      for(jj = 0; jj < size; jj++)
      {
         ma.invMatA[ii][jj] = 0.0f;

      }
      ma.invMatA[ii][ii] = 1.0f;
   }

   // do the reduction
   for(ii = 0; ii < size; ii++)
   {
      alpha = ma.MatA[ii][ii];
      if(alpha == 0.0f )
		  return(false); // -------------------> bail out

      for(jj = 0; jj < size; jj++)
      {
         ma.MatA[ii][jj]    = ma.MatA[ii][jj] / alpha;
         ma.invMatA[ii][jj] = ma.invMatA[ii][jj] / alpha;

      } // END jj loop

      for(kk = 0; kk < size; kk++)
      {
         if((kk - ii) != 0)
         {
            beta = ma.MatA[kk][ii];
            for(jj = 0; jj < size; jj++)
            {
               ma.MatA[kk][jj]    = ma.MatA[kk][jj] - beta * ma.MatA[ii][jj];
               ma.invMatA[kk][jj] = ma.invMatA[kk][jj] - beta * ma.invMatA[ii][jj];
            }
         }
      } // END kk loop
   } // END ii loop
   return true;
}

// Find the average of each component, report both integer and real versions
void em_average(INT16 (*xyz)[3], REAL *avg, INT16 *iavg, INT16 npts)
{
   INT16 ix;
   INT32 tempL[3] = {0,0,0};

   for(ix=0;ix<npts;ix++)
   {
      tempL[0] += xyz[ix][0];
      tempL[1] += xyz[ix][1];
      tempL[2] += xyz[ix][2];
   }

   iavg[0]=(INT16)(tempL[0]/npts);
   iavg[1]=(INT16)(tempL[1]/npts);
   iavg[2]=(INT16)(tempL[2]/npts);

// populate truncated floating point average
   avg[0]=iavg[0];
   avg[1]=iavg[1];
   avg[2]=iavg[2];
}

// subtract the average from the data
void em_demean(INT16 (*xyz)[3], INT16 *avg, INT16 npts) {
	INT16 ix;
	for (ix = 0; ix < npts; ix++) {
		xyz[ix][0] -= avg[0];
		xyz[ix][1] -= avg[1];
		xyz[ix][2] -= avg[2];
		//D("em_demean xyz[%d %d %d]\n",xyz[ix][0],xyz[ix][1],xyz[ix][2]);
	}
}

// prepare data for hard iron estimate:
// subtract average, move data to real arrays, and initialize pointers
void em_prepare_data(INT16 (*xyz)[3], INT16 *iavg, INT16 npts) {
	INT16 ix =0;
	REAL avg[3]={0};

	// subtract average off of integer input data
	em_average(xyz, &avg[0], iavg, npts);
	//D("em_prepare_data avg[%lf %lf %lf] iavg[%d %d %d]\n",avg[0],avg[1],avg[2],iavg[0],iavg[1],iavg[2]);
	em_demean(xyz, iavg, npts);

	for (ix = 0; ix < npts; ix++)
	{
#if __ABC__
		magXX[ix] = (REAL) xyz[ix][0] * (REAL) xyz[ix][0];
		magYY[ix] = (REAL) xyz[ix][1] * (REAL) xyz[ix][1];
		magZZ[ix] = (REAL) xyz[ix][2] * (REAL) xyz[ix][2];
		magX[ix] = (REAL) xyz[ix][0];
		magY[ix] = (REAL) xyz[ix][1];
		magZ[ix] = (REAL) xyz[ix][2];							 
		//D("em_prepare_data magXX[%lf] magYY[%lf] magZZ[%lf] magX[%lf] magY[%lf] magZ[%lf]\n",magXX[ix],magYY[ix],magZZ[ix],magX[ix],magY[ix],magZ[ix]);
#else
		magX[ix] = (REAL) xyz[ix][0];
		magY[ix] = (REAL) xyz[ix][1];
		magZ[ix] = (REAL) xyz[ix][2];
		rr[ix]   = (REAL) (magX[ix] * magX[ix] + magY[ix] * magY[ix]  + magZ[ix] * magZ[ix]);
#endif
	}
#if __ABC__
	Jpt[0] = &magXX[0];
	Jpt[1] = &magYY[0];
	Jpt[2] = &magZZ[0];
	Jpt[3] = &magX[0];
	Jpt[4] = &magY[0];
	Jpt[5] = &magZ[0];
#else
	Jpt[0] = &rr[0];
	Jpt[1] = &magX[0];
	Jpt[2] = &magY[0];
	Jpt[3] = &magZ[0];
#endif		
}

// J transpose times J
// This could be formed directly in ma.MatA to save a copy
void em_JTJ(INT16 npts) {
	INT16 jrr, jcc, jkk;

	for (jrr = 0; jrr < PARA_3D; jrr++) {
		for (jcc = 0; jcc < PARA_3D; jcc++) {
			JTJ[jrr][jcc] = 0.0f;
			for (jkk = 0; jkk < npts; jkk++) {
				JTJ[jrr][jcc] += Jpt[jrr][jkk] * Jpt[jcc][jkk];
			}
			//D("em_JTJ JTJ[%lf]\n",JTJ[jrr][jcc]);
		}
	}
}
// move data to matrix for inversion
void em_moveData(void) {
	INT16 jrr, jcc;
	for (jrr = 0; jrr < PARA_3D; jrr++){
		for (jcc = 0; jcc < PARA_3D; jcc++){
			ma.MatA[jrr][jcc] = JTJ[jrr][jcc];
			//D("em_moveData ma.MatA[%d][%d] %0.15lf\t\n",jrr,jcc,JTK[jrr]);
		}

	}
			
}

// vector K is a column of 1's
// so J'*K (J transpose times K --> JTK) is a column vector
// each of whose elements is the sum of one of the rows of J'
void em_JTK(INT16 npts) {
	INT16 jrr, jkk;
	for (jrr = 0; jrr < PARA_3D; jrr++) {
		JTK[jrr] = 0.0f;
		for (jkk = 0; jkk < npts; jkk++) {
			JTK[jrr] += Jpt[jrr][jkk];
		}
		//D("em_JTK JTK[%d] %0.15lf\n",jrr,JTK[jrr]);
	}
}

// inverse of J transpose times J ("JTJinv")
// times J'K
void em_JTJinvJK(void){
	INT16 jrr, jkk;
	for (jrr = 0; jrr < PARA_3D; jrr++) {
		Jout[jrr] = 0.0;
		for (jkk = 0; jkk < PARA_3D; jkk++) {
			Jout[jrr] += ma.invMatA[jrr][jkk] * JTK[jkk];
		}
		//D("em_JTJinvJK Jout[%d] %0.15lf\n",jrr,Jout[jrr]);
	}
}

// A(X^2 +Y^2 + Z^2) + Bx + Cy + D = 1 for sphere
// A is in inv[0]
// B is in inv[1] ...
// ...
//
// Return radius in inv[0]
// center offsets in inv[1..3] for sphere
REAL em_results(REAL *inv ,INT16 *ioutrxyz,INT16 *iavg,INT16 (*xyz)[3],INT16 npts)
{
#if __ABC__
	int i =0;
	REAL tempdt=0;
	REAL sum_dt=0;	
	sum_dt=0;
	for(i=0;i<npts;i++)
	{
		tempdt = inv[0]*xyz[i][0]*xyz[i][0] + inv[1]*xyz[i][1]*xyz[i][1] +inv[2]*xyz[i][2]*xyz[i][2] +inv[3]* xyz[i][0] +inv[4]* xyz[i][1]+inv[5]* xyz[i][2] -1;
		tempdt = tempdt*tempdt;
		sum_dt = sum_dt+tempdt;
	}

	inv[1] =inv[1] - 0.5 * sum_dt*inv[1];
	inv[2] =inv[2] + 0.5 * sum_dt*inv[2];
	inv[4] =inv[4] - 0.5 * sum_dt*inv[4];
	inv[5] =inv[5] + 0.5 * sum_dt*inv[5];

	ioutrxyz[1]=-inv[3]/(2.0f * inv[0]);
	ioutrxyz[2]=-inv[4]/(2.0f * inv[1]);
	ioutrxyz[3]=-inv[5]/(2.0f * inv[2]);

	D("ICAL_3D ofs_X[0] = %d,ofs_Y[1] = %d,ofs_Z[2] = %d\n",ioutrxyz[1],ioutrxyz[2],ioutrxyz[3]);

	Gain =  1 + 0.25 * inv[3] * inv[3] / inv[0] + 0.25 * inv[4] * inv[4] / inv[1] + 0.25 * inv[5] * inv[5] / inv[2];

	D("ICAL_3D Gain = %0.15f\n",Gain);

	ioutrxyz[4] =DEFAULT_NORMAL *DEFAULT_NORMAL * InvSqrt(inv[0] / Gain);
	ioutrxyz[5] =DEFAULT_NORMAL *DEFAULT_NORMAL * InvSqrt(inv[1] / Gain);
	ioutrxyz[6] =DEFAULT_NORMAL *DEFAULT_NORMAL * InvSqrt(inv[2] / Gain);

	D("ICAL_3D Kx = %d,Ky = %d,Kz = %d \n",ioutrxyz[4],ioutrxyz[5],ioutrxyz[6]);
	return 1;

#else
	REAL RR;
	REAL ofs[3];
	D("ICAL_3D inv[0] = %0.15f ,inv[1] = %0.15f ,inv[2] = %0.15f,inv[3] = %0.15f, [%0.15f]\n",inv[0],inv[1],inv[2],inv[3],1/inv[0]);

	ofs[0] = -inv[1]/(2.0f * inv[0]);
	ofs[1] = -inv[2]/(2.0f * inv[0]);
	ofs[2] = -inv[3]/(2.0f * inv[0]);

	RR = mSqrt(ofs[0] * ofs[0] + ofs[1] * ofs[1] + ofs[2] * ofs[2] + 1/inv[0]);
	D("ICAL_3D RR = %0.15f\n",RR);

	inv[0]=RR;
	inv[1]=ofs[0];
	inv[2]=ofs[1];
	inv[3]=ofs[2];
	return RR;
#endif	   
}

// put the average back into the data
// report the results out as integers
void  em_restore_data(INT16 (*xyz)[3], INT16 *inavg, REAL *results, INT16* ioutrxyz,INT16 npts)
{
   INT16 ii;
   // correct center with average. note results index starts at 1
   results[1]+=inavg[0];
   results[2]+=inavg[1];
   results[3]+=inavg[2];
   // put average back into data
   for(ii=0;ii<npts;ii++)
   {
      xyz[ii][0]+=inavg[0];
      xyz[ii][1]+=inavg[1];
      xyz[ii][2]+=inavg[2];
   }
   // integer output
   ioutrxyz[0]=(INT16)(results[0]);
   ioutrxyz[1]=(INT16)(results[1]);
   ioutrxyz[2]=(INT16)(results[2]);
   ioutrxyz[3]=(INT16)(results[3] );

}

INT16 em_center( INT16 (*xyz)[3], INT16 *ioutrxyz, INT16 npts)
{
   INT16 iavg[3] = {0};
   INT16 rc;

   em_prepare_data(xyz, &iavg[0], npts);	// subtract average, move to real arrays

   em_JTJ(npts);							// form J'J

   em_JTK(npts);							// form J'K

   em_moveData();							// Move J'J to matrix for inversion

   rc=em_invert(PARA_3D);					// invert J'J

   if(rc<0)
   {
      ioutrxyz[0]=0;

      return false;
   }
   em_JTJinvJK();// Multiply J'K by inverted matrix
   em_results(&Jout[0], ioutrxyz, iavg, xyz, npts);// use output to find hard iron offsets

#if __AAA__
   em_restore_data(xyz, iavg, Jout, ioutrxyz, npts);

  	// put average back into data and
	// move real results to integer ioutrxyz
	// hard iron offsets are in ioutrxyz[1..3]
#endif
   return true;
}

////////////////////////////////////////////////////////////////////
///////////////////// UTILITY FUNCTIONS ////////////////////////////

/*
*	returns the difference between 2 input vectors.
*
*/
void vec_diff(INT16 *v0, INT16 *v1, INT16 *diff)
{
	if ((v0 == 0) || (v1 == 0) || (diff == 0))
		return ;

	diff[0]	= v1[0]	- v0[0];
	diff[1]	= v1[1]	- v0[1];
	diff[2]	= v1[2]	- v0[2];
}

/*
*	returns the magnitude of a vector.
*
*/
INT32 vec_magnitude(INT16* vec)
{
	INT32 result = -1;
	REAL lX2,lY2,lZ2;

	if (vec)
	{
		lX2	= (REAL)(vec[0]*vec[0]);
		lY2	= (REAL)(vec[1]*vec[1]);
		lZ2	= (REAL)(vec[2]*vec[2]);

		result = (INT32)mSqrt(lX2 + lY2 + lZ2);
		//result = (INT32)sqrt(lX2 + lY2 + lZ2);
	}

	return result;
}

INT16 nDiffAccuracy_3D =0;
INT16 checkAccuracy(INT16 curMag, INT8 lastStatus, long timestamp_ms) {

	int st = ACCURACY_UNRELIABLE;
	if (mSabs(curMag - AvgMagR) < 1500)
		st =  ACCURACY_HIGH;
	else if (mSabs(curMag - AvgMagR) < 3000)
		st = ACCURACY_MEDIUM;
	else if (mSabs(curMag - AvgMagR) < 4500)
		st = ACCURACY_LOW;
	else
		st = ACCURACY_UNRELIABLE;

	if(st != lastStatus || st == ACCURACY_UNRELIABLE)
	{
		nDiffAccuracy_3D++;
		if (nDiffAccuracy_3D > 100)
			nDiffAccuracy_3D = 100;

	    if (nDiffAccuracy_3D >=100)
	    {
			nDiffAccuracy_3D = 0;
	    	ical_Init(timestamp_ms);
	    	diffSameCount = 0;
	    }
	}
	D("checkAccuracy, st = %d, lastStatus = %d , nDiffAccuracy_3D = %d", st, lastStatus,nDiffAccuracy_3D);
	return st;
}

#ifdef USE_OFFSET_AVG
void update_offset()
{
#if __ABC__
	return;
#else
	int i = 0, j = 0;
	INT16 offset_sum[3] = { 0, 0, 0 };
	for (i = 0; i < 3; i++)
		for (j = OFS_AVG_N - 1; j > 0; j--)
			offset_buff[i][j] = offset_buff[i][j - 1];

	offset_buff[0][0] = ofsX;
	offset_buff[1][0] = ofsY;
	offset_buff[2][0] = ofsZ;

	if (offset_avgN < OFS_AVG_N)
		offset_avgN++;
	#ifdef  _WIN32
		if((ofsX != ofsX_old) && (ofsY != ofsY_old) && (ofsZ != ofsZ_old)){		
			ofsX_old = ofsX;ofsY_old = ofsY;ofsZ_old = ofsZ;
			D("ical::update_offset[%6.3f %6.3f %6.3f]\n",ofsX/31.25,ofsY/31.25,ofsZ/31.25);
		}
	#endif
	for (i = 0; i < 3; i++)
		for (j = offset_avgN; j > 0; j--)
			offset_sum[i] += offset_buff[i][j-1];

	ofsX = offset_sum[0] / offset_avgN;
	ofsY = offset_sum[1] / offset_avgN;
	ofsZ = offset_sum[2] / offset_avgN;
#endif 
}

void clear_offset(void)
{
	offset_avgN = 0;
	//curAccuracy = 0;
}
#endif
////////////////////////////////////////////////////////////////////////

///////////////////// Main Process Calibration//////////////////////////

void mcal(long timestamp_ms)
{
	short rok = 0;
	rok = getCalibrationFromFile(&seedCal, wkCoefficients);
	
	if(rok == 1){
		ofsX = seedCal.Offset[0];
		ofsY = seedCal.Offset[1];
		ofsZ = seedCal.Offset[2];
		Kxx = seedCal.Matrix[0];
		Kyy = seedCal.Matrix[1];
		Kzz = seedCal.Matrix[2];
		AvgMagR = seedCal.rr;
		lastStatus = ACCURACY_HIGH;
		lastR = AvgMagR;			
	}else{
		lastStatus = ACCURACY_UNRELIABLE;
		ofsX = ofsY = ofsZ = 0;
		Kxx = Kyy = Kzz = 1562;
		AvgMagR = (unsigned short)DEFAULT_NORMAL;
		ical_Init(timestamp_ms);
		IsFirstCalAfterReboot = 1;			
	}
}

INT8 check_MagR(REAL* magData,INT16 AvgR)
{
	INT16 MagR;
	INT8  ret = 0;
	INT16 rawMag_X = 0;
	INT16 rawMag_Y = 0;
	INT16 rawMag_Z = 0;
	int st = ACCURACY_UNRELIABLE;
	static int nCount=0;
	rawMag_X = magData[0];
	rawMag_Y = magData[1];
	rawMag_Z = magData[2];

 
	MagR = (int) mSqrt(magData[0]*magData[0] + magData[1]*magData[1] +magData[2]*magData[2]);
	
	if (mSabs(MagR - AvgR) < 2000)
		st =  ACCURACY_HIGH;
	else if (mSabs(MagR - AvgR) <5000)
		st = ACCURACY_MEDIUM;
	else if (mSabs(MagR - AvgR) < 8000)
		st = ACCURACY_LOW;
	else
		st = ACCURACY_UNRELIABLE;


	curAccuracy = st;
   	if(st == ACCURACY_LOW || st == ACCURACY_UNRELIABLE)
	{
		nCount++;
	
	    if (nCount >=20)
	    {
			nCount = 0;
	    	ical_Init(0);
	    	diffSameCount = 0;
			return 1;
	    }
	}
	return 0;
}


void get_mag_offset(INT16 mag_offset[4])
{
	mag_offset[0]=ofsX;
	mag_offset[1]=ofsY;
	mag_offset[2]=ofsZ;
	mag_offset[3]=AvgMagR;
}

int process(float* magData, long timestamp_ms)
{
	INT16 rawMag_X = 0;
	INT16 rawMag_Y = 0;
	INT16 rawMag_Z = 0;
	INT16 ICALK_X = 0;
	INT16 ICALK_Y = 0;
	INT16 ICALK_Z = 0;
	INT16 rok, ret = 0;
	INT16 MagR;

	
	ICALK_X = rawMag_X = magData[0];
	ICALK_Y = rawMag_Y = magData[1];
	ICALK_Z = rawMag_Z = magData[2];

#if __AAA__
	magData[0] = (REAL) (rawMag_X - ofsX);
	magData[1] = (REAL) (rawMag_Y - ofsY);
	magData[2] = (REAL) (rawMag_Z - ofsZ);
#else
	magData[0] = (float) ((rawMag_X - ofsX) / DEFAULT_NORMAL * (float)Kxx);
	magData[1] = (float) ((rawMag_Y - ofsY) / DEFAULT_NORMAL * (float)Kyy);
	magData[2] = (float) ((rawMag_Z - ofsZ) / DEFAULT_NORMAL * (float)Kzz);	
#endif

#if 0
	D("process   A ipitch_iroll[%f %f]",ipitch,iroll);
	if((mFabs(ipitch) < 5.0f) && (mFabs(iroll) < 5.0f) )
	{
		nCheckHorizontal++;
		if (nCheckHorizontal >= 50){
			IsHorizontal = 1;
			nCheckHorizontal = 50;		
		}
	}
	else
		IsHorizontal = 0;
#endif

	if(!IsHorizontal)
		ical_collectDataPoint(ICALK_X, ICALK_Y, ICALK_Z, timestamp_ms);
	D("process  B   lastCalibCount,icalMgr.nsamps[%d,%f]",lastCalibCount,icalMgr.nsamps);
	if ((icalMgr.nsamps > FIRST_CALIB_NUM && icalMgr.nsamps % CALIB_INTERVAL_NUM_DEFAULT == 0)
			&& (lastCalibCount != icalMgr.nsamps))
	{
		rok = ical_computeCalibration(&newCal);
		D("process  C rok[%d]",rok);
		lastCalibCount =  icalMgr.nsamps;
		switch (rok)
		{
			case ICAL_SUCCESS:
			D("ical_3D lastCalibCount %d IsFirstCalAfterReboot %d\n",lastCalibCount,IsFirstCalAfterReboot);
			D("ical_3D Offset - X: %f\n",newCal.Offset[0]/31.25);
			D("ical_3D Offset - Y: %f\n",newCal.Offset[1]/31.25);
			D("ical_3D Offset - Z: %f\n",newCal.Offset[2]/31.25);
			D("ical_3D Offset - Kxx: %f\n",newCal.Matrix[0]/31.25);
			D("ical_3D Offset - Kyy: %f\n",newCal.Matrix[1]/31.25);
			D("ical_3D Offset - Kzz: %f\n",newCal.Matrix[2]/31.25);
			D("ical_3D Offset - R: %f\n",newCal.rr/31.25);

			if(newCal.rr > ONLY_UPDATE_LOWER_LIMIT && newCal.rr < ONLY_UPDATE_UPPER_LIMIT)
			{
				ret = 1;
				if (IsFirstCalAfterReboot == 1)
				{
					ofsX =  newCal.Offset[0];
					ofsY =  newCal.Offset[1];
					ofsZ =  newCal.Offset[2];
					Kxx = newCal.Matrix[0];
					Kyy = newCal.Matrix[1];
					Kzz = newCal.Matrix[2];
					AvgMagR = lastR = newCal.rr;

					#ifdef USE_OFFSET_AVG
					update_offset();
					#endif

					ical_clearFarPoint(newCal.rr, ofsX, ofsY, ofsZ);			
					ical_clearFarPoint(newCal.rr, ofsX, ofsY, ofsZ);
					ical_clearFarPoint(newCal.rr, ofsX, ofsY, ofsZ);
					
					rd = mSabs(lastR - newCal.rr);
					diffSameCount ++;
					IsFirstCalAfterReboot = 0;
					break;
				}
			}

			ical_clearFarPoint(newCal.rr, ofsX, ofsY, ofsZ);
			ical_clearFarPoint(newCal.rr, ofsX, ofsY, ofsZ);
	
			rd = mSabs(lastR - newCal.rr);
			lastR = newCal.rr;

			if (rd < RD_DIFF_MAX)
				diffSameCount++;
			else
				diffSameCount = 0;
                   D("process  D newCal.rr[%d]",newCal.rr);
			if(newCal.rr > ONLY_UPDATE_LOWER_LIMIT && newCal.rr < ONLY_UPDATE_UPPER_LIMIT)
			{
				if (diffSameCount >= ONLY_UPDATE_DIFFSAMECOUNT)
				{
					ofsX = newCal.Offset[0];
					ofsY = newCal.Offset[1];
					ofsZ = newCal.Offset[2];
					Kxx = newCal.Matrix[0];
					Kyy = newCal.Matrix[1];
					Kzz = newCal.Matrix[2];
					AvgMagR = newCal.rr;

					#ifdef USE_OFFSET_AVG
						update_offset();
					#endif	

					ical_clearFarPoint(newCal.rr, ofsX, ofsY, ofsZ);
					storeCalibrationToFile(newCal, wkCoefficients);		
				}
				else
				{
					diffSameCount = 0;
					ical_Init(timestamp_ms);
				}
			}
			else
			{
				E("ical_3D rr is over range, do anything !");
				diffSameCount = 0;
				ical_Init(timestamp_ms);
			}

			if (lastCalibCount >= MAXNUM_BEFORE_ROLLBACK)
				ical_shiftData(MAXNUM_BEFORE_ROLLBACK / 2);
			
			break;

			default:
			{
				diffSameCount = 0;
				ical_Init(timestamp_ms);
			}
		}//switch
	}//if FIRST_CALIB_NUM

	MagR = (int) mSqrt(magData[0]*magData[0] + magData[1]*magData[1] +magData[2]*magData[2]);
    D("process  E MagR[%d]",MagR);
	if (IsFirstCalAfterReboot != 1)
	{
		curAccuracy = lastStatus;
		lastStatus = checkAccuracy(MagR, lastStatus, timestamp_ms);
	}
	return ret;
}

int get_mag_bias(float* mag_bias)
{
	mag_bias[0] = ofsX / 31.25;
	mag_bias[1] = ofsY / 31.25;
	mag_bias[2] = ofsZ / 31.25;

	return true;
}

INT8 get_mag_accuracy(void){
	INT8 sf_status = curAccuracy;
	return sf_status;
}

void Set_Picth_Roll(float * Pitch, float * Roll)
{
	ipitch =  *Pitch;
	iroll  =  *Roll;
}

/*if calculate Orientation in HAL,then these codes below can be removed*/
#ifndef ORIENTATION_IN_HAL

#define FILTERNUM 10
REAL Accsum[3] = {0};
REAL fAcc[3][FILTERNUM] = { 0 };

REAL Magsum[3] = {0};
REAL fMag[3][2*FILTERNUM] = { 0 };
REAL iHead = -1;

REAL Filter_Ori(REAL Head)
{
	REAL oldCompass=0;
	REAL newCompass=0;
				 
	REAL SmoothFactorCompass = 0.1;  //0.5
	REAL SmoothThresholdCompass = 15.0;

	oldCompass= iHead;

	if(oldCompass<0)
		oldCompass = 0;
					
	newCompass= Head;

	if (mFabs(newCompass - oldCompass) < 180) 
	{
		if (mFabs(newCompass - oldCompass) > SmoothThresholdCompass)
		{
			oldCompass = newCompass;
		}
		else
		{
			oldCompass = oldCompass + SmoothFactorCompass * (newCompass - oldCompass);
		}
	}
	else
	{
		if(360.0 - mFabs(newCompass - oldCompass) > SmoothThresholdCompass) 
		{
			oldCompass = newCompass;
		}
		else
		{
			if( oldCompass > newCompass )
			{
				oldCompass = (oldCompass + SmoothFactorCompass * (360 + newCompass - oldCompass)) ;
			} 
			else 
			{
				oldCompass = (oldCompass - SmoothFactorCompass * (360 - newCompass + oldCompass));
			}
		}
	}
		
	if(oldCompass<0)
	{
		oldCompass = oldCompass + 360.0f;
	}

	if(oldCompass>360.0f)
	{
		oldCompass=oldCompass-360;
	}

	Head = oldCompass;

	return Head;
}

void Filter_Acc(REAL* acc){

	int i = 0;
	Accsum[0] = Accsum[1] = Accsum[2] = 0; 
	for (i = 0; i < FILTERNUM-1; i++) 
	{
		fAcc[0][i] = fAcc[0][i+1];
		fAcc[1][i] = fAcc[1][i+1];
		fAcc[2][i] = fAcc[2][i+1];
	}
	fAcc[0][FILTERNUM -1] = acc[0];
	fAcc[1][FILTERNUM -1] = acc[1];
	fAcc[2][FILTERNUM -1] = acc[2];

	for (i = 0; i < FILTERNUM; i++) {
		Accsum[0]+= fAcc[0][i];
		Accsum[1]+= fAcc[1][i];
		Accsum[2]+= fAcc[2][i];
	}

	acc[0] = Accsum[0]/FILTERNUM;
	acc[1] = Accsum[1]/FILTERNUM;
	acc[2] = Accsum[2]/FILTERNUM;
}

void Filter_Mag(REAL* mag){
	int i = 0;
	Magsum[0] = Magsum[1] = Magsum[2] = 0;

	for (i = 0; i < 2*FILTERNUM-1; i++) 
	{
		fMag[0][i] = fMag[0][i+1];
		fMag[1][i] = fMag[1][i+1];
		fMag[2][i] = fMag[2][i+1];
	}
	fMag[0][2*FILTERNUM -1] = mag[0];
	fMag[1][2*FILTERNUM -1] = mag[1];
	fMag[2][2*FILTERNUM -1] = mag[2];

	for (i = 0; i < 2*FILTERNUM; i++) {
		Magsum[0]+= fMag[0][i];
		Magsum[1]+= fMag[1][i];
		Magsum[2]+= fMag[2][i];
	}

	mag[0] = Magsum[0]/(2*FILTERNUM);
	mag[1] = Magsum[1]/(2*FILTERNUM);
	mag[2] = Magsum[2]/(2*FILTERNUM);
}

/*
	This function is used to calculate orientation,pitch and roll.
*/
int push2mcal(_QMC7983 *data)
{
	INT8 i;

	REAL Ax, Ay, Az,av;

	REAL yaw,pitch,roll;

	REAL sinP; /* sin value of pitch angle */
	REAL cosP; /* cos value of pitch angle */
	REAL sinR; /* sin value of roll angle */
	REAL cosR; /* cos value of roll angle */
	REAL Xh;   /* X axis element of vector which is projected to horizontal plane */
	REAL Yh;   /* Y axis element of vector which is projected to horizontal plane */	
	
	D("Filter acc_A[%f %f %f] mag_A[%f %f %f]",data->acc[0],data->acc[1],data->acc[2],data->data_cali[0],data->data_cali[1],data->data_cali[2]);

	Filter_Acc(&data->acc[0]);
	Ax = data->acc[0];Ay = data->acc[1];Az = data->acc[2];
	Filter_Mag(&data->data_cali[0]);

	D("Filter acc_B[%f %f %f] mag_B[%f %f %f]",data->acc[0],data->acc[1],data->acc[2],data->data_cali[0],data->data_cali[1],data->data_cali[2]);

	av = InvSqrt(Ax*Ax + Ay*Ay + Az*Az);
	pitch = asinf(-Ay / av);
	roll = asinf(Ax / av);
	
	sinP = sinf(pitch);
	cosP = cosf(pitch);
	sinR = sinf(roll);
	cosR = cosf(roll);	
	
	Yh = -(data->data_cali[0])*cosR + (data->data_cali[2])*sinR;
	Xh = (data->data_cali[0])*sinP*sinR + (data->data_cali[1])*cosP + (data->data_cali[2])*sinP*cosR;
	yaw = atan2(Yh, Xh)* 180.0f / PI;
//	D("push2mcal Mxyx[%f %f %f] iheading= %f\n",Mx,My,Mz,iheading);
	
	(yaw < 0)?(yaw += 360.0f):(yaw);

	data->pitch = pitch* 180.0f / PI;
	data->roll = roll* 180.0f / PI;
	Set_Picth_Roll(&data->pitch,&data->roll);

	D("Orientation_A - heading [%6.3f]\n",yaw);
	iHead = Filter_Ori(yaw);
	D("Orientation_B - heading [%6.3f]\n",iHead);

	data->yaw = iHead;	
	return 0;
}
#endif

int storeCalibrationToFile(TRANSFORM_T theCal, char* fileName)
{

	FILE* pFile = NULL;

	pFile = fopen(fileName, "wt+");

	if(pFile == NULL){
		D("open file error");
		return 0;
	}

	if (pFile)
	{
		// Simple calibration coefficient write-out to file
		// Offsets
		fprintf(pFile, "%d	%d	%d\n", theCal.Offset[0], theCal.Offset[1],theCal.Offset[2]);

		// Matrix
		fprintf(pFile, "%d	%d	%d\n", theCal.Matrix[0],theCal.Matrix[1], theCal.Matrix[2]);

		fprintf(pFile, "%d\n", theCal.rr);
		fclose(pFile);
		return 1;
	}

	return 0;
}


int getCalibrationFromFile(TRANSFORM_T* pCal, char* fileName)
{
	FILE* pFile = NULL;
	int rs;

	if (!pCal)
		return 0;

	pFile = fopen(fileName, "rt+");
	if(pFile == NULL)
	{
		D("open file error");
		return 0;
	}

	if (pFile)
	{
		// Simple calibration coefficient read-in from file
		// Offsets
		rs = fscanf(pFile, "%d	%d	%d", &(pCal->Offset[0]), &(pCal->Offset[1]),&(pCal->Offset[2]));
		// Matrix
		rs = fscanf(pFile, "%d	%d	%d", &(pCal->Matrix[0]),&(pCal->Matrix[1]), &(pCal->Matrix[2]));

		rs = fscanf(pFile, "%d", &(pCal->rr));
		
		fclose(pFile);
		return 1;
	}
	return 0;
}
