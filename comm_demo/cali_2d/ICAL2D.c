#include "ICAL2D.h"
#include "ICFG.h"

//X^2+AXY+BY^2+CX+DY+E
//#define AA Jpt[0];
//#define BB Jpt[1];
//#define CC Jpt[2];
//#define DD Jpt[3];
//#define EE Jpt[4];

#ifdef ELLIPSE_2D
	#define ENS_PARA		5
#else
	#define ENS_PARA		3
#endif

#define DIM_A	ENS_PARA

// Next two have to match
#define ENS_ARRAY_SIZE	50

#define ENS2D_STANDOFF          0.20//0.35					// Minimum distance between samples in radians

#define MAT_INVERT_OK   0
#define MAT_INVERT_ERR  1

// Internal Structural Type Definitions
typedef struct {
	INT16 mag[ENS_ARRAY_SIZE][2];
} CAL2D_ENSEMBLE_T;

typedef struct {
	INT16 trialMag[3];
	INT16 vecDiff[3];
	INT16 nsamps; //Count of collection data
	INT16 standoffCounts;
	INT32 sampleTime;
} ICAL2D_MGR_T;

typedef struct {
	REAL MatA[DIM_A][DIM_A];
	REAL invMatA[DIM_A][DIM_A];
} MA_INVERT_2D_T;

// Internal Variable Definitions
static CAL2D_ENSEMBLE_T ical2DEns;
static ICAL2D_MGR_T ical2DMgr;
static MA_INVERT_2D_T ma2d;

static REAL *Jpt_2D[DIM_A];
static REAL JTJ_2D[DIM_A][DIM_A];
static REAL JTK_2D[DIM_A];
static REAL Jout_2D[DIM_A];

static REAL magX_2D[ENS_ARRAY_SIZE];
static REAL magY_2D[ENS_ARRAY_SIZE];
static REAL magXX_2D[ENS_ARRAY_SIZE];
static REAL magXY_2D[ENS_ARRAY_SIZE];
static REAL magYY_2D[ENS_ARRAY_SIZE];
static REAL rr_2D[ENS_ARRAY_SIZE];

REAL avgR =0;
REAL T = 0;
REAL E_2D_ofsX = 0;
REAL E_2D_ofsY = 0;
REAL CosT = 0;
REAL SinT = 0;
REAL R1 = 1;
REAL R2 = 1;
/////////////////////////////////////////////////////////////////////
///////////////// Internal Function Prototypes //////////////////////
/////////////////////////////////////////////////////////////////////

void vec_diff_2D(INT16 *v0, INT16 *v1, INT16 *diff);
INT32 vec_magnitude_2D(INT16* vec);

INT16 ical_qualify2D(INT32 currentTimeMsec, INT16 rawMagX, INT16 rawMagY);
void ical2D_addSample(INT16* mag, INT16 sampIX);
void ical2D_shiftData(int length);
extern void removePointAt_K(int num);
INT16 em_invert_2D(int size);
void em_average_2D(INT16(*xyz)[2], REAL *avg, INT16 *iavg, INT16 npts);
void em_demean_2D(INT16(*xyz)[2], INT16 *avg, INT16 npts);
INT16 em_center_2D(INT16(*xyz)[2], INT16 *outrxyz, INT16 npts);
////////////////////////////////////////////////////////////////////
//////////////// START OF EXTERNAL LIBRARY CODE ////////////////////
////////////////////////////////////////////////////////////////////
#if 0
static int64_t get_time_in_nanosec(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000000000LL + tv.tv_usec * 1000LL;
}
#endif
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
void ical2D_Init(long currentTimeMSec, unsigned short Normalizer) {
	D("ical_2D ical_Init");
	// Zero the managers
	memset(&ical2DMgr, 0, sizeof(ICAL2D_MGR_T));
	memset(&ical2DEns, 0, sizeof(CAL2D_ENSEMBLE_T));

	// Initialize the sample time tick
	ical2DMgr.sampleTime = currentTimeMSec;

	// Initialize the stand off counts
	ical2DMgr.standoffCounts = (INT16) (Normalizer * ENS2D_STANDOFF);

}

/************************************************************************
 *	This function is used to determine if enough data has been collected
 *	to try and generate calibration coefficients from. It is important
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
INT16 ical2D_readyCheck() {
	return ical2DMgr.nsamps;
}


void removePointAt_2D(int num)
{
	int i = 0;
	D("ical_2D_clearFarPoint, removePointAt - %d", num);
	if (num >= ical2DMgr.nsamps || num == -1)
	{
		return;
	}

	for ( i = 0; i < ical2DMgr.nsamps; i++)
	{
		if (i > num)
		{
			ical2DEns.mag[i - 1][0] = ical2DEns.mag[i][0];
			ical2DEns.mag[i - 1][1] = ical2DEns.mag[i][1];
		}

	}
	ical2DMgr.nsamps--;
}

void ical2D_clearFarPoint(int rr, int ofx, int ofy)
{
	int i = 0;
	D("ical_2D_clearFarPoint, N = %d, INT -> [%d, %d]", ical2DMgr.nsamps, ofx, ofy);
	if (ical2DMgr.nsamps > 2) {
		int maxIndex = 0;
		int minIndex = 0;

		float maxVar, minVar;

		maxVar = 0;

		for (i = 0; i < ical2DMgr.nsamps; i++) {
			float var = sqrt((ical2DEns.mag[i][0] - ofx) * (ical2DEns.mag[i][0]
					- ofx) + (ical2DEns.mag[i][1] - ofy) * (ical2DEns.mag[i][1]
					- ofy));

			if (fabs(var - rr) > maxVar) {
				maxVar = fabs(var - rr);
				maxIndex = i;
			}

			//D("ical2D_clearFarPoint, n = %d, var = %f,  _[%d, %d]", i, var, ical2DEns.mag[i][0], ical2DEns.mag[i][1]);
		}
		//D("ical2D_clearFarPoint maxVar = %f _(%d), minVar = %f _(%d)", maxVar, maxIndex, minVar, minIndex);
		removePointAt_2D(maxIndex);

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
INT16 ical2D_collectDataPoint(INT16 rawMagX, INT16 rawMagY,
		INT32 currentTimeMsec)
{
	INT16 rok = 1;
	rok = ical_qualify2D(currentTimeMsec, rawMagX, rawMagY);
//	kal_prompt_trace(MOD_MATV,"ical_2D_collectDataPoint, (%d, %d) - %d   (icalMgr.nsamps = %d)  _rok = %d",rawMagX, rawMagY, currentTimeMsec, ical2DMgr.nsamps, rok);
	if (rok == 0)
		return 0;

	ical2D_addSample(&ical2DMgr.trialMag[0],ical2DMgr.nsamps);

	return 1;
}

/*************************************************************************
 *	This function is used to generated magnetic calibration coefficients. It
 *	should be called after the ical_readyCheck() function has indicated that
 *	enough qualified data (24) have been collected.
 *
 *	Output:
 *
 *		xfp:	- Structure that contains both the magnetic Hard Iron offsets
 *				  and the Soft Iron gain corrections. // xfp is an output argument
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
INT16 ical2D_computeCalibration(TRANSFORM_2D_T* xfp) {

	INT16 irxyz[3];
	
	INT16 rc;
	D("\n - ical_2D_computeCalibration - \n");

	if (!xfp)
		return 0;

	// Generate simple centering offsets and apply them to data set
	rc = em_center_2D(&ical2DEns.mag[0], &irxyz[0], ical2DMgr.nsamps);
	if (!rc)
	{
		return ICAL2D_FAILURE;
	}
#ifndef ELLIPSE_2D
	D("ical_2D Hard_Iron- r = %d, {%d, %d}, (%d)", irxyz[0], irxyz[1], irxyz[2], ical2DMgr.nsamps);
	// Get the generated radius

	xfp->rr = irxyz[0];
	xfp->Offset[0] =  irxyz[1];
	xfp->Offset[1] =  irxyz[2];

#else
//	EllipseRemovePoint();
//	EllipseRemovePoint();
	xfp->rr = (int) avgR;
	xfp->Offset[0] = (int) E_2D_ofsX;
	xfp->Offset[1] = (int) E_2D_ofsY;
	D("ical_2D Ellipse - r = %d, {%d, %d}, (%d)", xfp->rr, xfp->Offset[0], xfp->Offset[1], ical2DMgr.nsamps);
#endif
	return ICAL2D_SUCCESS;
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
 */
INT16 ical_qualify2D(INT32 currentTimeMsec, INT16 rawMagX, INT16 rawMagY) {
	INT16 ii;
	int num;
//	kal_prompt_trace(MOD_MATV,"ical_qualify2D, (cur = %d, sampleTime = %d abs = %d)", currentTimeMsec, ical2DMgr.sampleTime,abs(currentTimeMsec - ical2DMgr.sampleTime));
	
	if (abs(currentTimeMsec - ical2DMgr.sampleTime) < 25) { /*25*/
		D("ical_qualify2D, return 0 - 1, (cur = %ld, sampleTime = %ld)", currentTimeMsec, ical2DMgr.sampleTime);
//		return 0;
	}

	ical2DMgr.trialMag[0] = rawMagX;
	ical2DMgr.trialMag[1] = rawMagY;

	ical2DMgr.sampleTime = currentTimeMsec;

	if (ical2DMgr.nsamps == 0) {
		D("ical_qualify2D, return 1 - 1");
		return 1;
	}

	ii = (ical2DMgr.nsamps <= STANDOFF_CHKNUM_2D) ? ical2DMgr.nsamps
			: STANDOFF_CHKNUM_2D;

	D("ical2DMgr.nsamps = %d, ii = %d", ical2DMgr.nsamps,ii);
	//int num = ical2DMgr.nsamps - ii;
	num = ical2DMgr.nsamps - ii;

	D("2D, ii = %d, num = %d", ii, num);

	while (ii > 0) {
		ii--;
		vec_diff_2D(&(ical2DMgr.trialMag[0]), &(ical2DEns.mag[num + ii][0]),
				&(ical2DMgr.vecDiff[0]));

		D("ical_qualify2D vec_magnitude_2D %d\n",vec_magnitude_2D(&ical2DMgr.vecDiff[0]));

		if (vec_magnitude_2D(&ical2DMgr.vecDiff[0]) < ical2DMgr.standoffCounts) {
			D("ical_qualify2D, return 0 - 2");
			return 0;
		}
	}
	D("ical_qualify2D, return 1 - 2");
	return 1;
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
void ical2D_addSample(INT16* mag, INT16 sampIX)
{
	INT16 ii;

	for (ii = 0; ii < 2; ii++) {
		ical2DEns.mag[sampIX][ii] = mag[ii];
	//	ical2DEns.grav[sampIX][ii] = acc[ii];
	}
	ical2DMgr.nsamps = ical2DMgr.nsamps + 1;

	// Circle around if exceed buffer size
	if (ical2DMgr.nsamps >= ENS_ARRAY_SIZE)
		ical2DMgr.nsamps = 0;
}

/*
 *
 * Left shift data
 *
 *
 * */
void ical2D_shiftData(int length) {

	int i = 0;
	D("ical2D_shiftData, nsamps = %d, length = %d", ical2DMgr.nsamps, length);
	if (ical2DMgr.nsamps < length || length > MAXNUM_BEFORE_ROLLBACK_2D)
		return;

	for (i = 0; i < ical2DMgr.nsamps; i++) {
		ical2DEns.mag[i][0] = ical2DEns.mag[i + length][0];
		ical2DEns.mag[i][1] = ical2DEns.mag[i + length][1];
	}

	ical2DMgr.nsamps -= length;

	for (i = 0; i < length; i++) {
		ical2DEns.mag[i + ical2DMgr.nsamps][0] = ical2DEns.mag[i
				+ ical2DMgr.nsamps][1] = 0;
	}

}

/////////////////////////////////////////////////////////////////////

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
//        0 = Successful inversion
//        -1 = Error: Singular Matrix
//
INT16 em_invert_2D(int size) {
	REAL alpha, beta;
	INT16 ii, jj, kk;

	if ((size < 2) || (size > DIM_A))
		return (MAT_INVERT_ERR);

	// initialize the reduction matrix--> unity matrix
	for (ii = 0; ii < size; ii++) {
		for (jj = 0; jj < size; jj++) {
			ma2d.invMatA[ii][jj] = 0.0f;
		//	 D("em_invert_2D - ma2d.MatA[%d][%d] = %0.15f", ii,jj,ma2d.MatA[ii][jj]);
		}
		ma2d.invMatA[ii][ii] = 1.0f;
	}

	// do the reduction
	for (ii = 0; ii < size; ii++) {
		alpha = ma2d.MatA[ii][ii];

		if (alpha == 0.0f)
			return (MAT_INVERT_ERR); // -------------------> bail out

		for (jj = 0; jj < size; jj++) {
			ma2d.MatA[ii][jj] = ma2d.MatA[ii][jj] / alpha;
			ma2d.invMatA[ii][jj] = ma2d.invMatA[ii][jj] / alpha;
		} // END jj loop

		for (kk = 0; kk < size; kk++) {
			if ((kk - ii) != 0) {
				beta = ma2d.MatA[kk][ii];
				for (jj = 0; jj < size; jj++) {
					ma2d.MatA[kk][jj] = ma2d.MatA[kk][jj] - beta
							* ma2d.MatA[ii][jj];
					ma2d.invMatA[kk][jj] = ma2d.invMatA[kk][jj] - beta
							* ma2d.invMatA[ii][jj];
				}
			}
		} // END kk loop
	} // END ii loop

	return (MAT_INVERT_OK);
}

// Find the average of each component, report both integer and real versions
//
void em_average_2D(INT16(*xyz)[2], REAL *avg, INT16 *iavg, INT16 npts) {
	INT16 ix;
	INT32 tempL[2] = {0};

	for (ix = 0; ix < npts; ix++) {
		tempL[0] += xyz[ix][0];
		tempL[1] += xyz[ix][1];
	}

	avg[0] = (REAL) (tempL[0] / npts);
	avg[1] = (REAL) (tempL[1] / npts);

	// populate truncated floating point average
	iavg[0] = (INT16)avg[0];
	iavg[1] = (INT16)avg[1];

}

// subtract the average from the data
void em_demean_2D(INT16(*xyz)[2], INT16 *avg, INT16 npts) {
	INT16 ix;

	for (ix = 0; ix < npts; ix++) {
		xyz[ix][0] -= avg[0];
		xyz[ix][1] -= avg[1];
	}
}

// prepare data for hard iron estimate:
// subtract average, move data to real arrays, and initialize pointers
void em_prepare_data_2D(INT16(*xyz)[2], INT16 *iavg, INT16 npts) {
	INT16 ix;

#ifndef ELLIPSE_2D
	REAL avg[2] = {0};
	//subtract average off of integer input data
	 em_average_2D( xyz ,&avg[0],iavg,npts);
	 em_demean_2D ( xyz ,iavg,npts);
#endif

	for (ix = 0; ix < npts; ix++) {
		
#ifndef ELLIPSE_2D

		magX_2D[ix] =  xyz[ix][0];
		magY_2D[ix] =  xyz[ix][1];
		rr_2D[ix] = (magX_2D[ix] * magX_2D[ix] + magY_2D[ix] * magY_2D[ix]);
#else
		magX_2D[ix] =  (REAL)xyz[ix][0];
		magY_2D[ix] =  (REAL)xyz[ix][1];
		rr_2D[ix] = (magX_2D[ix] * magX_2D[ix] + magY_2D[ix] * magY_2D[ix]);
		magXX_2D[ix] =  xyz[ix][0] * xyz[ix][0];
		magXY_2D[ix] =  xyz[ix][0] * xyz[ix][1];
		magYY_2D[ix] =  xyz[ix][1] * xyz[ix][1];
#endif
		
	}
	
#ifndef ELLIPSE_2D
	Jpt_2D[0]=&rr_2D[0];
	Jpt_2D[1]=&magX_2D[0];
	Jpt_2D[2]=&magY_2D[0];
#else
	Jpt_2D[0] = &magXX_2D[0];
	Jpt_2D[1] = &magXY_2D[0];
	Jpt_2D[2] = &magYY_2D[0];
	Jpt_2D[3] = &magX_2D[0];
	Jpt_2D[4] = &magY_2D[0];
#endif
}

// J transpose times J
// This could be formed directly in ma.MatA to save a copy
void em_JTJ_2D(INT16 npts) {
	INT16 jrr, jcc, jkk;

	for (jrr = 0; jrr < ENS_PARA; jrr++) {
		for (jcc = 0; jcc < ENS_PARA; jcc++) {
			JTJ_2D[jrr][jcc] = 0.0f;
			for (jkk = 0; jkk < npts; jkk++) {
				JTJ_2D[jrr][jcc] += Jpt_2D[jrr][jkk] * Jpt_2D[jcc][jkk];
				 //D("em_JTJ_2D - JTJ_2D[%d][%d] = %0.15f", jrr,jcc,JTJ_2D[jrr][jcc]);
			}
		}
	}
}
// move data to matrix for inversion
void em_moveData_2D(void) {
	INT16 jrr, jcc;
	for (jrr = 0; jrr < ENS_PARA; jrr++)
		for (jcc = 0; jcc < ENS_PARA; jcc++) {
			ma2d.MatA[jrr][jcc] = JTJ_2D[jrr][jcc];
			 //  D("em_moveData_2D - ma2d.MatA[%d][%d] = %0.15f", jrr,jcc,ma2d.MatA[jrr][jcc]);
		}
}

// vector K is a column of 1's
// so J'*K (J transpose times K --> JTK) is a column vector
// each of whose elements is the sum of one of the rows of J'

void em_JTK_2D(INT16 npts) {

	INT16 jrr, jkk;

	for (jrr = 0; jrr < ENS_PARA; jrr++) {
		JTK_2D[jrr] = 0.0f;
		for (jkk = 0; jkk < npts; jkk++) {
			JTK_2D[jrr] += Jpt_2D[jrr][jkk];
			  //D("em_JTK_2D - JTK_2D[%d] = %0.15f", jrr,JTK_2D[jrr]);
		}
	}
}

// inverse of J transpose times J ("JTJinv")
// times J'K
void em_JTJinvJK_2D(void) {
	INT16 jrr, jkk;
	for (jrr = 0; jrr < ENS_PARA; jrr++) {
		Jout_2D[jrr] = 0.0;
		for (jkk = 0; jkk < ENS_PARA; jkk++) {
			Jout_2D[jrr] += ma2d.invMatA[jrr][jkk] * JTK_2D[jkk];
			 // D("em_JTJinvJK_2D - ma2d.invMatA[%d][%d] = %0.15f", jrr,jkk,ma2d.invMatA[jrr][jkk]);
			//  D("em_JTJinvJK_2D - JTK_2D[%d] = %0.15f",jkk,JTK_2D[jkk]);
			//  D("em_JTJinvJK_2D - Jout_2D[%d] = %0.15f",jrr,Jout_2D[jrr]);
		}
	}
}

// A(X^2 +Y^2 + Z^2) + Bx + Cy + D = 1 for sphere
// A is in inv[0]
// B is in inv[1] ...
// ...
//
// Return radius in inv[0]
// center offsets in inv[1..3] for sphere

REAL em_results_2D(REAL *inv)
{
	REAL RR =0 ;
	REAL ofs[2] = {0};
#ifdef ELLIPSE_2D
	REAL theta;
	//D("em_results_2D - {%0.15f, %0.15f, %0.15f, %0.15f, %0.15f}", inv[0], inv[1], inv[2], inv[3], inv[4]);

	REAL AA = inv[0] * (-1);
	REAL BB = inv[1] * (-1);
	REAL CC = inv[2] * (-1);
	REAL DD = inv[3] * (-1);
	REAL EE = inv[4] * (-1);
	REAL tmp1;
	REAL tmp2;
	REAL ct;
	REAL st;
	REAL ap;
	REAL cp;


	
	D("em_results_2D - {%0.15f, %0.15f, %0.15f, %0.15f, %0.15f}", inv[0], inv[1], inv[2], inv[3], inv[4]);
	D("em_results_2D - AA =%0.15f,BB =%0.15f,CC =%0.15f,DD =%0.15f,EE =%0.15f,",AA,BB,CC,DD,EE);

	ofs[0] = (BB * EE - 2 * CC * DD) / (4 * AA * CC - BB * BB);
	ofs[1] = (BB * DD - 2 * AA * EE) / (4 * AA * CC - BB * BB);


	#if 0
	REAL tmp1 = AA * ofs[0] * ofs[0] + CC * ofs[1] * ofs[1] + BB * ofs[1] * ofs[0] - 1;
	REAL tmp2 = sqrt(BB * BB + ((AA - CC) * (AA - CC)));
    #else
	 tmp1 = AA * ofs[0] * ofs[0] + CC * ofs[1] * ofs[1] + BB * ofs[1] * ofs[0] - 1;
	 tmp2 = sqrt(BB * BB + ((AA - CC) * (AA - CC)));


	#endif
	
	
	D("em_results_2D - tmp1 =%0.15f,tmp2 =%0.15f,N = %d",tmp1,tmp2, ical2DMgr.nsamps);
	theta = (REAL) 0.5f * atan2(BB , (AA - CC));

	#if 0
	REAL ct = cos(theta);
	REAL st = sin(theta);
	REAL ap = AA * ct * ct + BB * ct * st + CC *st *st;
	REAL cp = AA * st * st - BB * ct * st + CC *ct *ct;
	#else
	 ct = cos(theta);
	 st = sin(theta);
	 ap = AA * ct * ct + BB * ct * st + CC *st *st;
	 cp = AA * st * st - BB * ct * st + CC *ct *ct;
	#endif

	if((tmp1 / ap)>0)
		R1 = sqrt( tmp1 / ap);
	else
		R1 = 1;

	if(( tmp1 / cp)>0)
		R2 = sqrt( tmp1 / cp);
	else
		R2 = 1;

	D("em_results_2D - num = %d,  ofs [%0.15f, %0.15f], rr {%0.15f, %0.15f}, theta = %0.15f",ical2DMgr.nsamps, ofs[0], ofs[1], R1, R2, theta*180/PI);

	T = (-1) * theta;
	E_2D_ofsX = ofs[0];
	E_2D_ofsY = ofs[1];
	CosT = cos(T);
	SinT = sin(T);
	
	EllipseRemovePoint();
	RR = avgR;
	return RR;
#else	
	D("em_results_2D - {%f, %f, %f}", inv[0], inv[1], inv[2]);
	ofs[0] = -inv[1]/(2.0f * inv[0]);
	ofs[1] = -inv[2]/(2.0f * inv[0]);
	//RR2 = 4.0f*inv[0] + inv[1]*inv[1] + inv[2]*inv[2];
	RR = (REAL)sqrt(4.0f*inv[0] + inv[1]*inv[1] + inv[2]*inv[2])/(2.0f*inv[0]);

	inv[0] = RR;
	inv[1] = ofs[0];
	inv[2] = ofs[1];

	return RR;
#endif
}

void EllipseRemovePoint()
{
	int i = 0;

	if (ical2DMgr.nsamps > 2)
	{
		REAL tempCompute[2] = {0};
		REAL tempEnsX= 0;
		REAL tempEnsY= 0;
		int maxIndex = 0;
		int minIndex = 0;
		REAL maxVar = 0;
		REAL minVar = 0;
		REAL sumVar = 0;
		REAL avgVar = 0;
		REAL var[100] = {0};

		tempCompute[0] = (CosT * (ical2DEns.mag[0][0] - E_2D_ofsX) - SinT * (ical2DEns.mag[0][1] - E_2D_ofsY)) / R1;
		tempCompute[1] = (SinT * (ical2DEns.mag[0][0] - E_2D_ofsX) + CosT * (ical2DEns.mag[0][1] - E_2D_ofsY)) / R2;
		tempEnsX = ( CosT * tempCompute[0] + SinT * tempCompute[1] ) * DEFAULT_NORMAL_2D;
		tempEnsY = ( CosT * tempCompute[1] - SinT * tempCompute[0] ) * DEFAULT_NORMAL_2D;
		maxVar = minVar = tempEnsX * tempEnsX+ tempEnsY * tempEnsY;

		for (i = 1; i < ical2DMgr.nsamps; i++)
		{

			tempCompute[0] = (CosT * (ical2DEns.mag[i][0] - E_2D_ofsX) - SinT * (ical2DEns.mag[i][1] - E_2D_ofsY)) / R1;
			tempCompute[1] = (SinT * (ical2DEns.mag[i][0] - E_2D_ofsX) + CosT * (ical2DEns.mag[i][1] - E_2D_ofsY)) / R2;

			tempEnsX = ( CosT * tempCompute[0] + SinT * tempCompute[1] ) * DEFAULT_NORMAL_2D;
			tempEnsY = ( CosT * tempCompute[1] - SinT * tempCompute[0] ) * DEFAULT_NORMAL_2D;
			var[i] = tempEnsX * tempEnsX + tempEnsY * tempEnsY;
			
			sumVar =sumVar+var[i];

			if (var[i] > maxVar) {
				maxVar = var[i];
				maxIndex = i;
			}
			if (var[i] < minVar) {
				minVar = var[i];
				minIndex = i;
			}

			//D("ical2D_clearFarPoint, n = %d, var = %f,  _[%d, %d]", i, var, ical2DEns.mag[i][0], ical2DEns.mag[i][1]);
		}
		D("ical_2D_EllipseRemovePoint maxVar = %f _(%d), minVar = %f _(%d)", maxVar, maxIndex, minVar, minIndex);


		avgVar = sumVar / ical2DMgr.nsamps;
		avgR = sqrt(avgVar);

		removePointAt_2D(maxIndex);
		if(maxIndex < minIndex)
			minIndex--;
		removePointAt_2D(minIndex);
	}

}

// put the average back into the data
// report the results out as integers
void em_restore_data_2D(INT16(*xyz)[2], INT16 *inavg, REAL *results,
		INT16* ioutrxyz, INT16 npts) {
	INT16 ii;
	// correct center with average. note results index starts at 1
	results[1] += inavg[0];
	results[2] += inavg[1];
	// put average back into data
	for (ii = 0; ii < npts; ii++) {
		xyz[ii][0] += inavg[0];
		xyz[ii][1] += inavg[1];
	}
	// integer output
	ioutrxyz[0] = (INT16) (results[0]);
	ioutrxyz[1] = (INT16) (results[1]);
	ioutrxyz[2] = (INT16) (results[2]);

}

INT16 em_center_2D(INT16(*xyz)[2], INT16 *ioutrxyz, INT16 npts) {
	INT16 iavg[2] = { 0 };
	INT16 rc;

	em_prepare_data_2D(xyz, &iavg[0], npts); // subtract average, move to real arrays
	em_JTJ_2D(npts); // form J'J
	em_JTK_2D(npts); // form J'K
	em_moveData_2D(); // Move J'J to matrix for inversion
	rc = em_invert_2D(ENS_PARA); // invert J'J

	if (rc < 0) {
		ioutrxyz[0] = 0;
		return 0;
	}
	em_JTJinvJK_2D(); // Multiply J'K by inverted matrix
	em_results_2D(&Jout_2D[0]); // use output to find hard iron offsets
	#ifndef ELLIPSE_2D
	// put average back into data and
	// move real results to integer ioutrxyz
	// hard iron offsets are in ioutrxyz[1..3]
	em_restore_data_2D(xyz, iavg, Jout_2D, ioutrxyz, npts); 
	#endif
	return 1;
}

///////////////////// UTILITY FUNCTIONS ////////////////////////////

/*
 *	returns the difference between 2 input vectors.
 *
 */
void vec_diff_2D(INT16 *v0, INT16 *v1, INT16 *diff) {
	if ((v0 == 0) || (v1 == 0) || (diff == 0))
		return;

	diff[0] = v1[0] - v0[0];
	diff[1] = v1[1] - v0[1];
}

/*
 *	returns the magnitude of a vector.
 *
 */
INT32 vec_magnitude_2D(INT16* vec) {
	INT32 result = -1;
	INT32 lX2, lY2, lZ2;

	if (vec) {
		lX2 = vec[0] * vec[0];
		lY2 = vec[1] * vec[1];

		result = (INT32) sqrt(lX2 + lY2);
	}

	return result;
}
