#include "ICFG.h"
#include "ICAL2D.h"


#define	ACCURACY_UNRELIABLE		0
#define	ACCURACY_LOW			1
#define	ACCURACY_MEDIUM			2
#define	ACCURACY_HIGH			3

/********************Calibration***************************/
long nowTime = 0;
long lastTime = 0;

// 2D
TRANSFORM_2D_T newCal2D = {0};
TRANSFORM_2D_T seedCal2D = {0};
int ofsX_2D = 0, ofsY_2D = 0;
static short AvgMagR_2D = DEFAULT_NORMAL_2D;
//static int IsHorizontal = 0;
static short Is2DReady = 0;
int lastCalibCount_2D = 0;
int vd_2D = 0;
int vd_xy_2D = 0;
int rd_2D = 0;
static int lastStatus_2D = ACCURACY_UNRELIABLE;
static int curAccuracy_2D = ACCURACY_UNRELIABLE;
int lastHard_2D[3] = { 0, 0 };
int lastR_2D = 0;
int diffSameCount_2D = 0;
REAL M_CosT = 0;
REAL M_SinT = 0;
REAL M_E_2D_ofsX = 0;
REAL M_E_2D_ofsY = 0;
REAL M_R1 = 1;
REAL M_R2 = 1;
REAL M_T = 1;

extern REAL T;
extern REAL E_2D_ofsX;
extern REAL E_2D_ofsY;
extern REAL CosT;
extern REAL SinT;
extern REAL R1;
extern REAL R2;
//extern char* wkCoefficients;

/**********************************************/

//int getCalibrationFromFile(TRANSFORM_2D_T* pCal, char* fileName);
//int storeCalibrationToFile(TRANSFORM_2D_T theCal, char* fileName);

#define OFS_AVG_N    4
int offset_buff[2][OFS_AVG_N] = { 0 };
int offset_avgN = 0;

#ifdef  _WIN32
int64_t get_time_in_nanosec(void)
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	D("get_time_in_nanosec = %f",sys.wSecond);
	return (sys.wSecond * 1000LL + sys.wMilliseconds) * 1000000LL;
}
#else
int64_t get_time_in_nanosec(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000000000LL + tv.tv_usec * 1000LL;
}
#endif

int CUSTOMER_SET_EllipsePram(EllipsePram *pram){
	int rot = 0; 
	
	M_CosT = pram->CosT;
	M_SinT = pram->SinT;
	M_E_2D_ofsX = pram->E_2D_ofsX;
	M_E_2D_ofsY = pram->E_2D_ofsY;
	M_R1 = pram->R1;
	M_R2 = pram->R2;
	M_T = pram->T;
	AvgMagR_2D = pram->rr;

	if(fabs(M_E_2D_ofsX) > 0.0f && fabs(M_E_2D_ofsY) >0.0f && AvgMagR_2D > 0){
		rot = 1;
	}
	return rot;
}

void mcal(short rok) {

	D("into mcal ...");

	//E(VER_CHK);
#ifdef DISABLE_SOFT_IRON
	E("3D : Hard magnetic");
#else
	E("3D : Soft magnetic");
#endif

//	rok = getCalibrationFromFile(&seedCal2D, wkCoefficients);

	if (rok == 1) {
		ofsX_2D = M_E_2D_ofsX;
		ofsY_2D = M_E_2D_ofsY;
		AvgMagR_2D = AvgMagR_2D;

		lastStatus_2D = ACCURACY_HIGH;
		ical2D_Init((long) (get_time_in_nanosec() / 1000000), DEFAULT_NORMAL_2D);
		E("Load calibration data success !");

		lastHard_2D[0] = ofsX_2D;
		lastHard_2D[1] = ofsY_2D;
		lastR_2D = AvgMagR_2D;
		Is2DReady = 1;
	}
	else
	{
		lastR_2D = ACCURACY_UNRELIABLE;
		ofsX_2D = ofsY_2D = 0;
		AvgMagR_2D = DEFAULT_NORMAL_2D;
		ical2D_Init((long) (get_time_in_nanosec() / 1000000), DEFAULT_NORMAL_2D);

	}

	E("Offset - X: %d", ofsX_2D);
	E("       - Y: %d", ofsY_2D);
	E("Load - AvgMagR = %d",AvgMagR_2D);
	E("-------------------------------------");

}

int nDiffAccuracy_2D =0;
int checkAccuracy_2D(int curMag_2D, int lastStatus_2D)
{
	int st_2D = ACCURACY_UNRELIABLE;

	D("checkAccuracy_2D, curMag_2D = %d, AvgMagR_2D = %d, \t|(%d)|", curMag_2D, AvgMagR_2D, (curMag_2D - AvgMagR_2D));

	//int st_2D = ACCURACY_UNRELIABLE;
	if (curMag_2D - AvgMagR_2D < 700 && AvgMagR_2D - curMag_2D < 700)//300
	{

		st_2D =  ACCURACY_HIGH;
	}
	else if (curMag_2D - AvgMagR_2D < 900 && AvgMagR_2D - curMag_2D < 900)//500
	{

		st_2D = ACCURACY_MEDIUM;
	}
	else if (curMag_2D - AvgMagR_2D < 1200 && AvgMagR_2D - curMag_2D < 1200)//700
	{
		st_2D = ACCURACY_LOW;
	}
	else
		st_2D = ACCURACY_UNRELIABLE;


	D("checkAccuracy2D, st_2D = %d, lastStatus_2D = %d", st_2D, lastStatus_2D);
//	kal_prompt_trace(MOD_MATV,"checkAccuracy2D, st_2D = %d, lastStatus_2D = %d\n", st_2D, lastStatus_2D);
	if(st_2D == ACCURACY_UNRELIABLE)
	{
		nDiffAccuracy_2D++;
		if(nDiffAccuracy_2D >= 150)
			nDiffAccuracy_2D =150;

		if(nDiffAccuracy_2D >=150)
		{
			nDiffAccuracy_2D = 0;
			ical2D_Init((long) (get_time_in_nanosec() / 1000000), DEFAULT_NORMAL_2D);
			Is2DReady =0;
		}

	}
	else
		nDiffAccuracy_2D =0;
	D("checkAccuracy2D, st_2D = %d, lastStatus_2D = %d,nDiffAccuracy_2D = %d", st_2D, lastStatus_2D,nDiffAccuracy_2D);
	return st_2D;
}


int vecDiff_XY_2D(int newX, int newY) {
	int vdx = newX - lastHard_2D[0];
	int vdy = newY - lastHard_2D[1];


	if (lastHard_2D[0] == 0 && lastHard_2D[1] == 0)
		return 0;

	D("vd_xy_2D - (%d, %d)", vdx, vdy);
	return (int) sqrt(vdx * vdx + vdy * vdy);
}

void updateEllipseRes()
{

//	kal_prompt_trace(MOD_MATV,"updateEllipseRes 2D! diffSameCount_2D = %d", diffSameCount_2D);
	M_CosT = CosT;
	M_SinT = SinT;
	M_E_2D_ofsX = E_2D_ofsX;
	M_E_2D_ofsY = E_2D_ofsY;
	M_R1 = R1;
	M_R2 = R2;
	M_T = T;
#if 0
	memset(buf_qws_double,0,100);
	sprintf(buf_qws_double,"updateEllipseRes[%f, %f, %f, %f, %f, %f, %f]\n",
		M_CosT,M_SinT,M_E_2D_ofsX,M_E_2D_ofsY,M_R1,M_R2,M_T); 															
	kal_prompt_trace(MOD_MATV,buf_qws_double);
#endif
}

void update_offset()
{
	int i = 0, j = 0;
	int offset_sum[2] = { 0, 0};
	for (i = 0; i < 2; i++)
	{
		for (j = OFS_AVG_N - 1; j >= 0; j--)
		{
			offset_buff[i][j] = offset_buff[i][j - 1];
		}
	}

	offset_buff[0][0] = ofsX_2D;
	offset_buff[1][0] = ofsY_2D;

	if (offset_avgN < OFS_AVG_N)
		offset_avgN++;

	for (i = 0; i < 2; i++)
	{			
		for (j = offset_avgN; j > 0; j--)
		{
			offset_sum[i] += offset_buff[i][j-1];
		}
	}

	ofsX_2D = offset_sum[0] / offset_avgN;
	ofsY_2D = offset_sum[1] / offset_avgN;
}

//int process(sensors_event_t* mMagneticEvent)
int process(REAL *mMagneticData)
{
	short rawMag_X = 0;
	short rawMag_Y = 0;
	short rawMag_Z = 0;
	short rok = 0;
	short rok_2D = 0;
	int r = 0,i =0,j=0;

	rawMag_X = mMagneticData[0];
	rawMag_Y = mMagneticData[1];
	rawMag_Z = mMagneticData[2];

	//D("Raw_data [%d, %d, %d ]",rawMag_X, rawMag_Y, rawMag_Z);

	//kal_prompt_trace(MOD_MATV,"Is2DReady = %d\n ",Is2DReady);
	//kal_prompt_trace(MOD_MATV,"2D sample numbers = %d\n",ical2D_readyCheck());

	//kal_prompt_trace(MOD_MATV, "msec %d\n",((long)get_time_in_nanosec() / 1000000));
	ical2D_collectDataPoint(rawMag_X, rawMag_Y,(long) (get_time_in_nanosec() / 1000000));

	if(bEnable_ELLIPSE_2D)
	{
		if ((ical2D_readyCheck() > FIRST_CALIB_NUM_2D && ical2D_readyCheck()%CALIB_INTERVAL_NUM_2D == 0) && (lastCalibCount_2D!= ical2D_readyCheck()))
		{
			lastCalibCount_2D = ical2D_readyCheck(); 
			rok_2D = ical2D_computeCalibration(&newCal2D);
			switch (rok_2D)
			{
			  case ICAL2D_SUCCESS:
				if (newCal2D.rr < ONLY_UPDATE_LOWER_LIMIT_2D || newCal2D.rr > ONLY_UPDATE_UPPER_LIMIT_2D)
				{
					ical2D_Init((long) (get_time_in_nanosec() / 1000000),DEFAULT_NORMAL_2D);
					lastCalibCount_2D = -1;
					diffSameCount_2D = 0;
//					kal_prompt_trace(MOD_MATV,"ical_2D check fail\n");
					break;
				}
				
				if (Is2DReady == 0)
				{
					updateEllipseRes();
					ofsX_2D = lastHard_2D[0] = newCal2D.Offset[0];
					ofsY_2D = lastHard_2D[1] = newCal2D.Offset[1];
					AvgMagR_2D = lastR_2D = newCal2D.rr;
					//lastCalibCount_2D = -1;
					ical2D_shiftData(4);
					Is2DReady = 1;
				}
//				kal_prompt_trace(MOD_MATV,"ical_2D lastHard_2D_x =%d,lastHard_2D_y=%d",lastHard_2D[0],lastHard_2D[1]);
				vd_xy_2D= vecDiff_XY_2D(newCal2D.Offset[0], newCal2D.Offset[1]);
				rd_2D = abs(lastR_2D - newCal2D.rr);
//				kal_prompt_trace(MOD_MATV,"ical_2D vd_xy_2D =%d,rd_2D=%d",vd_xy_2D,rd_2D);


				if (vd_xy_2D < VD_XY_DIFF_MAX_2D && rd_2D < RD_DIFF_MAX_2D)
				{
//					kal_prompt_trace(MOD_MATV,"___A-point enter\n");
					diffSameCount_2D++;
					ical2D_clearFarPoint(newCal2D.rr, newCal2D.Offset[0],newCal2D.Offset[1]);
				}
				else
					diffSameCount_2D = 0;


//				kal_prompt_trace(MOD_MATV,"ical_2D, diffSameCount_2D = %d", diffSameCount_2D);

				if (diffSameCount_2D > 1) 
				{
					ofsX_2D = lastHard_2D[0] = newCal2D.Offset[0];
					ofsY_2D = lastHard_2D[1] = newCal2D.Offset[1];
					AvgMagR_2D = lastR_2D = newCal2D.rr;
					updateEllipseRes();
//					kal_prompt_trace(MOD_MATV,"ical_2D bEnable_ELLIPSE_2D ofsX = %d,ofsY = %d",ofsX_2D,ofsY_2D);
					//storeCalibrationToFile(newCal2D, wkCoefficients);
				}
				else
				{
					lastHard_2D[0] = newCal2D.Offset[0];
					lastHard_2D[1] = newCal2D.Offset[1];
					lastR_2D = newCal2D.rr;
				}

				if (ical2D_readyCheck() > MAXNUM_BEFORE_ROLLBACK_2D)
				{
					ical2D_shiftData(MAXNUM_BEFORE_ROLLBACK_2D / 2);
				}
				break;

			default:
	//			kal_prompt_trace(MOD_MATV,"ical_2D Calibration Error!\n");
				ical2D_Init((long) (get_time_in_nanosec() / 1000000),DEFAULT_NORMAL_2D);
				lastCalibCount_2D = -1;
				diffSameCount_2D = 0;
				break;
			}
		}

	}
	else
	{
		if ((ical2D_readyCheck() > FIRST_CALIB_NUM_2D && ical2D_readyCheck()% CALIB_INTERVAL_NUM_2D == 0) && (lastCalibCount_2D!= ical2D_readyCheck()))
		{

			lastCalibCount_2D = ical2D_readyCheck();
			rok_2D = ical2D_computeCalibration(&newCal2D);
			switch (rok_2D) {
				case ICAL2D_SUCCESS:
				if (newCal2D.rr < ONLY_UPDATE_LOWER_LIMIT_2D || newCal2D.rr> ONLY_UPDATE_UPPER_LIMIT_2D)
				{

					ical2D_Init((long) (get_time_in_nanosec() / 1000000),DEFAULT_NORMAL_2D);
					lastCalibCount_2D = -1;
					diffSameCount_2D = 0;
					D("ical_2D check fail");
					break;
				}

				if (Is2DReady == 0) {
					ofsX_2D = lastHard_2D[0] = newCal2D.Offset[0];
					ofsY_2D = lastHard_2D[1] = newCal2D.Offset[1];
					AvgMagR_2D = lastR_2D = newCal2D.rr;
					//lastCalibCount_2D = -1;
					ical2D_shiftData(4);
					Is2DReady = 1;
					ical2D_clearFarPoint(newCal2D.rr, newCal2D.Offset[0],newCal2D.Offset[1]);
					ical2D_clearFarPoint(newCal2D.rr, newCal2D.Offset[0],newCal2D.Offset[1]);
				}
				
				D("ical_2D lastHard_2D_x =%d,lastHard_2D_y=%d",lastHard_2D[0],lastHard_2D[1]);
				vd_xy_2D = vecDiff_XY_2D(newCal2D.Offset[0], newCal2D.Offset[1]);
				rd_2D = abs(lastR_2D - newCal2D.rr);

				if (vd_xy_2D < VD_XY_DIFF_MAX_2D && rd_2D < RD_DIFF_MAX_2D)
				{
					diffSameCount_2D++;
					ical2D_clearFarPoint(newCal2D.rr, newCal2D.Offset[0],newCal2D.Offset[1]);
				}
				else
				{
					diffSameCount_2D = 0;
				}

				D("ical_2D diffSameCount_2D = %d", diffSameCount_2D);
				if (diffSameCount_2D > 1)
				{

					ofsX_2D = lastHard_2D[0] = newCal2D.Offset[0];
					ofsY_2D = lastHard_2D[1] = newCal2D.Offset[1];
					AvgMagR_2D = lastR_2D = newCal2D.rr;

					D("ical_2D ofsX = %d,ofsY = %d",ofsX_2D,ofsY_2D);
					//storeCalibrationToFile(newCal2D, wkCoefficients);
				}
				else {
					lastHard_2D[0] = newCal2D.Offset[0];
					lastHard_2D[1] = newCal2D.Offset[1];
					lastR_2D = newCal2D.rr;
				}

				if (ical2D_readyCheck() >= MAXNUM_BEFORE_ROLLBACK_2D)
				{

					ical2D_shiftData(MAXNUM_BEFORE_ROLLBACK_2D / 2);
					lastCalibCount_2D = -1;

				}

				break;
			}
		}
	}

	if(Is2DReady ==1)
	{
		if(bEnable_ELLIPSE_2D)
		{
			REAL tempCompute[2] = { 0 };
			int MagR_2D;
#if 0			
			memset(buf_qws_double,0,100);
			sprintf(buf_qws_double,"ELLIPSE_2D [%f, %f, %f, %f, %f, %f, %f]\n",
					M_CosT,M_SinT,M_E_2D_ofsX,M_E_2D_ofsY,M_R1,M_R2,M_T); 															
			kal_prompt_trace(MOD_MATV,buf_qws_double);
#endif 
			D("ical_2D ELLIPSE [ %f, %f, %f, %f, %f, %f, %f ]",M_CosT,M_SinT,M_E_2D_ofsX,M_E_2D_ofsY,M_R1,M_R2,M_T);

			
			tempCompute[0] = (M_CosT * (rawMag_X - M_E_2D_ofsX) - M_SinT * (rawMag_Y - M_E_2D_ofsY)) / M_R1;
			tempCompute[1] = (M_SinT * (rawMag_X - M_E_2D_ofsX) + M_CosT * (rawMag_Y - M_E_2D_ofsY)) / M_R2;
			mMagneticData[0] = (M_CosT * tempCompute[0] + M_SinT * tempCompute[1]) * DEFAULT_NORMAL_2D;
			mMagneticData[1] = (M_CosT * tempCompute[1] - M_SinT * tempCompute[0]) * DEFAULT_NORMAL_2D;
				
			MagR_2D = (int) sqrt(mMagneticData[0] * mMagneticData[0] + mMagneticData[1] * mMagneticData[1]);
			lastStatus_2D = checkAccuracy_2D(MagR_2D, lastStatus_2D);
			curAccuracy_2D = lastStatus_2D;
			D("curAccuracy_2D = %d ori=%f", curAccuracy_2D, (atan2(mMagneticData[1],mMagneticData[0])*180/PI)+180);
			//mMagneticEvent->magnetic.status = curAccuracy_2D;
		}
		else
		{
		    int MagR_2D;
			D("ical_2D use offset [%d, %d]", ofsX_2D, ofsY_2D);
			mMagneticData[0] = rawMag_X - ofsX_2D;
			mMagneticData[1] = rawMag_Y - ofsY_2D;

			MagR_2D = (int) sqrt(mMagneticData[0] * mMagneticData[0] + mMagneticData[1] * mMagneticData[1]);
			
			lastStatus_2D = checkAccuracy_2D(MagR_2D, lastStatus_2D);
			curAccuracy_2D = lastStatus_2D;
			//mMagneticEvent->magnetic.status = curAccuracy_2D;
		}
	}
		
	return 1;
}

void CUSTOMER_GET_CLIABPRAM(int *accuracy, EllipsePram *pram){

	//updateEllipseRes();

	*accuracy = curAccuracy_2D;
	if(*accuracy != ACCURACY_HIGH){
		memset(pram, 0, sizeof(EllipsePram));
		pram->R1 = pram->R2 = 1;
	}else{
		pram->CosT = M_CosT;
		pram->SinT = M_SinT;
		pram->E_2D_ofsX = M_E_2D_ofsX;
		pram->E_2D_ofsY = M_E_2D_ofsY;
		pram->R1 = M_R1;
		pram->R2 = M_R2;
		pram->T = M_T;
		pram->rr = AvgMagR_2D;
	}
}

/*
int storeCalibrationToFile(TRANSFORM_2D_T theCal, char* fileName)
{
	D("----------------storeCalibrationToFile start \n");
	int ii = 0;
	FILE* pFile = NULL;
	pFile = fopen(fileName, "wt+");
	if (pFile)
	{
		// Simple calibration coefficient write-out to file	
		if(bEnable_ELLIPSE_2D){
			fprintf(pFile, "%lf	%lf\n", M_E_2D_ofsX, M_E_2D_ofsY);
			fprintf(pFile, "%lf	%lf\n", M_CosT, M_SinT);
			fprintf(pFile, "%lf	%lf\n", M_R1, M_R2);
			fprintf(pFile, "%lf\n", M_T);			
			fprintf(pFile, "%d\n", theCal.rr);
			D("----------------storeCalibrationToFile end\n");
		}else{
			fprintf(pFile, "%d	%d\n", theCal.Offset[0], theCal.Offset[1]);
			fprintf(pFile, "%d\n", theCal.rr);
			D("----------------storeCalibrationToFile end\n");
		}

		fclose(pFile);
		return 1;
	}

	return 0;
}

int getCalibrationFromFile(TRANSFORM_2D_T* pCal, char* fileName)
{
	D("----------------getCalibrationFromFile start \n");
	int ii = 0;
	FILE* pFile = NULL;
	int rs;

	if (!pCal)
		return 0;

	pFile = fopen(fileName, "rt+");
	if (pFile)
	{
		// Simple calibration coefficient read-in from file

		if(bEnable_ELLIPSE_2D){
			fscanf(pFile, "%lf	%lf", &M_E_2D_ofsX, &M_E_2D_ofsY);
			fscanf(pFile, "%lf	%lf", &M_CosT, &M_SinT);
			fscanf(pFile, "%lf	%lf", &M_R1, &M_R2);
			fscanf(pFile, "%lf", &M_T);
			fscanf(pFile, "%d", &(pCal->rr));
			D("----------------getCalibrationFromFile end\n");
		}else{
			fscanf(pFile, "%d	%d\n", &(pCal->Offset[0]), &(pCal->Offset[1]));
			fscanf(pFile, "%d", &(pCal->rr));
			D("----------------getCalibrationFromFile end\n");			
		}
		
		fclose(pFile);
		return 1;
	}
	return 0;
}

*/
