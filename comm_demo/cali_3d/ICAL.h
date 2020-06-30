/*
 * ICAL.h
 *
 *  Created on: 20170228
 *      Author: QST
 */

#ifndef ICAL_H_
#define ICAL_H_

#define	ACCURACY_UNRELIABLE		0
#define	ACCURACY_LOW			1
#define	ACCURACY_MEDIUM			2
#define	ACCURACY_HIGH			3

#define ICAL_SUCCESS				1
#define ICAL_FALSE                  0

// Internal Type Definitions
typedef		char					INT8; 	
typedef		unsigned char			UINT8; 
typedef		short					INT16; 	
typedef		unsigned short			UINT16; 	
typedef		int						INT32; 	
typedef		unsigned int			UINT32; 	
typedef		long long				INT64;
typedef		unsigned long long		UINT64;
typedef		float					REAL;

//typedef		double					REAL;
//typedef		INT8					int8_t;
//typedef		UCHAR					uint8_t;
//typedef		INT16 					int16_t;
//typedef		UINT16 					uint16_t;
//typedef		INT32 					int32_t;
//typedef		UINT32 					uint32_t;

typedef struct
{
	INT16	Offset[3];	// Magnetic Hard Iron x, y, z offsets
	INT16	Matrix[3]; //Kx,Ky,Kz
	INT16	rr;
} TRANSFORM_T;

typedef struct {
	REAL dRawMag[3];
	REAL mag_bias[3];
	REAL data_cali[3];
	REAL pitch,
	      roll,
		  yaw;
	REAL acc[3];
}_QMC7983;

INT64 get_time_in_nanosec(void);
void mcal(long timestamp_ms);
int process(REAL* magData,long timestamp_ms);
void Set_Picth_Roll(float * Pitch, float * Roll);
int get_mag_bias(REAL* mag_bias);
INT8 get_mag_accuracy(void);
int push2mcal(_QMC7983 *data);

#endif /* ICAL_H_ */
