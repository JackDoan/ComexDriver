// TITLE:   Pos/speed measurement using EQEP peripheral
// Header file containing data type and object definitions and
// initializers.

#ifndef __POSSPEED__
#define __POSSPEED__
#include "F2806x_Cla_typedefs.h"

//Define the structure of the POSSPEED Object
typedef struct {float theta_elec;       // Output: Motor Electrical angle
                float theta_mech;       // Output: Motor Mechanical Angle
                int DirectionQep;       // Output: Motor rotation direction
                int QEP_cnt_idx;        // Variable: Encoder counter index
                unsigned int theta_raw;          // Variable: Raw angle from Timer 2
                int mech_scaler;        // Parameter: 0.9999/total count, total count = 6400
                int pole_pairs;         // Parameter: Number of pole pairs (Q0)
                int cal_angle;          // Parameter: Raw angular offset between encoder and phase A
                int index_sync_flag;    // Output: Index sync status

                Uint32 SpeedScaler;     // Parameter :  Scalar converting 1/N cycles to a speed ///CALCULATE THIS
                float Speed_pr;         // Output :  speed in per-unit
                Uint32 BaseRpm;         // Parameter : maximum RPM
                int SpeedRpm_pr;      // Output : speed in r.p.m. (Q0) - independently with global Q

                float  oldpos;          // Input: Electrical angle (pu)
                float Speed_fr;         // Output :  speed in per-unit
                int SpeedRpm_fr;      // Output : Speed in rpm
                void (*init)();         // Pointer to the init funcion
                void (*calc)();         // Pointer to the calc funtion
                }  POSSPEED;

//Default initializer for the POSSPEED Object.
#define POSSPEED_DEFAULTS {\
	0x0,\
	0x0,\
	0x0,\
	0x0,\
	0x0,\
	25600,\
	12,\
	0,\
	0x0,\
    80,\
	0,\
	3200,\
	0,\
    0,\
	0,\
	0,\
   (void (*)(long))POSSPEED_Init, (void (*)(long))POSSPEED_Calc}  //pointers to functions


//Define a POSSPEED_handle

typedef POSSPEED *POSSPEED_handle;


//Prototypes for the functions in posspeed.c
void POSSPEED_Init(void);
void POSSPEED_Calc(POSSPEED_handle);

#endif /*  __POSSPEED__ */

