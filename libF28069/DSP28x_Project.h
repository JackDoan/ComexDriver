/*
 * DSP28x_Project.h
 *
 *  Created on: May 20, 2015
 *      Author: jad140230
 */

//#define BIGBOARD
#define LITTLEBOARD

#ifndef DSP28X_PROJECT_H_
#define DSP28X_PROJECT_H_

#ifdef BIGBOARD

#include "F2806x_common/F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_common/F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_common/F2806x_Examples.h"   	// F2806x Examples Include File

#endif

#ifdef LITTLEBOARD

#include "f2802x_common/F2802x_Device.h"


#endif

#include "libSCI.h"
#include "MenuStrings.h"

#endif /* DSP28X_PROJECT_H_ */
