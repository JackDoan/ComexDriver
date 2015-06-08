/*
 * adc.h
 *
 *  Created on: Jun 8, 2015
 *      Author: jad140230
 */

#ifndef ADC_H_
#define ADC_H_

__interrupt void adc_isr(void);
void ConfigureADC(void);


// Global variables used in this example:
extern Uint16 LoopCount;
extern Uint16 ConversionCount;
extern Uint16 Voltage1[10];
extern Uint16 Voltage2[10];




#endif /* ADC_H_ */
