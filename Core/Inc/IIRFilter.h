/*
 * IIRFilter.h
 *
 *  Created on: Mar 28, 2023
 *      Author: fuadzade
 *
 *  2nd order IIR Butterworth Filter designed on https://www.micromodeler.com/dsp/
 *
 */

#ifndef INC_IIRFILTER_H_
#define INC_IIRFILTER_H_

struct IIRFilter
{
	float out[3];
	float in[3];

	float out_coef[3];
	float in_coef[3];
};

void IIR_Init(struct IIRFilter *filter, float oc1, float oc2, float ic0, float ic1, float ic2);
float IIR_Update(struct IIRFilter *filter, float in);

#endif /* INC_IIRFILTER_H_ */
