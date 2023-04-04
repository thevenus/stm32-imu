/*
 * IIRFilter.c
 *
 *  Created on: Mar 28, 2023
 *      Author: fuadzade
 */

#include "IIRFilter.h"
#include "stdint.h"
#include "stddef.h"

void IIR_Init(struct IIRFilter *filter, float oc1, float oc2, float ic0, float ic1, float ic2)
{
	for (uint8_t i = 0; i<3; i++) {
		filter->out[i] = 0;
		filter->in[i] = 0;
	}

	filter->out_coef[0] = 1;
	filter->out_coef[1] = oc1;
	filter->out_coef[2] = oc2;

	filter->in_coef[0] = ic0;
	filter->in_coef[1] = ic1;
	filter->in_coef[2] = ic2;
}

float IIR_Update(struct IIRFilter *filter, float in)
{
	// shift input and add new one
	filter->in[2] = filter->in[1];
	filter->in[1] = filter->in[0];
	filter->in[0] = in;

	// shift output and calculate the new output
	filter->out[2] = filter->out[1];
	filter->out[1] = filter->out[0];
	filter->out[0] = filter->out_coef[1] * filter->out[1] + filter->out_coef[2] * filter->out[2] +
			filter->in_coef[0] * filter->in[0] + filter->in_coef[1] * filter->in[1] + filter->in_coef[2] * filter->in[2];

	return filter->out[0];
}
