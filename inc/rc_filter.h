#ifndef RC_FILTER_H
#define RC_FILTER_H

//SW RC circuit filter
typedef struct {
	float coeff[2];
	float out[2];

} RCFilter;

void RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTimeS);
float RCFilter_Update(RCFilter *filt, float inp);

#endif