#include "statistic.h"

float Mean(float* sample,u16 N)
{
	float sum=0.0;
	u16 i;
	for(i=0;i<N;i++)
	{
		sum += sample[i];
	}
	return sum/N;
}

float Var(float* sample,u16 N)
{
	float mean;
	u16 i;
	float var=0.0;
	mean = Mean(sample,N);
	for(i=0;i<N;i++)
	{
		var += (sample[i]-mean)*(sample[i]-mean);
	}
	var /= N;
	
	return var;
}
