#include "filter.h"

const float GCoef[5][10]={{0.2146,0.2051,0.1792,0.1431,0.1045,0.0697,0.0425,0.0237,0.0120,0.0056},  //10
					{0.2357,0.2229,0.1887,0.1429,0.0969,0.0588,0.0319,0.0155,0.0067,0.0},
					{0.2613,0.2436,0.1973,0.1388,0.0848,0.0451,0.0208,0.0083,0.0   ,0.0},
					{0.2933,0.2675,0.2031,0.1283,0.0675,0.0295,0.0108,0.0   ,0.0   ,0.0},
					{0.3341,0.2949,0.2027,0.1085,0.0452,0.0147,0.0   ,0.0   ,0.0   ,0.0}};
					
float GaussianFilter(GFilterType *gft,float newData)
{
	u8 i;
	float output=0.0;
	gft->pool[gft->head]=newData;
	for(i=0;i<gft->length;i++)
	{
		output+=gft->pool[(gft->head+i)%(gft->length)]*GCoef[10-gft->length][i];
	}
	if(gft->head==0) gft->head=gft->length-1;
	else gft->head--;

	if(gft->pool[gft->head]<0.00001 && gft->pool[gft->head]>-0.00001)
		return newData;
	return output;
}					
