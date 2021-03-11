
#include "pipetools.h"

using namespace zapata;

float PipeTools::processMedian(const integrale_s &local,const integrale_s &r1,const integrale_s &r2,const integrale_s &r3,int * nbMedian,FieldSelectorCallback fcb) {
	zapata::StdVector<float> allValues(4);
	(*nbMedian)=0;
	//Local contrib
	if (local.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		allValues.push_back((*fcb)(local));
		(*nbMedian)|=1;
	}
	//Remote contrib
	if (r1.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		allValues.push_back((*fcb)(r1));
		(*nbMedian)|=1;
	}
	if (r2.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		allValues.push_back((*fcb)(r2));
		(*nbMedian)|=4;
	}
	if (r3.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		allValues.push_back((*fcb)(r3));
		(*nbMedian)|=8;
	}
	return processMedianOnVector(allValues);
}

float PipeTools::processMedianOnVector(zapata::StdVector<float> &values) {
	if (values.size()==1) {
		return values[0];
	}
	//Sort
	zapata::quicksort(values,0,values.size()-1); //Lowest first
	//Case 2
	if (values.size()==2) {
		//if (sys_id==1) {
			return values[0];
		/*} else {
			return values[1];
		}*/
	}

	//if 4 values , remove farthest
	if (values.size()==4) {
		float distLow=values[1]-values[0];
		float distHigh=values[3]-values[2];
		if (distHigh<distLow) {
			values[0]=values[1];
			values[1]=values[2];
			values[2]=values[3];
		}
		values.pop_back();
	}
	//Case 3
	return values[1];
}
float PipeTools::processAverage(zapata::StdVector<float> & acc,float value,int32_t nbAverage) {
	acc.push_back(value);
	if (acc.size()>(unsigned int)nbAverage) {
		for(unsigned int i=0;i<acc.size()-1;i++) {
			acc[i]=acc[i+1];
		}
		acc.pop_back();
	}
	float result=0.0f;
	for(unsigned int i=0;i<acc.size();i++) {
		result+=acc[i];
	}
	return result/(float)acc.size();
}
float PipeTools::processMultAndClamp(float val,float mult,float clamp) {
	float result=val*mult;
	float limit=abs(clamp);
	if (result>limit) {
		return limit;
	}
	if (result<-limit) {
		return -limit;
	}
	return result;
}
bool PipeTools::isIntegraleValid(const integrale_s &ival) {
	return PX4_ISFINITE(ival.roll_rate_integral) &&
		PX4_ISFINITE(ival.pitch_rate_integral) &&
		PX4_ISFINITE(ival.yaw_rate_integral)  &&
		PX4_ISFINITE(ival.thrust) &&
		ival.status==integrale_s::INTEGRALE_STATUS_COMPLETE;
}
