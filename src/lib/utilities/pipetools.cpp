
#include "pipetools.h"

using namespace zapata;



double PipeTools::processMedian(const integrale_s &local,const integrale_s &r1,const integrale_s &r2,const integrale_s &r3,int * nbMedian,FieldSelectorCallback fcb,int32_t * medianIndex) {
	zapata::StdVector<ValIndex> allValues;
	(*nbMedian)=0;
	//Local contrib
	if (local.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		allValues.push_back({(double)(*fcb)(local),local.index});
		(*nbMedian)|=1;
	}
	//Remote contrib
	if (r1.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		allValues.push_back({(double)(*fcb)(r1),r1.index});
		(*nbMedian)|=2;
	}
	if (r2.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		allValues.push_back({(double)(*fcb)(r2),r2.index});
		(*nbMedian)|=4;
	}
	if (r3.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		allValues.push_back({(double)(*fcb)(r3),r3.index});
		(*nbMedian)|=8;
	}
	return processMedianOnVector(allValues,medianIndex);
}
double PipeTools::processMedianVSP(const vehicle_share_position_s &local,
				   const vehicle_share_position_s &r1,
				   const vehicle_share_position_s &r2,
				   const vehicle_share_position_s &r3,
				   int * nbMedian,
				   FieldSelectorCallbackVSP fcb,
				   ValiderCallbackVSP vcb,
				   int32_t * medianIndex) {

	zapata::StdVector<ValIndex> allValues;
	(*nbMedian)=0;
	//Local contrib
	if (local.status==vehicle_share_position_s::VSP_STATUS_COMPLETE && vcb(local)) {
		allValues.push_back({(double)(*fcb)(local),local.index});
		(*nbMedian)|=1;
	}
	//Remote contrib
	if (r1.status==vehicle_share_position_s::VSP_STATUS_COMPLETE && vcb(r1)) {
		allValues.push_back({(double)(*fcb)(r1),r1.index});
		(*nbMedian)|=2;
	}
	if (r2.status==vehicle_share_position_s::VSP_STATUS_COMPLETE && vcb(r2)) {
		allValues.push_back({(double)(*fcb)(r2),r2.index});
		(*nbMedian)|=4;
	}
	if (r3.status==vehicle_share_position_s::VSP_STATUS_COMPLETE && vcb(r3)) {
		allValues.push_back({(double)(*fcb)(r3),r3.index});
		(*nbMedian)|=8;
	}
	return processMedianOnVector(allValues,medianIndex);
}

double PipeTools::processMedianOnVector(zapata::StdVector<ValIndex> &values,int32_t * medianIndex) {
	if (values.size()==1) {
		return values[0].value;
	}
	//Sort
	zapata::quicksortValues(values,0,values.size()-1); //Lowest first
	//Case 2
	if (values.size()==2) {
		if (values[0].index<values[1].index) {
			(* medianIndex)=values[0].index;
			return values[0].value;
		} else {
			(* medianIndex)=values[1].index;
			return values[1].value;
		}
	}

	//if 4 values , remove farthest
	if (values.size()==4) {
		float distLow=values[1].value-values[0].value;
		float distHigh=values[3].value-values[2].value;
		if (distHigh<distLow) {
			values[0]=values[1];
			values[1]=values[2];
			values[2]=values[3];
		}
		values.pop_back();
	}
	//Case 3
	(* medianIndex)=values[1].index;
	return values[1].value;
}
double PipeTools::processAverage(zapata::StdVector<double> & acc,double value,int32_t nbAverage) {
	acc.push_back(value);
	if (acc.size()>(unsigned int)nbAverage) {
		for(unsigned int i=0;i<acc.size()-1;i++) {
			acc[i]=acc[i+1];
		}
		acc.pop_back();
	}
	double result=0.0f;
	for(unsigned int i=0;i<acc.size();i++) {
		result+=acc[i];
	}
	return result/(double)acc.size();
}
double PipeTools::processMultAndClamp(double val,double mult,double clamp) {
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
		PX4_ISFINITE(ival.vx) &&
		PX4_ISFINITE(ival.vy) &&
		PX4_ISFINITE(ival.thrust) &&
		ival.status==integrale_s::INTEGRALE_STATUS_COMPLETE;
}
bool PipeTools::isVSPValid(const vehicle_share_position_s &val) {
	return PX4_ISFINITE(val.x) &&
		PX4_ISFINITE(val.y) &&
		PX4_ISFINITE(val.z)  &&
		PX4_ISFINITE(val.vx) &&
		PX4_ISFINITE(val.vy) &&
		PX4_ISFINITE(val.vz) &&
		val.status==vehicle_share_position_s::VSP_STATUS_COMPLETE;
}
