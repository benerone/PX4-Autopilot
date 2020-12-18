#include "stdvector.h"
#include <lib/mathlib/mathlib.h>
#include <platforms/posix/apps.h>
#include <uORB/topics/integrale.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <gtest/gtest.h>

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

void init_app_map(apps_map_type &apps) {}

void list_builtins(apps_map_type &apps) {}

float processMedianOnVector(zapata::StdVector<float> &values) {
	if (values.size()==1) {
		return values[0];
	}
	//Sort
	zapata::quicksort(values,0,values.size()-1); //Lowest first
	//Case 2
	if (values.size()==2) {
		return MIN(values[0],values[1]);
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

matrix::Vector3f processMedian(const integrale_s &local,const integrale_s &r1,const integrale_s &r2,const integrale_s &r3,bool * isvalid) {
	matrix::Vector3f result;
	zapata::StdVector<float> rolls;
	zapata::StdVector<float> pitchs;
	zapata::StdVector<float> yows;
	*isvalid=true;
	//Local contrib
	if (local.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		rolls.push_back(local.roll_rate_integral);
		pitchs.push_back(local.pitch_rate_integral);
		yows.push_back(local.yaw_rate_integral);
	}
	//Remote contrib
	if (r1.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		rolls.push_back(r1.roll_rate_integral);
		pitchs.push_back(r1.pitch_rate_integral);
		yows.push_back(r1.yaw_rate_integral);
	}
	if (r2.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		rolls.push_back(r2.roll_rate_integral);
		pitchs.push_back(r2.pitch_rate_integral);
		yows.push_back(r2.yaw_rate_integral);
	}
	if (r3.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		rolls.push_back(r3.roll_rate_integral);
		pitchs.push_back(r3.pitch_rate_integral);
		yows.push_back(r3.yaw_rate_integral);
	}
	if (rolls.size()==0) {
		//Case no valid value
		*isvalid=false;
		return matrix::Vector3f(0.0f,0.0f,0.0f);
	}
	result(0)=processMedianOnVector(rolls);
	result(1)=processMedianOnVector(pitchs);
	result(2)=processMedianOnVector(yows);
	return result;
}




matrix::Vector3f pipeIntegrale(matrix::Vector3f & _pi_coef,matrix::Vector3f &_pi_limit,
				uORB::Subscription &_integrale_sub,uORB::Subscription &_r1integrale_sub,uORB::Subscription &_r2integrale_sub,uORB::Subscription &_r3integrale_sub) {
	integrale_s _r1integrale;
	integrale_s _r2integrale;
	integrale_s _r3integrale;
	integrale_s _lintegrale;

	//Get local
	if (!_integrale_sub.copy(&_lintegrale)) {
		_lintegrale.status=integrale_s::INTEGRALE_STATUS_NONE;
	}
	//Get remote integrale
	if (!_r1integrale_sub.copy(&_r1integrale)) {
		_r1integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
	}
	if (!_r2integrale_sub.copy(&_r2integrale)) {
		_r2integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
	}
	if (!_r3integrale_sub.copy(&_r3integrale)) {
		_r3integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
	}

	//Process Median
	bool isValid=true;
	matrix::Vector3f correction=matrix::Vector3f(0.0f,0.0f,0.0f);
	matrix::Vector3f imedian=processMedian(_lintegrale,_r1integrale,_r2integrale,_r3integrale,&isValid);
	if (!isValid) {
		return correction;
	}
	//Process in pwm
	matrix::Vector3f ilocal=matrix::Vector3f(_lintegrale.roll_rate_integral,_lintegrale.pitch_rate_integral,_lintegrale.yaw_rate_integral);

	matrix::Vector3f imedian_pwm=(imedian.emult(_pi_coef)*500.0f)+1000.0f;
	matrix::Vector3f ilocal_pwm=(ilocal.emult(_pi_coef)*500.0f)+1000.0f;
	matrix::Vector3f ierror_pwm=ilocal_pwm-imedian_pwm;

	for(int i=0;i<3;i++) {
		correction(i)=ierror_pwm(i);

		if (correction(i)<(-_pi_limit(i)) ||correction(i)>_pi_limit(i)) {
			if (correction(i)<0) correction(i)=-_pi_limit(i);
			if (correction(i)>0) correction(i)=_pi_limit(i);
		}
	}
	return correction;

}

class IntegralePipeTest : public ::testing::Test
{

};

TEST_F(IntegralePipeTest, testMedian)
{
	zapata::StdVector<float> values;
	zapata::StdVector<float> values2;
	zapata::StdVector<float> values3;
	zapata::StdVector<float> values4;


	values.push_back(1.0f);
	EXPECT_FLOAT_EQ(1.0f,processMedianOnVector(values));
	values2.push_back(2.0f);
	values2.push_back(1.0f);
	EXPECT_FLOAT_EQ(1.0f,processMedianOnVector(values2));
	EXPECT_EQ(true, values2[0]<values2[1]);
	values3.push_back(2.0f);
	values3.push_back(1.0f);
	values3.push_back(5.0f);
	EXPECT_FLOAT_EQ(2.0f,processMedianOnVector(values3));
	EXPECT_EQ(true, values3[0]<values3[1]);
	EXPECT_EQ(true, values3[1]<values3[2]);
	values4.push_back(2.0f);
	values4.push_back(1.0f);
	values4.push_back(5.0f);
	values4.push_back(3.0f);
	EXPECT_FLOAT_EQ(2.0f,processMedianOnVector(values4));
	EXPECT_EQ(true, values4[0]<values4[1]);
	EXPECT_EQ(true, values4[1]<values4[2]);
	EXPECT_EQ(true, values4[2]<values4[3]);
}

TEST_F(IntegralePipeTest, testProcessMedian) {
	integrale_s l;
	integrale_s r1;
	integrale_s r2;
	integrale_s r3;
	bool isValid;

	l={0L,0.0f,0.0f,0.0f,integrale_s::INTEGRALE_STATUS_NONE};
	r1={0L,0.0f,0.0f,0.0f,integrale_s::INTEGRALE_STATUS_NONE};
	r2={0L,0.0f,0.0f,0.0f,integrale_s::INTEGRALE_STATUS_NONE};
	r3={0L,0.0f,0.0f,0.0f,integrale_s::INTEGRALE_STATUS_NONE};
	matrix::Vector3f result1=processMedian(l,r1,r2,r3,&isValid);
	EXPECT_FLOAT_EQ(0.0f,result1(0));
	EXPECT_FLOAT_EQ(0.0f,result1(1));
	EXPECT_FLOAT_EQ(0.0f,result1(2));
	EXPECT_EQ(false, isValid);

	l={0L,4.0f,2.0f,5.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r1={0L,3.0f,4.0f,7.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r2={0L,1.0f,5.0f,3.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r3={0L,0.0f,0.0f,0.0f,integrale_s::INTEGRALE_STATUS_NONE};
	matrix::Vector3f result2=processMedian(l,r1,r2,r3,&isValid);
	EXPECT_FLOAT_EQ(3.0f,result2(0));
	EXPECT_FLOAT_EQ(4.0f,result2(1));
	EXPECT_FLOAT_EQ(5.0f,result2(2));
	EXPECT_EQ(true, isValid);

	l={0L,4.0f,2.0f,5.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r1={0L,3.0f,4.0f,7.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r2={0L,1.0f,5.0f,3.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r3={0L,0.0f,0.0f,0.0f,integrale_s::INTEGRALE_STATUS_NONE};
	matrix::Vector3f result3=processMedian(l,r1,r2,r3,&isValid);
	EXPECT_FLOAT_EQ(3.0f,result3(0));
	EXPECT_FLOAT_EQ(4.0f,result3(1));
	EXPECT_FLOAT_EQ(5.0f,result3(2));
	EXPECT_EQ(true, isValid);
	l={0L,4.0f,2.0f,40.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r1={0L,3.0f,-10.0f,7.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r2={0L,1.0f,5.0f,3.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	r3={0L,9.0f,4.0f,5.0f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	matrix::Vector3f result4=processMedian(l,r1,r2,r3,&isValid);
	EXPECT_FLOAT_EQ(3.0f,result4(0));
	EXPECT_FLOAT_EQ(4.0f,result4(1));
	EXPECT_FLOAT_EQ(5.0f,result4(2));
	EXPECT_EQ(true, isValid);
}
/*
TEST_F(IntegralePipeTest, testProcessPipeBasic) {

	uORB::Publication<integrale_s>	_integrales_pub{ORB_ID(integrale)};
	uORB::Publication<integrale_s>	_r1integrales_pub{ORB_ID(r1integrale)};
	uORB::Publication<integrale_s>	_r2integrales_pub{ORB_ID(r2integrale)};
	uORB::Publication<integrale_s>	_r3integrales_pub{ORB_ID(r3integrale)};

	uORB::Subscription _r1integrale_sub{ORB_ID(r1integrale)};
	uORB::Subscription _r2integrale_sub{ORB_ID(r2integrale)};
	uORB::Subscription _r3integrale_sub{ORB_ID(r3integrale)};
	uORB::Subscription _integrale_sub{ORB_ID(integrale)};

	integrale_s integrale_data{};
	integrale_data.timestamp= hrt_absolute_time();
	integrale_data.roll_rate_integral=1000.0f;
	integrale_data.pitch_rate_integral=1001.0f;
	integrale_data.yaw_rate_integral=1002.0f;
	integrale_data.status=integrale_s::INTEGRALE_STATUS_COMPLETE;
	_integrales_pub.publish(integrale_data);


	integrale_s l;
	integrale_s r1;
	integrale_s r2;
	integrale_s r3;

	if (!_integrale_sub.copy(&l)) {
		ASSERT_TRUE(false);
	}
	EXPECT_FLOAT_EQ(1000.0f,l.roll_rate_integral);
	EXPECT_FLOAT_EQ(1001.0f,l.pitch_rate_integral);
	EXPECT_FLOAT_EQ(1002.0f,l.yaw_rate_integral);
	ASSERT_TRUE(integrale_s::INTEGRALE_STATUS_COMPLETE==l.status);


}*/
TEST_F(IntegralePipeTest, testProcessPipe) {

	uORB::Publication<integrale_s>	_integrales_pub{ORB_ID(integrale)};
	uORB::Publication<integrale_s>	_r1integrales_pub{ORB_ID(r1integrale)};
	uORB::Publication<integrale_s>	_r2integrales_pub{ORB_ID(r2integrale)};
	uORB::Publication<integrale_s>	_r3integrales_pub{ORB_ID(r3integrale)};

	uORB::Subscription _r1integrale_sub{ORB_ID(r1integrale)};
	uORB::Subscription _r2integrale_sub{ORB_ID(r2integrale)};
	uORB::Subscription _r3integrale_sub{ORB_ID(r3integrale)};
	uORB::Subscription _integrale_sub{ORB_ID(integrale)};

	matrix::Vector3f _pi_coef=matrix::Vector3f(1.0f,2.0f,3.0f);
	matrix::Vector3f _pi_limit=matrix::Vector3f(40.0f,50.0f,60.0f);

	integrale_s ls={0L,0.2f,0.2f,0.2f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r1={0L,0.21f,0.21f,0.21f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r2={0L,0.22f,0.22f,0.22f,integrale_s::INTEGRALE_STATUS_COMPLETE};
	//integrale_s r3={0L,0.22f,0.22f,0.22f,integrale_s::INTEGRALE_STATUS_NONE};
	_integrales_pub.publish(ls);
	_r1integrales_pub.publish(r1);
	_r2integrales_pub.publish(r2);

	matrix::Vector3f c1=pipeIntegrale(_pi_coef,_pi_limit,_integrale_sub,_r1integrale_sub,_r2integrale_sub,_r3integrale_sub);
	EXPECT_FLOAT_EQ(-0.01f*500.0f,c1(0));
	EXPECT_FLOAT_EQ(-0.02f*500.0f,c1(1));
	EXPECT_FLOAT_EQ(-0.03f*500.0f,c1(2));

	_pi_coef=matrix::Vector3f(10.0f,20.0f,30.0f);
	_integrales_pub.publish(ls);
	_r1integrales_pub.publish(r1);
	_r2integrales_pub.publish(r2);
	matrix::Vector3f c2=pipeIntegrale(_pi_coef,_pi_limit,_integrale_sub,_r1integrale_sub,_r2integrale_sub,_r3integrale_sub);
	EXPECT_FLOAT_EQ(-40.0f,c2(0));
	EXPECT_FLOAT_EQ(-50.0f,c2(1));
	EXPECT_FLOAT_EQ(-60.0f,c2(2));

	_pi_coef=matrix::Vector3f(-1.0f,-2.0f,-3.0f);
	_integrales_pub.publish(ls);
	_r1integrales_pub.publish(r1);
	_r2integrales_pub.publish(r2);

	matrix::Vector3f c3=pipeIntegrale(_pi_coef,_pi_limit,_integrale_sub,_r1integrale_sub,_r2integrale_sub,_r3integrale_sub);
	EXPECT_FLOAT_EQ(0.01f*500.0f,c3(0));
	EXPECT_FLOAT_EQ(0.02f*500.0f,c3(1));
	EXPECT_FLOAT_EQ(0.03f*500.0f,c3(2));

	_pi_coef=matrix::Vector3f(-10.0f,-20.0f,-30.0f);
	_integrales_pub.publish(ls);
	_r1integrales_pub.publish(r1);
	_r2integrales_pub.publish(r2);
	matrix::Vector3f c4=pipeIntegrale(_pi_coef,_pi_limit,_integrale_sub,_r1integrale_sub,_r2integrale_sub,_r3integrale_sub);
	EXPECT_FLOAT_EQ(40.0f,c4(0));
	EXPECT_FLOAT_EQ(50.0f,c4(1));
	EXPECT_FLOAT_EQ(60.0f,c4(2));


	zapata::StdVector<matrix::Vector3f> accuCorrection;
	accuCorrection.push_back(matrix::Vector3f(1.0f,2.0f,3.0f));
	accuCorrection.push_back(matrix::Vector3f(2.0f,3.0f,4.0f));
	accuCorrection.push_back(matrix::Vector3f(3.0f,4.0f,5.0f));
	accuCorrection.push_back(matrix::Vector3f(4.0f,5.0f,6.0f));
	accuCorrection.push_back(matrix::Vector3f(5.0f,6.0f,7.0f));
	matrix::Vector3f finalCorrection=matrix::Vector3f(0.0f,0.0f,0.0f);
	for(unsigned int i=0;i<accuCorrection.size();i++) {
		finalCorrection+=accuCorrection[i];
	}
	EXPECT_FLOAT_EQ(15.0f,finalCorrection(0));
	EXPECT_FLOAT_EQ(20.0f,finalCorrection(1));
	EXPECT_FLOAT_EQ(25.0f,finalCorrection(2));
	finalCorrection=finalCorrection/accuCorrection.size();
	EXPECT_FLOAT_EQ(3.0f,finalCorrection(0));
	EXPECT_FLOAT_EQ(4.0f,finalCorrection(1));
	EXPECT_FLOAT_EQ(5.0f,finalCorrection(2));
	accuCorrection.push_back(matrix::Vector3f(6.0f,7.0f,8.0f));
	for(unsigned int i=0;i<accuCorrection.size()-1;i++) {
		accuCorrection[i]=accuCorrection[i+1];
	}
	accuCorrection.pop_back();
	finalCorrection=matrix::Vector3f(0.0f,0.0f,0.0f);
	for(unsigned int i=0;i<accuCorrection.size();i++) {
		finalCorrection+=accuCorrection[i];
	}
	EXPECT_FLOAT_EQ(20.0f,finalCorrection(0));
	EXPECT_FLOAT_EQ(25.0f,finalCorrection(1));
	EXPECT_FLOAT_EQ(30.0f,finalCorrection(2));
	finalCorrection=finalCorrection/accuCorrection.size();
	EXPECT_FLOAT_EQ(4.0f,finalCorrection(0));
	EXPECT_FLOAT_EQ(5.0f,finalCorrection(1));
	EXPECT_FLOAT_EQ(6.0f,finalCorrection(2));
}
