#include <gtest/gtest.h>
#include "pipetools.h"

using namespace zapata;

TEST(PipeTest, processMedianOnVector)
{
	zapata::StdVector<zapata::ValIndex> values;
	values.push_back({1.5,3});
	values.push_back({1.0,1});
	values.push_back({2.0,2});
	int32_t medianIndex;
	PipeTools::processMedianOnVector(values,&medianIndex);
	EXPECT_EQ(medianIndex, 3);
	EXPECT_EQ(values[1].index, 3);
	EXPECT_EQ(values[1].value, 1.5);
	values.clear();
	values.push_back({3.0,3});
	values.push_back({1.0,1});
	values.push_back({2.0,2});
	PipeTools::processMedianOnVector(values,&medianIndex);
	EXPECT_EQ(medianIndex, 2);
	EXPECT_EQ(values[1].index, 2);
	EXPECT_EQ(values[1].value, 2.0);
	values.clear();
	values.push_back({2.0,2});
	values.push_back({3.0,3});
	values.push_back({1.0,1});
	PipeTools::processMedianOnVector(values,&medianIndex);
	EXPECT_EQ(medianIndex, 2);
	EXPECT_EQ(values[1].index, 2);
	EXPECT_EQ(values[1].value, 2.0);
}

TEST(PipeTest, processAverage) {
	zapata::StdVector<double> values;

	EXPECT_EQ(PipeTools::processAverage(values,1.0,1),1.0);
	EXPECT_EQ(PipeTools::processAverage(values,2.0,2),1.5);

	values.clear();
	EXPECT_EQ(PipeTools::processAverage(values,1.0,1),1.0);
	EXPECT_EQ(PipeTools::processAverage(values,2.0,1),2.0);
	values.clear();
	EXPECT_EQ(PipeTools::processAverage(values,1.0,3),1.0);
	EXPECT_EQ(PipeTools::processAverage(values,2.0,3),1.5);
	EXPECT_EQ(PipeTools::processAverage(values,3.0,3),2.0);
}

TEST(PipeTest, processMedian) {
	integrale_s local={0L,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r1={0L,2.0f,2.0f,2.0f,2.0f,2.0f,2.0f,1.0f,2,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r2={0L,3.0f,3.0f,3.0f,3.0f,3.0f,3.0f,1.0f,3,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r3={0L,3.0f,3.0f,3.0f,3.0f,3.0f,3.0f,1.0f,3,integrale_s::INTEGRALE_STATUS_NONE};
	int nbMedian;
	int32_t medianIndex;
	double result=PipeTools::processMedian(local,r1,r2,r3,&nbMedian,[](const integrale_s &r) {
					return r.roll_rate_integral;
				},&medianIndex);
	EXPECT_EQ(result,2.0f);
	EXPECT_EQ(nbMedian,7);
	EXPECT_EQ(medianIndex,2);
}

TEST(PipeTest, processMedian2) {
	integrale_s local={0L,2.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r1={0L,1.0f,2.0f,2.0f,2.0f,2.0f,2.0f,1.0f,2,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r2={0L,3.0f,3.0f,3.0f,3.0f,3.0f,3.0f,1.0f,3,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r3={0L,3.0f,3.0f,3.0f,3.0f,3.0f,3.0f,1.0f,3,integrale_s::INTEGRALE_STATUS_NONE};
	int nbMedian;
	int32_t medianIndex;
	double result=PipeTools::processMedian(local,r1,r2,r3,&nbMedian,[](const integrale_s &r) {
					return r.roll_rate_integral;
				},&medianIndex);
	EXPECT_EQ(result,2.0f);
	EXPECT_EQ(nbMedian,7);
	EXPECT_EQ(medianIndex,1);
}
TEST(PipeTest, processMedian3) {
	integrale_s local={0L,3.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r1={0L,1.0f,2.0f,2.0f,2.0f,2.0f,2.0f,1.0f,2,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r2={0L,2.0f,3.0f,3.0f,3.0f,3.0f,3.0f,1.0f,3,integrale_s::INTEGRALE_STATUS_COMPLETE};
	integrale_s r3={0L,1.5f,3.0f,3.0f,3.0f,3.0f,3.0f,1.0f,3,integrale_s::INTEGRALE_STATUS_NONE};
	int nbMedian;
	int32_t medianIndex;
	double result=PipeTools::processMedian(local,r1,r2,r3,&nbMedian,[](const integrale_s &r) {
					return r.roll_rate_integral;
				},&medianIndex);
	EXPECT_EQ(result,2.0f);
	EXPECT_EQ(nbMedian,7);
	EXPECT_EQ(medianIndex,3);
}

