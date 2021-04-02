
#pragma once
#include <px4_platform_common/defines.h>
#include <uORB/topics/integrale.h>
#include <uORB/topics/vehicle_share_position.h>
#include "stdvector.h"

namespace zapata {

	typedef float (*FieldSelectorCallback) (const integrale_s &r) ;
	typedef float (*FieldSelectorCallbackVSP) (const vehicle_share_position_s &r) ;
	typedef bool (*ValiderCallbackVSP) (const vehicle_share_position_s &r) ;

	typedef struct {
		double value;
		int32_t index;
	} ValIndex;

	class PipeTools {
	public:
		/**
		 * @brief Process median
		 *
		 * @param local local integrale
		 * @param r1 r1 integrale
		 * @param r2 r2 integrale
		 * @param r3 r3 integrale
		 * @param nbMedian nbMedian
		 * @param fcb Filed selector callback
		 * @return float
		 */
		static double processMedian(const integrale_s &local,const integrale_s &r1,const integrale_s &r2,const integrale_s &r3,int * nbMedian,FieldSelectorCallback fcb,int32_t * medianIndex);
		/**
		 * @brief Process median vehicle_share_position
		 *
		 * @param local local vehicle_share_position
		 * @param r1 r1 vehicle_share_position
		 * @param r2 r2 vehicle_share_position
		 * @param r3 r3 vehicle_share_position
		 * @param nbMedian nbMedian
		 * @param fcb Filed selector callback
		 * @return float
		 */
		static double processMedianVSP(const vehicle_share_position_s &local,
					       const vehicle_share_position_s &r1,
					       const vehicle_share_position_s &r2,
					       const vehicle_share_position_s &r3,
					       int * nbMedian,
					       FieldSelectorCallbackVSP fcb,
					       ValiderCallbackVSP vcb,
					       int32_t * medianIndex);
		/**
		 * @brief Process median on array of float values
		 *
		 * @param values
		 * @return float
		 */
		static double processMedianOnVector(zapata::StdVector<ValIndex> &values,int32_t * medianIndex);
		/**
		 * @brief Process average
		 *
		 */
		static double processAverage(zapata::StdVector<double> & acc,double value,int32_t nbAverage);
		/**
		 * @brief Multiplie and clamp
		 *
		 */
		static double processMultAndClamp(double val,double mult,double clamp);
		/**
		 * @brief test if integrale is valid
		 *
		 */
		static bool isIntegraleValid(const integrale_s &ival);
		/**
		 * @brief test if vehicle_share_position is valid
		 *
		 */
		static bool isVSPValid(const vehicle_share_position_s &val);
	};

}
