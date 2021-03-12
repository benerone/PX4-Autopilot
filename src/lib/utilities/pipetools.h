
#pragma once
#include <px4_platform_common/defines.h>
#include <uORB/topics/integrale.h>
#include "stdvector.h"

namespace zapata {

	typedef float (*FieldSelectorCallback) (const integrale_s &r) ;

	typedef struct {
		float value;
		int index;
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
		static float processMedian(const integrale_s &local,const integrale_s &r1,const integrale_s &r2,const integrale_s &r3,int * nbMedian,FieldSelectorCallback fcb,int * medianIndex);
		/**
		 * @brief Process median on array of float values
		 *
		 * @param values
		 * @return float
		 */
		static float processMedianOnVector(zapata::StdVector<ValIndex> &values,int * medianIndex);
		/**
		 * @brief Process average
		 *
		 */
		static float processAverage(zapata::StdVector<float> & acc,float value,int32_t nbAverage);
		/**
		 * @brief Multiplie and clamp
		 *
		 */
		static float processMultAndClamp(float val,float mult,float clamp);
		/**
		 * @brief test if integrale is valid
		 *
		 */
		static bool isIntegraleValid(const integrale_s &ival);
	};

}
