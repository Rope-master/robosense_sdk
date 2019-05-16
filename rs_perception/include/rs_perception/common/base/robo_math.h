
/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: robosense Perception Group
 * Version: 2.0.0
 * Date: 2018.6
 *
 * DESCRIPTION
 *
 * robosense common math module, including common used algebra, trigonometric, etc.
 *
 */

#ifndef ROBOSENSE_MATH_H
#define ROBOSENSE_MATH_H
#include "rs_perception/common/base/basic_types.h"

namespace robosense {
namespace perception {

struct alignas(16) MathUtil {


	static float gaussianWeight(float val, float sigma, float mean = 0.f);

	static float gaussianProb(float val, float sigma, float mean = 0.f);

	static float calcAngleErr2D(const float &a, const float &b);

	static float calcAngleSum2D(const float &a, const float &b);

	static float normalizeAngle(const float &a);

	template<typename T>
	static int sign(const T &val) {
		return val >= 0 ? 1 : -1;
	}

	static bool inPolygon(int nvert, float *vertx, float *verty, float testx, float testy);

};


}
}



#endif