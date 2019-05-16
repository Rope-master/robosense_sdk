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
 * robosense classification module, for object recognition.
 *
 */

#ifndef CLASSIFIER_BASE_0_H
#define CLASSIFIER_BASE_0_H
#include <iostream>
#include <fstream>
#include <vector>


namespace robosense {
namespace perception {


std::vector<float> classifierBase0(std::vector<float> &sample);

}
}

#endif
