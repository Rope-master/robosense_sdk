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
 * robosense user utility module.
 *
 */


#include "util/robosense_util.h"
#include <stdio.h>


namespace robosense {
namespace perception {


template<typename PointT>
std::string RobosenseUtil<PointT>::obj_types_[5] = {"unKnow", "pedestrian", "nonMot", "smallMot", "bigMot"};


template<typename PointT>
RobosenseUtil<PointT>::RobosenseUtil() {

}


template<typename PointT>
int RobosenseUtil<PointT>::print2file(const std::vector<typename Object<PointT>::Ptr> &object_in,
                                      const std::string& savePath, const std::string& saveName, int saveFrmNumber) {

    if (saveFrmNumber < 0) {
        COUTR("RobosenseUtil: invalid frame number, it should be >=0 !");
        return -1;
    }

    char fullPathBuffer[256] = {0};
    sprintf(fullPathBuffer, "%s/%s_%d.txt", savePath.c_str(), saveName.c_str(), saveFrmNumber);

    std::ofstream ofstr((const char *) fullPathBuffer,
                        std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);

    if (!ofstr.is_open()) {
        COUTR("RobosenseUtil: can Not Open File To Write !");
        return -1;
    }

    // 写入数据
    size_t iter;
    std::string labelName = "";
    for (iter = 0; iter < object_in.size(); ++iter) {
        const typename Object<PointT>::Ptr &object = object_in[iter];

        if (object->type == MAX_OBJ_TYPE_NUM) {// 不处理
            continue;
        }

        labelName.clear();
        labelName = obj_types_[static_cast<int>(object->type)];

        ofstr << labelName << " " << object->tracker_id << " ";
        ofstr << object->center[0] << " " << object->center[1] << " " << object->center[2] << " ";
        ofstr << object->length << " " << object->width << " " << object->height << " ";
        ofstr << 0 << " " << 0 << " " << object->yaw << " ";
        ofstr << 1 << " ";

        // 附加数据
        ofstr << 0 << " ";
        ofstr << 0 << " " << 0 << " " << 0 << " ";
        ofstr << 0 << " " << 0 << " " << 0 << " ";
        ofstr << 0 << " " << 0 << " " << 0 << " ";
        ofstr << 0 << " " << 0 << " " << 0 << " ";

        ofstr << object->type_confidence << " ";

        ofstr << std::endl;
    }

    ofstr.flush();
    ofstr.close();

    return 0;
}


template class RobosenseUtil<pcl::PointXYZI>;

}
}