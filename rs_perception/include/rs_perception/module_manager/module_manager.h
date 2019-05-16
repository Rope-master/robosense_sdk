
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
 * robosense module manager, for sdk modules authority management.
 *
 */

#ifndef ROBOSENSE_MODULE_MANAGER_H
#define ROBOSENSE_MODULE_MANAGER_H

#include <string>
#include <memory>

namespace robosense {
namespace perception {

class alignas(16) ModuleManager {
	public:

	typedef std::shared_ptr<ModuleManager> Ptr;
	typedef std::shared_ptr<const ModuleManager> ConstPtr;

	ModuleManager(const std::string &key_path = ".", const std::string &pcap_path = "", const unsigned short &udp_port = 6699);

	static bool initModule();

	private:

	class ModuleManagerInternal;
	ModuleManagerInternal* Internal;

};


}
}

#endif