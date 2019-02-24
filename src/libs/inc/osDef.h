/**
 * osDef.h
 *
 *  Created on: Mar 2, 2018
 *      Author: Kamran Shamaei
 *
 * @brief - This file contains definition of platform architecture
 *
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright [2017-2017] Kamran Shamaei .
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 *
 */


#ifndef SRC_LIBS_INC_OSDEF_H
#define SRC_LIBS_INC_OSDEF_H

namespace tarsim {
#ifdef __ARM_ARCH
	#define EIT_ARM
#elif __linux__
	#define LINUX
#endif
} // end of namespace tarsim
/**
#endif /* SRC_LIBS_INC_OSDEF_H */
