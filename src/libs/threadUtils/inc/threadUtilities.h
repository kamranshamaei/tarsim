/**
 *
 * @file: threadUtilities.h
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - Provide thread related utilities such as blocking forever,
 *         sleep calls in milliseconds to microseconds.
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright Kamran Shamaei
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 *
 */
#ifndef SRC_LIBS_THREADUTILS_INC_THREADUTILS_H_
#define SRC_LIBS_THREADUTILS_INC_THREADUTILS_H_

#include <pthread.h>
#include <thread>
#include <stdint.h>

namespace tarsim {
class ThreadUtilities
{
public:
    ThreadUtilities() {}; // do nothing
    virtual ~ThreadUtilities(); // do nothing
    static void blockForever();
    static void waitforMilliseconds(int32_t delayMilliseconds);
    static void waitforMicroseconds(int32_t delayMicroSeconds);
};
} // end of namespace tarsim
#endif /* SRC_LIBS_THREADUTILS_INC_THREADUTILS_H_ */
