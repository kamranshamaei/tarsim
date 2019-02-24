
//
// @file: threadUtilities.cpp
//
// @Created on: Jul 22, 2017
// @Author: Kamran Shamaei
//
//
// @brief - Provide thread related utilities such as blocking forever,
//         sleep calls in milliseconds to microseconds.
// <Requirement Doc Reference>
// <Design Doc Reference>
//
// @copyright Copyright Kamran Shamaei
// All Rights Reserved.
//
// This file is subject to the terms and conditions defined in
// file 'LICENSE', which is part of this source code package.
// 
//


#include <pthread.h>
#include <future>
#include <chrono>
#include "threadUtilities.h"

namespace tarsim {
/**
 * @brief if this function is called, it never returns;
 * it uses std::future for an event which will never occur.
 *
 */
void ThreadUtilities::blockForever()
{
    std::promise<void> p;
    p.get_future().wait();

}


/**
 * @brief block the calling thread for n microseconds
 * @param delayMicroSeconds
 */
void ThreadUtilities::waitforMicroseconds(int32_t delayMicroSeconds)
{
    std::this_thread::sleep_for(std::chrono::microseconds(delayMicroSeconds));
}

/**
 * @brief block the calling thread for n milliseconds
 * @param delayMicroSeconds
 */
void ThreadUtilities::waitforMilliseconds(int32_t delayMilliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delayMilliseconds));
}
} // end of namespace tarsim
