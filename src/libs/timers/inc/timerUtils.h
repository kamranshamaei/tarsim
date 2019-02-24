/**
 *
 * @file: timerUtils.h
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - Provide interface definition for time functions related utilities
 *          such as creating timer event
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

#ifndef SRC_TIMERUTILS_H_
#define SRC_TIMERUTILS_H_

#include "msgQClient.h"
#include <mutex>

namespace tarsim {
enum class NotifyMethod
{
    NotifyMq,
    NotifyMutex
};


class TimerUtils
{
public:
    TimerUtils();
    TimerUtils(const std::string &qName);
    virtual ~TimerUtils();
    Errors FireTimer(int32_t timeMs, NotifyMethod notification);
    void CancelTimer() { m_cancelTimer = true;}

private:

    MsgQClient *m_caller = nullptr;
    std::string m_queName = "";
    std::mutex m_mutex {};
    bool m_cancelTimer = false;
};
} // end of namespace tarsim
#endif /* SRC_TIMERUTILS_H_ */
