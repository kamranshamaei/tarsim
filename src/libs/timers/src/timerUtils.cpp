
//
// @file: timerUtils.cpp
//
// @Created on: Jul 22, 2017
// @Author: Kamran Shamaei
//
//
// @brief - Provide interface definition for time functions related utilities
//          such as creating timer event
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

#include "timerUtils.h"
#include "msgQClient.h"
#include <chrono>
#include <thread>
#include <iostream>
#include "logClient.h"
#include "ipcMessages.h"

namespace tarsim {
TimerUtils::TimerUtils()
{
}

TimerUtils::TimerUtils(const std::string &msgQName) :
    m_queName(msgQName)
{
    m_caller = new MsgQClient(m_queName);
    if (m_caller != nullptr && NO_ERR != m_caller->connect())
    {
        LOG_FAILURE("Failed to connect to server(%s)",msgQName.c_str());
    }
}

TimerUtils::~TimerUtils()
{
    if (nullptr != m_caller && m_caller->isConnected())
    {
        m_caller->disconnect();
    }
    delete m_caller;
    m_caller = nullptr;
}


Errors TimerUtils::FireTimer(int32_t timeMs, NotifyMethod notification)
{
    const int oneMilliSeconds = 1000000; // nanoseconds 1e6
    m_cancelTimer = false;
    if (NotifyMethod::NotifyMutex ==  notification)
    {
        std::cout << "Taking Lock\n";
        m_mutex.lock();
    }

    std::thread([timeMs,  notification, this]()
    {
        std::this_thread::sleep_for(std::chrono::nanoseconds(timeMs *oneMilliSeconds ));
        if (NotifyMethod::NotifyMutex ==  notification)
        {
            std::cout << "unLock\n";
            m_mutex.unlock();
        }
        else if (m_cancelTimer)
        {
            return;
        }
        else if (NotifyMethod::NotifyMq ==  notification)
        {
            SimpleMsg_t m;
            m.msgId = MSG_TIMER_EVENT;
            if (nullptr == m_caller)
            {
                LOG_FAILURE("m_caller is null \n");
            }
            else if (NO_ERR != m_caller->isConnected())
            {
                LOG_FAILURE("Server %s is not connected\n",m_queName.c_str());
            }
            else if (m_caller->send(&m, sizeof(m), 0))
            {
                LOG_FAILURE("Failed to send timer message to %s\n",m_queName.c_str());
            }
            else if (0) // for debugging option
            {
                std::cout << "send MSG_TIMER_EVENT messsage to(" << m_queName << ")\n";
            }
        }
    }).detach();

    return NO_ERR;
}
} // end of namespace tarsim
