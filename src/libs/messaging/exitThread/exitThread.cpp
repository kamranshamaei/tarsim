
//
// @file: robotConntrolClient.cpp
//
// @Created on: Jul 22, 2017
// @Author: Kamran Shamaei
//
//
// @brief - implementation for RobotJointsSender
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


#include "exitThread.h"
#include <cstdarg>
#include "serverDefs.h"
#include "ipcMessages.h"
namespace tarsim {

//namespace
using namespace std;


//globals

/**
 * @brief initialize m_instance
 */
ExitThread::ExitThread(const std::string &threadName) : MsgQClient(threadName)
{

}

/**
 * @brief should
 */
ExitThread::~ExitThread()
{

}


Errors ExitThread::sendExit()
{
    bool connected;
    SimpleMsg_t msg;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = MSG_EXIT_EVENT;

    if (send(&msg, sizeof(msg), 0) != NO_ERR)
    {
    	return ERR_MQ_FAILED_SEND;
    }
    //disconnect(); // the queue is no longer available - closing connection is not meaningfull.
    return NO_ERR;
}

} // end of namespace tarsim
