/**
 *
 * @file: msgQServer.h
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief -
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

#ifndef SRC_LIBS_MSGQSERVER_INC_H_
#define SRC_LIBS_MSGQSERVER_INC_H_

//INCLUDES
#include <mqueue.h>
#include <pthread.h>
#include "ipcMessages.h"
#include <string>
#include "eitErrors.h"
#include <thread>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

namespace tarsim {
//namespaces--------------------------------------------------------------------
//consts------------------------------------------------------------------------
//enumeration-------------------------------------------------------------------

class MsgQServer
{
//public------------------------------------------------------------------------
public:

    MsgQServer(
            const std::string &qName,
            int policy = DEFAULT_RT_THREAD_POLICY,
            int priority = DEFAULT_RT_THREAD_PRIORITY);
    virtual ~MsgQServer();
    virtual Errors start();
    Errors stop();
    virtual void allowedToExit() { m_allowedToForceExit = true;};

//protected---------------------------------------------------------------------
protected:

    virtual void onMessage(const GenericData_t &inComingData);
    virtual void onExit();

//private-----------------------------------------------------------------------
private:
    Errors cleanup();
    Errors running();
    void endAndExit();
    static void* threadFunctionHelper(void* void_ptr_to_this_threaded_object);
    std::string         m_qName;        // Queue Name
    mqd_t               m_qId;          // Q id
    struct mq_attr      m_attr;         // Attributes of the Q
    bool                m_runforEver ;  // Stay in the thread loop for ever

    std::mutex          m_mtx;          // used for thread synchronization.
    std::condition_variable m_cv;       // used for thread synchronization.
    bool                m_ready;        // used for thread synchronization.
    bool                m_printingStdio;// printing to stdio
    bool                m_allowedToForceExit = false;;// User has requested to shut down the application.

    std::unique_ptr<pthread_t> m_pthread;

    int m_threadPolicy = 0;
    int m_threadPriority = 0;

};
} // end of namespace tarsim
#endif /* SRC_LIBS_MSGQSERVER_INC_H_ */
