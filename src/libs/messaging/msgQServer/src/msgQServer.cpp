
//
// @file: msgQServer.cpp
//
// @Created on: Jul 22, 2017
// @Author: Kamran Shamaei
//
//
// @brief -
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


//INCLUDES
#include "msgQServer.h"
#include <pthread.h>
#include "logClient.h"
#include <errno.h>
#include <cstring>
#include <iostream>


namespace tarsim {
//namespaces--------------------------------------------------------------------
//consts------------------------------------------------------------------------
//eumeration--------------------------------------------------------------------


/**
 * @brief initialize message queue data members
 * @param qName - the thread name
 * @param[in] priority - set the thread priority
 */
MsgQServer::MsgQServer(
    const std::string &qName, int policy, int priority) :
        m_qName(qName),
        m_threadPolicy(policy),
        m_threadPriority(priority)
{
    m_qId = -1;
    m_attr.mq_curmsgs = 0;
    m_attr.mq_flags = 0;
    m_attr.mq_maxmsg = MAX_QUEUE_SIZE ;
    m_attr.mq_msgsize = MAX_MSG_SIZE;
    m_runforEver = true;
    m_ready = false;
    m_printingStdio = m_qName.compare(LogServerThreadName) == 0;
    m_printingStdio = true;
}

/**
 * @brief spawns the thread and waits for the thread to start before returning
 * @return NO_ERR
 */
Errors MsgQServer::start()
{
    std::unique_lock<std::mutex> lck(m_mtx);

    if (nullptr != m_pthread.get()) {
      printf("Failed: Thread already exists\n");
      return ERR_INVALID;
    }
    m_pthread = std::unique_ptr<pthread_t>(new pthread_t);

    int ret = 0;
    /* Initialize pthread attributes (default values) */
    pthread_attr_t attr;
    ret = pthread_attr_init(&attr);
    if (ret) {
      printf("Failed to initialize pthread attributes (error = %d)\n", ret);
      return ERR_INVALID;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, m_threadPolicy);
    if (ret) {
      printf("Failed to set scheduling policy of pthread (error = %d)\n", ret);
      return ERR_INVALID;
    }

    struct sched_param param;
    param.sched_priority = m_threadPriority;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
      printf("Failed to set scheduling parameters of pthread %d\n", ret);
      return ERR_INVALID;
    }

    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
      printf("Failed to set inherit scheduling of pthread (error = %d)\n", ret);
      return ERR_INVALID;
    }

    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    /* Create a pthread with specified attributes */
    ret = pthread_create(
        m_pthread.get(), &attr, MsgQServer::threadFunctionHelper, (void*)this);
    if (ret) {
        pthread_attr_setschedpolicy(&attr, 0);
        param.sched_priority = 0;
        pthread_attr_setschedparam(&attr, &param);
        ret = pthread_create(
                m_pthread.get(), &attr, MsgQServer::threadFunctionHelper, (void*)this);

        if (ret) {
            printf("Failed to create non-realtime thread %s (error = %d)\n",
                m_qName.c_str(), ret);
            return ERR_INVALID;
        }

        printf("Create non-realtime thread %s\n", m_qName.c_str());
    }

    // Set the thread name
    pthread_setname_np(*m_pthread.get(), m_qName.c_str());

    pthread_attr_destroy(&attr);

    // Wait for thread to come up
    //LOG_FAILURE("Wait %s\n",m_qName.c_str());
    while (!m_ready)
    {
        //LOG_FAILURE("looping\n");
        m_cv.wait(lck);
    }
    LOG_INFO("Started: %s\n",m_qName.c_str());
    return Errors::NO_ERR;
}


void* MsgQServer::threadFunctionHelper(
    void* void_ptr_to_this_threaded_object)
{
  MsgQServer* ptr_to_this_threaded_object =
      static_cast<MsgQServer*>(void_ptr_to_this_threaded_object);
  Errors err = ptr_to_this_threaded_object->running();
  if (NO_ERR != err) {
    printf("Tarsim server failed...\n");
  }
  return nullptr;
}

/**
 * @brief - core routine which creates and the message queue and waits for incoming
 * messages
 * @return NO_ERR - if the thread started successfully.
 * @returns ERR_FAILED_SPAWNED - if it fails to create the message queue
 */
Errors MsgQServer::running()
{
    /* receive the message */
    GenericData_t data;// buffer;
    std::string serverName = m_qName;
    m_qName ="/" + m_qName;
    mq_unlink(m_qName.c_str()); // Remove if already exists in /dev/mq
    m_qId = mq_open(m_qName.c_str(), O_CREAT | O_RDONLY, 0644, &m_attr);
    if (-1 == m_qId)
    {
        if (m_printingStdio)
        {
            LOG_FAILURE("Failed to create queue (%s) %d %s \n",m_qName.c_str(), errno,
                    strerror(errno));
        }

        return ERR_FAILED_SPAWNED;

    }

    //Ready to go
    std::unique_lock<std::mutex> lck(m_mtx);
    m_ready = true;
    lck.unlock();
    m_cv.notify_one();

    while(m_runforEver)
    {
        ssize_t bytesRead;
        bytesRead = mq_receive(m_qId, (char *)&data, MAX_MSG_SIZE, NULL);
        if (bytesRead <= 0)
        {
            if (m_printingStdio)
            {
                LOG_INFO("qName(%s) bytesRead is less than 0. bytesRead(%d)\n",
                        m_qName.c_str(), int(bytesRead));

            }
        }
        else
        {
        	m_runforEver = (MSG_EXIT_EVENT != data.simpleMsg.msgId);
            if (!m_runforEver )
            {
            	onExit(); //tell the child class to clean up.
            	cleanup(); // clean MsgQServer Objects;
            }
            else
            {
            	onMessage(data);
            }
        }

    }
    return Errors::NO_ERR;
}

void MsgQServer::onExit()
{
	//allow cleanup for the child if needed
}
/**
 * @brief - does not thing. child class performs the task
 * @param inComingData
 */

void MsgQServer::onMessage(const GenericData_t &inComingData)
{
}


/**
 * @brief       Close the message queue
 * @return      NO_ERR: if closed successfully;
 *              false: otherwise
 */
Errors MsgQServer::cleanup()
{
	Errors errCode = Errors::NO_ERR;
    if ((mqd_t) -1 != m_qId )
    {
        int status;
        status = mq_close(m_qId);
        m_qId = (mqd_t) - 1;
        if( -1 == status )
        {
            if (m_printingStdio)
            {
                LOG_FAILURE("Failed to close the message queue %s, errno (%d)=(%s)\n",
                         m_qName.c_str(), errno, strerror(errno));
            }
        	errCode = ERR_MQ_FAILED_CLOSE;
        }
    }
	mq_unlink(m_qName.c_str()); // Remove if already exists in /dev/mqueue
	if (m_allowedToForceExit)
	{
		exit(EXIT_SUCCESS);
	}
    return errCode;
}


MsgQServer::~MsgQServer()
{
	endAndExit();
}
void MsgQServer::endAndExit()
{
  m_runforEver = false;
  printf("Closing server %s\n", m_qName.c_str());

  if (Errors::NO_ERR != stop()) {
    printf("Failed to properly close server\n");
  }

}



/**
 * @brief it is intended to stop the server
 * @returns NO_ERR
 *
 */
Errors MsgQServer::stop()
{
    mqd_t qId;
    struct mq_attr attr;

    if ((mqd_t)-1 == m_qId)
    {
        return Errors::NO_ERR;
    }
    //Initialize
    qId = -1;
    attr.mq_curmsgs = 0;
    attr.mq_flags = 0;
    attr.mq_maxmsg = 0;
    attr.mq_msgsize = 0;
    //Make Connection
    qId = mq_open(m_qName.c_str(), O_WRONLY);
    m_printingStdio = true;
    if ((mqd_t)-1 == m_qId)
    {
        if (m_printingStdio)
        {
            LOG_FAILURE("Failed to connect to message queue (%s) error(%d=%s)\n",
                    m_qName.c_str(),  errno, strerror(errno));

        }
        return ERR_MQ_FAILED_OPEN;
    }

    SimpleMsg_t data;
    data.msgId = MSG_EXIT_EVENT;
    int result = mq_send(qId,(char *)&data, sizeof(SimpleMsg_t), 0);
    if (result == -1)
    {
        if (m_printingStdio)
        {
            LOG_FAILURE("Failed to send to receiver %s message queue, errno(%d)=%s\n",
                    m_qName.c_str(),errno, strerror(errno));
        }
        return ERR_MQ_FAILED_SEND;
    }

    mq_close(qId);
    // Wait for thread to go away
    m_runforEver = false;
    int ret = pthread_join(*m_pthread.get(), nullptr);
    if (ret) {
      printf("Failed to join cthread %s -> %m (error = %d) for thread %s",
          m_qName.c_str(), ret, m_qName.c_str());
      return ERR_INVALID;
    }
    return Errors::NO_ERR;
}
} // end of namespace tarsim
