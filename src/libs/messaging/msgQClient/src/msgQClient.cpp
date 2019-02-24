
//
// @file:
//
// @Created on: Jul 22, 2017
// @Author: Kamran Shamaei
//
//
// @brief - implementation for MsgQClient
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


#include <iostream>
#include <ostream>
#include <string>
#include "msgQClient.h"
#include "logClient.h"
#include "serverDefs.h"
#include <stdio.h>

#include <cstring>

namespace tarsim {

//namespaces

/**
 * @brief initialize private data members for connecting to message queue
 * @param[in] qName- to what message queue to be connected
 */
MsgQClient::MsgQClient(
        const std::string &qName, bool debugOption, unsigned int priority):
        m_qName(qName),
        m_debug(debugOption),
        m_priority(priority)
{
    //cout << "created object MsgQClient for  connecting to Server: "<<  qName << endl;
    m_qId = -1;
    m_attr.mq_curmsgs = 0;
    m_attr.mq_flags = 0;
    m_attr.mq_maxmsg = 0;
    m_attr.mq_msgsize = 0;
    m_printingStdio = m_qName.compare(LogServerThreadName) == 0;
    m_printingStdio = true;// because logger is not shared library - removed cross dependency
}
/**
 * close the connection to the message q server
 */
MsgQClient::~MsgQClient()
{
    //std::cout << "destroyed class CMsgQSender " << m_qName << std::endl;
    if ((mqd_t)-1 != m_qId)
    {
        disconnect();
    }
    m_qId =-1;

}

/**
 * @brief make connect to queue thread
 * @return NO_ERR if connected successfully otherwise returns ERR_MQ_FAILED_OPEN
 */
Errors  MsgQClient::connect()
{
    std::string queueName = "/" + m_qName;
    m_qId = mq_open(queueName.c_str(), O_WRONLY);

    if ((mqd_t)-1 == m_qId)
    {
        if (m_printingStdio)
        {
            printf("Failed to connect to server (%s) message queue (%s) error(%d=%s)\n",
                    m_qName.c_str(), queueName.c_str() , errno, strerror(errno));

        }
        return ERR_MQ_FAILED_OPEN;
    }
    else if (m_debug)
    {
        if (m_printingStdio)
        {
            printf("Connected to channel (%s) message queue (%s)\n",
                    m_qName.c_str(), queueName.c_str());

        }

    }
    return NO_ERR;
}
Errors  MsgQClient::disconnect()
{
	Errors errorCode = NO_ERR;
    if (mq_close(m_qId) == (mqd_t)-1)
    {
        //Generate fault is not necessary, we are disconnecting anyway...
        if (m_printingStdio)
        {
            printf("Failed to close connection to receiver %s message queue\n",
                    m_qName.c_str());
        }
        errorCode = ERR_MQ_FAILED_CLOSE;
    }
    m_qId = (mqd_t)-1;
    return errorCode;

}
/**
 * @brief verify if connection was made successfully
 * @param isConnected - returns true of m_qId is not -1, meanning has valid connection
 * @return NO_ERR
 */
Errors MsgQClient::isConnected() const
{
    if ((mqd_t)-1 == m_qId)
    {
        return ERR_MQ_FAILED_OPEN;
    }
    return NO_ERR;
}

/**
 * @brief send data to the message q. server
 * @param send_data - the buffer data to be sent
 * @param sendDataSize - size of buffer to send
 * @return NO_ERR if data was sent successfylly otherwise returns ERR_MQ_FAILED_SEND
 */
Errors MsgQClient::send(const void* send_data, const size_t sendDataSize,
        unsigned int msgPriority) const
{
    if ((mqd_t)-1 == m_qId)
    {
        if (m_printingStdio)
        {
            printf("Failed to send to receiver %s message queue, receiverMqId is invalid\n",
                    m_qName.c_str());

        }
        return ERR_MQ_FAILED_SEND;

    }
    int result = mq_send(m_qId,(char *)send_data, sendDataSize, m_priority);
    if (result == -1)
    {
        if (m_printingStdio)
        {
            printf("Failed to send to receiver %s message queue, errno(%d)=%s\n",
                    m_qName.c_str(),errno, strerror(errno));
        }
        return ERR_MQ_FAILED_SEND;
    }
    return NO_ERR;

}

Errors MsgQClient::sendMessage(const void* send_data,
                                    const size_t sendDataSize,
                                    const char *fileStr, const int &lineNumber)
{

    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            if (m_printingStdio)
            {
                printf("Failed to connect to server(%s) file(%s) line(%d)",
                        m_qName.c_str(), fileStr, lineNumber);
            }
            return ERR_MQ_FAILED_OPEN;
        }
    }
    if (send(send_data, sendDataSize, m_priority) != NO_ERR)
    {
        if (m_printingStdio)
        {
            printf("Failed to send data to server(%s) file(%s) line(%d)",
                    m_qName.c_str(), fileStr, lineNumber);
        }
        return ERR_MQ_FAILED_SEND;
    }
    return NO_ERR;
}

} // end of namespace tarsim

