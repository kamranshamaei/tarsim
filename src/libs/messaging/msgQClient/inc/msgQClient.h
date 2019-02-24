/**
 *
 * @file: msgQClient.h
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

#ifndef SRC_LIBS_MSGQCLIENT_INC_H_
#define SRC_LIBS_MSGQCLIENT_INC_H_


//INCLUDES
#include <mqueue.h>
#include <string>
#include "eitErrors.h"
#include "ipcMessages.h"
namespace tarsim {

class MsgQClient
{

public:
    MsgQClient(const std::string &qName, bool debugOption = false,
            unsigned int priority = DEFAULT_MSG_PRIORITY);
    virtual ~MsgQClient();
    Errors send(const void* send_data, const size_t sendDataSize,
            unsigned int msgPriority) const;
    Errors connect();
    Errors disconnect();
    Errors isConnected() const;
    Errors sendMessage(const void* send_data, const size_t sendDataSize,
                const char *fileStr =__FILE__, const int &lineNumber = __LINE__);

private:
    std::string m_qName = "";
    mqd_t m_qId;
    struct mq_attr m_attr;
    bool m_printingStdio ;
    bool m_debug = false;
    unsigned int m_priority = 0;
};
} // end of namespace tarsim
#endif /* SRC_LIBS_MSGQCLIENT_INC_H_ */
