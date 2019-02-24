/**
 *
 * @file:   logClient.h
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
#ifndef SRC_LIBS_LOGCLIENT_INC_LOGCLIENT_H_
#define SRC_LIBS_LOGCLIENT_INC_LOGCLIENT_H_

#include "cstdarg"
#include <string>
#include "eitErrors.h"
#include <mqueue.h>
#include <pthread.h>
#include <thread>
#include <mutex>
#include "msgQClient.h"
#include "serverDefs.h"

namespace tarsim {
//const & defines


//Macros used for application to log messages
#define CALLER_INFO __FILE__, __FUNCTION__,__LINE__

//logging informative messages
#define LOG_INFO(...) tarsim::LogClient::getInstance()->log (tarsim::LogClient::INFO, CALLER_INFO, __VA_ARGS__)
//logging warning messages but system is operational
#define LOG_WARNING(...) tarsim::LogClient::getInstance()->log (tarsim::LogClient::WARNING, CALLER_INFO, __VA_ARGS__)
//logging catastrophic messages , system can't proceed to contiue
#define LOG_FAILURE(...) tarsim::LogClient::getInstance()->log (tarsim::LogClient::FAILURE, CALLER_INFO, __VA_ARGS__)


//
class LogClient
{
public:
    class connectToLogServer
    {
    public:
            connectToLogServer();
            virtual ~connectToLogServer();
            Errors send(const void* send_data, const size_t sendDataSize) const;
            Errors connect();
            Errors disconnect();
            Errors isConnected(bool &isConnected) const;
    private:
            mqd_t m_qId;
            struct mq_attr m_attr;
            std::string m_qName;

    };
    enum LogLevels
    {
        FAILURE,      ///< Failure that could be a critical condition,
                      ///< alert, or emergency. An example is a fault. Requires
                      ///< resolution.
        WARNING,      ///< Warning message, not a failure, but indication that
                      ///< a failure will occur if action is not taken.
        NOTICE,       ///< Normal but significant condition. Event that is
                      ///< unusual but not a failure condition. No immediate
                      ///< action required. Condition is typically temporary,
                      ///< such as subsystems being out of state.
        INFO,         ///< Informational message. Normal operational message. No
                      ///< action required.
        DEBUG         ///< Info useful to developers for debugging the
                      ///< application, not useful during operations.
    };
    void log(LogLevels logLevel,const char *fileNanme,const char *functionName, int lineNumber,const char *fmt,... );

public:
    static LogClient * getInstance();
    void destroy();

protected:

    LogClient();
    ~LogClient();
    // delete copy and move constructors and assign operators
    LogClient(LogClient const&) = delete;             // Copy construct
    LogClient(LogClient&&) = delete;                  // Move construct
    LogClient& operator=(LogClient const&) = delete;  // Copy assign
    LogClient& operator=(LogClient &&) = delete;      // Move assign
private:
    static LogClient *m_instance ;
    std::string convertLogLevelString(LogLevels logLevel);
    MsgQClient          m_msgSender = MsgQClient(LogServerThreadName);
    static std::mutex   m_mtx;          // used for thread synchronization.



};
} // end of namespace tarsim
#endif /* SRC_LIBS_LOGCLIENT_INC_LOGCLIENT_H_ */
