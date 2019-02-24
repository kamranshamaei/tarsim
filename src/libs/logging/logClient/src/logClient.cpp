
//
// @file:
//
// @Created on: Jul 22, 2017
// @Author: Kamran Shamaei
//
//
// @brief -  This file contains implementation for LogClient
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


#include "logClient.h"
#include <cstdarg>
#include "ipcMessages.h"
#include "serverDefs.h"
#include "ctime"
#include "eitErrors.h"
#include <stdio.h>
#include <iostream>
#include <string.h>
namespace tarsim {

//namespace
using namespace std;


//globals
LogClient* LogClient::m_instance = 0;
std::mutex LogClient::m_mtx;


/**
 * @brief initialize m_instance
 */
LogClient::LogClient()
{
    m_instance = nullptr;
}

/**
 * @brief should call destroy to delete m_instance
 */
LogClient::~LogClient()
{

}

void LogClient::destroy()
{
    delete m_instance;
    m_instance = nullptr;
}

/**
 * @brief provide access to singleton object
 * @return a pointer logClient object
 */
LogClient* LogClient::getInstance()
{
    if (!m_instance)
    {
        std::unique_lock<std::mutex> lck(m_mtx);
        m_instance = new LogClient();
    }
    return m_instance;
}

/**
 *
 * @param[in] logLevel - tyep of log message (info, warning, failure)
 * @param[in] fileName - caller source code file name
 * @param[in] functionName caller source code function name
 * @param[in] lineNumber - caller source code line number
 * @param[in] format - formatted string as supported in printf function
 */
void LogClient::log(LogLevels logLevel,const char *fileName,const char *functionName, int lineNumber,const char *format,... )
{

    const int c_lookupChar = '/';

    char header[LOG_MAX_DATA_SIZE];
    char buffer[LOG_MAX_DATA_SIZE];
    LogData_t msg;
    struct timespec currentTime;

    msg.msgId = MSG_LOG_CMD;
    clock_gettime(CLOCK_REALTIME, &msg.threadtime);

    va_list argptr;
    va_start(argptr, format);
    vsnprintf(buffer, LOG_MAX_DATA_SIZE - 1, format, argptr);
    va_end(argptr);


    const char *fileNamewithNoPath = strrchr(fileName,c_lookupChar)+1;//Do Reverse Lookup.
    int result = snprintf(header,LOG_MAX_DATA_SIZE/2,"(%s:%s:%d)",fileNamewithNoPath,functionName,lineNumber);
    if (result >= (LOG_MAX_DATA_SIZE/2) || result < 0) // Give some space for logLevel
    {
        std::cout << "***************** BUFFER OVER FLOW " << header << std::endl;
        std::cout << fileName << " " << functionName << " " << lineNumber ;
        return;
    }
    std::string levelString = convertLogLevelString(logLevel);

    // determine the length to copy, it will be the min of MAX_STRING_SIZE-1
    // or test.size.  This will ensure that even if the string it shorter than
    // the MAX_STRING_SIZE, that it will copy properly -- without reading past the
    // array bounds of the string.
    auto copy_len = std::min((int) levelString.size(), LOG_MAX_DATA_SIZE - 1);
    std::copy(
            levelString.begin(),
            levelString.begin() + copy_len,
            msg.data);
    msg.data[copy_len] =0;
    strncat(msg.data,header,LOG_MAX_DATA_SIZE/2);
    strncat(msg.data, buffer,LOG_MAX_DATA_SIZE-1);
    msg.data[LOG_MAX_DATA_SIZE - 1] = 0; // ensure null termination at the end
    if (strlen(msg.data) >= LOG_MAX_DATA_SIZE)
    {
        std::cout << "***************** BUFFER OVER FLOW " << buffer << std::endl;
        std::cout << fileName << " " << functionName << " " << lineNumber ;
        return;
    }

    bool connected;
    if (m_msgSender.isConnected() != NO_ERR)
    {
        if (m_msgSender.connect() != NO_ERR)
        {
            std::cout << "Failed to connect to logger" << endl;
            return ;
        }
    }
    if (m_msgSender.send(&msg, sizeof(msg), 0) != NO_ERR)
    {
        std::cout << "Failed to send data" << msg.data << endl;
    }
}

/**
 * @brief get string for log message level
 * @param[in] log message level
 * @param[out] log message level string
 */
std::string LogClient::convertLogLevelString(LogLevels logLevel)
{
    std::string newString;
    switch (logLevel)
    {
    case FAILURE:
        newString = "FAILURE";
        break;

    case DEBUG:
        newString = "DEBUG";
        break;

    case INFO:
        newString = "INFO";
        break;

    case WARNING:
        newString = "WARNING";
        break;

    case NOTICE:
        newString = "NOTICE";
        break;

    default:
        newString = "??? ";
        break;
    }
    return newString;
}

} // end of namespace tarsim


