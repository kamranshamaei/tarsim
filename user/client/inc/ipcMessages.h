/**
 *
 * @file:IpcMessages.h
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - provides class definition for posix message Q thread.
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

#ifndef SRC_LIBS_INC_IPCMESSAGES_H_
#define SRC_LIBS_INC_IPCMESSAGES_H_

#include <time.h>
#include "cstdint"

namespace tarsim {
static const int32_t MAX_QUEUE_SIZE = 10; // maximum of items in the queue at one time
static const int32_t MAX_MSG_SIZE = 1024;  // maximum of size data being exchanged between client & server
static const size_t MAX_STATUS_TEXT_SIZE = 200; // maximum length of status message text size
static const int32_t LOG_MAX_DATA_SIZE = 256; // maximum of data record for a log message
static const unsigned int DEFAULT_MSG_PRIORITY = 10; // Default message priority
static const int DEFAULT_RT_THREAD_POLICY = SCHED_FIFO; // Default message priority
static const int DEFAULT_RT_THREAD_PRIORITY = 80; // Default message priority


struct MessageHeader_t
{
	int32_t srcPid;         // From what process the message was initiated.
	int32_t msgId;          // Every message will have a message id
	int32_t msgCounter;     // Every message will have a message id
};

// Log  Related data
struct LogData_t : MessageHeader_t
{
    int32_t logType; 					//for future expansion
    timespec threadtime;				// Recorded time when logClient is called
    char data[LOG_MAX_DATA_SIZE];	    // data containing the log information
};
struct SimpleMsg_t : MessageHeader_t
{
// Simple message doesn't have a body by design, only inherits Message Header
};

struct BlobOfData_t : MessageHeader_t
{
    uint8_t data[MAX_MSG_SIZE - sizeof(MessageHeader_t)];
};

//Union of all data structure
union GenericData_t
{                               
    LogData_t 	logData;        //LOG_CNTL_MSG
    SimpleMsg_t simpleMsg;      //Simple message only for msg id
    BlobOfData_t blobOfData;
};

// msg Q releated send/receive/creation errors
enum MessageId_t
{
    MSG_LOG_CMD,
    MSG_TIMER_EVENT,
    MSG_EXIT_EVENT,
    MSG_CLIENT_DISCONNECTED_EVENT,
    MSG_LAST_GENERIC,
    MSG_FIRST_APPLICATION = MSG_LAST_GENERIC
};
} // end of namespace tarsim
#endif /* SRC_LIBS_INC_IPCMESSAGES_H_ */
