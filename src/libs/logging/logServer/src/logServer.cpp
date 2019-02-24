
//
// @file: LogServer.cpp
//
// @Created on: Jul 22, 2017
// @Author: Kamran Shamaei
//
//
// @brief - This file contains class definition for LogServer .
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


#include "logServer.h"
#include "serverDefs.h"
#include "ipcMessages.h"
#include <iostream>
#include "eitErrors.h"
#include <stdio.h>

namespace tarsim {

using namespace std;

const long c_maxSize = 10000000000;//10 Gig
/**
 * @brief constructor for the log server
 */
LogServer::LogServer() :
    MsgQServer(LogServerThreadName, SCHED_OTHER, 0)
{
    filename = "/tmp/eitLog.txt";
	//open with append mode if possible
	outf.open(filename, std::fstream::in | std::fstream::out | std::fstream::trunc);
	// If file does not exist, Create new file
	if (!outf )
	{
		cout << "Failed to create new file " << filename;
	}
}

/**
 * @brief destructor for the log server
 */
LogServer::~LogServer()
{
    onExit();
}

void LogServer::onExit()
{
    outf.close();
}
/**
 * @process the incoming data to the log server
 * supported message id MSG_LOG_CNTL
 * message is already formatted by logClient object,
 * this server outputs to the console data &time of the record log message
 * @param[in] inComingData , casted to LogData
 */
void LogServer::onMessage(const GenericData_t &inComingData)
{
    if (MSG_LOG_CMD == inComingData.logData.msgId)
    {
        char tsBuf[64], tmbuf[64];
        struct tm* tsTm;
        std::string outputFormat   = "%Y-%m-%d %H:%M:%S";


        tsTm = localtime(&inComingData.logData.threadtime.tv_sec);  // convert calendar time to local time
        strftime(tmbuf, sizeof tmbuf, outputFormat.c_str(), tsTm);
        snprintf(tsBuf, sizeof tsBuf, "%s:%03ld", tmbuf, inComingData.logData.threadtime.tv_nsec/1000000);


//        cout << '[' << tsBuf << "] " ;
//        cout << inComingData.logData.data  << endl <<  std::flush;
        outf << '[' << tsBuf << "] " ;
        outf << inComingData.logData.data  << endl <<  std::flush;
        long fsize = outf.tellg();
        if (fsize > c_maxSize)
        {
           outf.close();
           std::string fileNameFormat   = "%Y-%m-%d-%H-%M-%S ";
           time_t rawtime;
           struct tm * timeinfo;
           time (&rawtime);
           timeinfo = localtime (&rawtime);
           strftime(tmbuf, sizeof tmbuf, fileNameFormat.c_str(), timeinfo);
           printf("File size is larger %ld %s %s\n", fsize,tmbuf,  (tmbuf+filename).c_str() );
           int result= rename( filename.c_str() , (tmbuf+filename).c_str() );
           if ( 0 == result )
           {
               std::cout << "Renamed successfully\n";
           }
           else
           {
               perror( "Error renaming file" );
           }
           outf.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
        }
    }
    else
    {
        printf( "Unexpected message of logServer %d", inComingData.logData.msgId);
    }
}
} // end of namespace tarsim

