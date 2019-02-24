/**
 *
 * @file: LogServer.h
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - This file contains class definition for LogServer .
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

#ifndef SRC_LIBS_LOGSERVER_INC_LOGSERVER_H_
#define SRC_LIBS_LOGSERVER_INC_LOGSERVER_H_

#include <fstream>
#include <iostream>
#include "msgQServer.h"

namespace tarsim {

using std::fstream;


class LogServer : public MsgQServer
{
private:
    std::string filename = "";
    fstream outf;

public:
	LogServer();
	virtual ~LogServer();

private:
	virtual void onMessage(const GenericData_t &inComingData);
	virtual void onExit();
};
} // end of namespace tarsim
#endif /* SRC_LIBS_LOGSERVER_INC_LOGSERVER_H_ */
