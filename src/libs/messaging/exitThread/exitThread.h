/**
 *
 * @file: exitThread.h
 *
 * @Created on: May 10, 2018
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


#ifndef SRC_LIBS_EXIT_THREAD_H

#define SRC_LIBS_EXIT_THREAD_H

#include "cstdarg"
#include <string>
#include "eitErrors.h"
#include "msgQClient.h"
#include "serverDefs.h"

namespace tarsim {

//const & defines


//Macros used for application 
//class definition

class ExitThread : MsgQClient
{

public:
    ExitThread(const std::string &serverName);
    Errors sendExit();
    virtual ~ExitThread();

protected:

    // delete copy and move constructors and assign operators
    ExitThread(ExitThread const&) = delete;             // Copy construct
    ExitThread(ExitThread&&) = delete;                  // Move construct
    ExitThread& operator=(ExitThread const&) = delete;  // Copy assign
    ExitThread& operator=(ExitThread &&) = delete;      // Move assign
private:

};
} // end of namespace tarsim
#endif /* SRC_LIBS_EXIT_THREAD_H */
