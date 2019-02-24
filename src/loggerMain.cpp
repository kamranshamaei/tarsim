/**
 *
 * @file:  loggerMain.cpp
 *
 * @Created on: Dec 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - create a logger program

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


//INCLUDES

#include <iostream>
#include "threadUtilities.h"
#include "logClient.h"
#include "threadUtilities.h"
#include "logServer.h"

/**
 * @brief creates the log server and remains blocked for ever.
 * @param argc - number of arguments
 * @param argv - list of arguments
 * @return EXIT_SUCCESS
 */
int main(int argc, char **argv)
{
    tarsim::LogServer *logServerThread = new tarsim::LogServer();
    logServerThread->start();
    LOG_INFO("Logger main started");
    tarsim::ThreadUtilities::blockForever();
    return EXIT_SUCCESS;
}



