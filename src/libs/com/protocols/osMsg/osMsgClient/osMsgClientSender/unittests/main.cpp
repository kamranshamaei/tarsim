
/*
 * @file: main.cpp
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - Test program for robotClient
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright Kamran Shamaei
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 */


//INCLUDES

#include <iostream>

#include "eitOsMsgClientSender.h"
#include "threadUtilities.h"
#include "logClient.h"
#include "threadUtilities.h"

/**
 * @brief creates configurator clients, sends some messages and then exits the
 * application
 * @param argc - number of arguments
 * @param argv - list of arguments
 * @return EXIT_SUCCESS
 */
int main(int argc, char **argv)
{
    EitOsMsgClientSender *eitSender = EitOsMsgClientSender::getInstance();
    JointPositions_t pos;
    LOG_INFO("EitSender client test program started");
    const int numJoints = 2;

    int counter = 0;
    pos.numJoints = numJoints;
    while (true) {
        for (int i = 0; i < numJoints; i++)
        {
            pos.indices[i] = i;
            pos.positions[i] = (double)counter/10.0;
        }
//        eitSender->sendJointsPosition(pos);
        ThreadUtilities::waitforMilliseconds(100);
        counter++;
    }

    delete eitSender;
    eitSender = nullptr;

    return EXIT_SUCCESS;
}



