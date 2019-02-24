/*
 * @file: eitErrosr.h
 *
 * @Created on: Apr 5, 2017
 * @Author: Kamran Shamaei
 *
 * @brief - This file contains definition of all error codes

 * @copyright Copyright Kamran Shamaei 
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 ****************************************************************************/

#ifndef SRC_LIBS_INC_EITERRORSS_H_
#define SRC_LIBS_INC_EITERRORSS_H_

namespace tarsim {

typedef enum Errors
{
    NO_ERR = 0,
    ERR_INVALID,

    // Errors setting and getting member variables
    ERR_SET,
    ERR_GET,

    // msg Q releated send/receive/creation errors
    ERR_MQ_FAILED_CREATION,
    ERR_MQ_FAILED_SEND,
    ERR_MQ_FAILED_RECEIVE,
    ERR_MQ_FAILED_OPEN,
    ERR_MQ_FAILED_CLOSE,

    // Simulator Errors (TODO: Move to simApp)
    ERR_JOINT_POSITION_LIMIT,
    ERR_JOINT_VELOCITY_LIMIT,
    ERR_JOINT_ACCELERATION_LIMIT,

    // Thread Failures
    ERR_FAILED_SPAWNED,
    ERR_UNDEFINED,         // generic error
    ERR_LAST_DEFINED       // Last defined error, application can defined

} Errors;
} // end of namespace tarsim
#endif /* SRC_LIBS_INC_EITERRORSS_H_ */
