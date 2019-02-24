/*
 * ServerDefs.h
 *
 *  Created on: Jul 22, 2017
 *      Author: Kamran Shamaei
 *
 * @brief - This file contains definition of all server names
 *
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright [2017-2017] Kamran Shamaei .
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 *
 */


#ifndef SRC_LIBS_INC_SERVERDEFS_H_
#define SRC_LIBS_INC_SERVERDEFS_H_

#include "string"

namespace tarsim {
/**
//name of message queue, which is also name of thread name
//message queue name is made up of "/" + thread anem
//thread name is define below
#define XYStageServer  	"XYStageServer"     // the thread name of moving XY stage
#define LogServerQueue  "LogServerQueue"    // the thread name for logging message

*/
//name of message queue, which is also name of thread name
//message queue name is made up of "/" + thread name
//thread name is define below
const std::string ConfiguratorThreadName =            "ConfigServer";              // the thread name of Configurator
const std::string LogServerThreadName =               "TarsimLogServer";           // the thread name for logging message
const std::string AdmittanceServerThreadName =        "AdmittanceServer";          // admittance server
const std::string SafetyMonitorServerThreadName =     "SafetyMonServer";           // Safety Monitor
const std::string EtherCATServerThreadName =          "EtherCATManagerServer";     // Managing Ethercat communication
const std::string EtherCATDataServerThreadName =      "EtherCATDataServer";        // Receiving EtherCAT Data
const std::string RecoveryServerThreadName =          "RecoveryServer";            // Recovery Server Thread
const std::string DDSServerThreadName =               "DDSServer";                 // DDS Server
const std::string RobotJointsReceiverThreadName =     "TarsimRobotServer";         // Robot COntrol
const std::string UserAppThreadName =                 "UserAppSrvr";               // UserApp Server
}; // end of namespace tarsim

#endif /* SRC_LIBS_INC_SERVERDEFS_H_ */
