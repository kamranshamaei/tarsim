/**
 * @file: eitServer.cpp
 *
 * @Created on: March 31, 2018
 * @Author: Kamran Shamaei
 *
 *
 * @brief -
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright [2017-2018] Kamran Shamaei .
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 */

//INCLUDES
#include "eitServer.h"
namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
EitServer::EitServer(ConfigParser* cp, Kinematics* kin, Gui* gui,
        int policy, int priority, unsigned int msgPriority)
{
    m_eitOsMsgServerReceiver = new EitOsMsgServerReceiver(
            cp, kin, gui, policy, priority, msgPriority);
    m_eitOsMsgServerReceiver->start();
}

EitServer::~EitServer()
{
    delete m_eitOsMsgServerReceiver;
    m_eitOsMsgServerReceiver = nullptr;
}

EitOsMsgServerReceiver* EitServer::getEitOsMsgServerReceiver()
{
    return m_eitOsMsgServerReceiver;
}

} // end of namespace tarsim



