/**
 *
 * @file: node.h
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
 *
 */


// IFNDEF
#ifndef EIT_SERVER_H
#define EIT_SERVER_H

//INCLUDES
#include "eitOsMsgServerReceiver.h"
#include "eitOsMsgServerSender.h"

namespace tarsim {
// FORWARD DECLARATIONS

// TYPEDEFS AND DEFINES


// ENUMS


// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class EitServer
{
public:
    // FUNCTIONS
    EitServer(ConfigParser* cp, Kinematics* kin, Gui* gui,
            int policy, int priority, unsigned int msgPriority);
    virtual ~EitServer();
    EitOsMsgServerReceiver* getEitOsMsgServerReceiver();

    // MEMBERS

private:
    // FUNCTIONS
    // MEMBERS
//#if (EIT_COM_TYPE == "OsMsg")
    EitOsMsgServerReceiver* m_eitOsMsgServerReceiver = nullptr;
//#endif
};
} // end of namespace tarsim
// ENDIF
#endif /* EIT_SERVER_H */
