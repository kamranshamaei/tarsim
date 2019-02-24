/**
 *
 * @file: eitOsMsgServerReceiver.h
 *
 * @Created on: May 05, 2018
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


#ifndef SRC_LIBS_EitOsMsgServerSender_H

#define SRC_LIBS_EitOsMsgServerSender_H

#include "cstdarg"
#include <string>
#include <mutex>
#include "eitErrors.h"
#include "msgQClient.h"
#include "simulatorMessages.h"
#include "serverDefs.h"

namespace tarsim {

//const & defines


//Macros used for application 

//class definition

class EitOsMsgServerSender : public MsgQClient
{

public:
    Errors sendEndEffectorFrame(Frame_t &msg);
    Errors sendRigidBodyFrame(Frame_t &msg);
    Errors sendObjectFrame(Frame_t &msg);
    Errors sendErrorMessage(ErrorMessage_t &msg);
    Errors sendJointValues(JointPositions_t &msg);
    Errors sendSimulatorStatus(SimulatorStatus_t &msg);
    Errors sendIncrementalCommand(IncrementalCommandMessage_t &msg);
    Errors sendSpeed(SpeedMessage_t &msg);
    Errors sendCollisions(CollisionMessage_t &msg);

    virtual ~EitOsMsgServerSender();
    EitOsMsgServerSender(
            std::string &mqName, unsigned int msgPriority = DEFAULT_MSG_PRIORITY);


protected:

private:
    int32_t qPid = -1; //initialize process id of recevier to -1
    unsigned int m_msgPriority = 0;
};
} // end of namespace tarsim
#endif /* SRC_LIBS_EitOsMsgServerSender_H */
