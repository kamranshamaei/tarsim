/**
 *
 * @file: eitOsMsgClientSender.h
 *
 * @Created on: Jul 22, 2017
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

#ifndef EIT_SENDER_H
#define EIT_SENDER_H

#include "cstdarg"
#include <string>
#include <mutex>
#include "eitErrors.h"
#include "msgQClient.h"
#include "simulatorMessages.h"
#include "serverDefs.h"
namespace tarsim {
class EitOsMsgClientSender : MsgQClient
{
public:
    EitOsMsgClientSender(int32_t index);
    virtual ~EitOsMsgClientSender();

    bool notifyDisconnect(unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool isConnected();

    bool sendJointPositions(
            JointPositions_t &robotPosition,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendJointPosition(
            JointPosition_t &robotPosition,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendBaseFrame(
            Frame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendCamera(
            Camera_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendLockObjectToRigidBody(
            LockObjectToRigidBody_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendUnlockObjectFromRigidBody(
            UnlockObjectFromRigidBody_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendShutdown(unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    bool sendRequestRecord(
            RequestRecord_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    bool sendFaultMessage(
            GuiStatusMessage_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    bool executeForwardKinematics(
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    bool sendRequestEndEffectorFrame(
            RequestEndEffectorFrame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendRequestRigidBodyFrame(
            RequestRigidBodyFrame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendRequestObjectFrame(
            RequestObjectFrame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendObjectFrame(
            Frame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendRequestJointValues(
            RequestJointValues_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendRequestSimulatorStatus(
            SimulatorStatus_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
    bool sendRequestInstallTool(
        RequestInstallTool_t &msg,
        unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    bool sendSetEndEffector(
        SetEndEffector_t &msg,
        unsigned int msgPriority = DEFAULT_MSG_PRIORITY);
protected:

private:
    MsgQClient m_msgSender = MsgQClient(RobotJointsReceiverThreadName);
    int32_t m_index = 0;
};
} // end of namespace tarsim
#endif /* EIT_SENDER_H */
