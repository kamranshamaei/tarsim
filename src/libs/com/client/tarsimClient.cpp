/**
 * @file: tarsimClient.cpp
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
#include "eitOsMsgClientSender.h"
#include "eitOsMsgClientReceiver.h"
#include "tarsimClient.h"
#include <signal.h>
#include "fileSystem.h"
#include <chrono>
#include <algorithm>

namespace tarsim {

// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
TarsimClient::TarsimClient(int policy, int priority)
{
    std::string userAppReplyMsgQName = FileSystem::getMQNamePid();
    mq_unlink(("/" + userAppReplyMsgQName).c_str());
    m_eitOsMsgClientReceiver =
            new EitOsMsgClientReceiver(userAppReplyMsgQName, policy, priority);
    m_eitOsMsgClientSender = EitOsMsgClientSender::getInstance();
}

TarsimClient::~TarsimClient()
{
    if (!close()) {
        printf("Failed to properly close TarsimClient\n");
    }
}

bool TarsimClient::connect(unsigned int msgPriority)
{
    if (NO_ERR != m_eitOsMsgClientReceiver->start()) {
        printf("Failed to connect to tarsim\n");
        return false;
    }

    SimulatorStatus_t out;
    out.msgCounter = getMsgStamp();
    if (!m_eitOsMsgClientSender->sendRequestSimulatorStatus(out)) {
        printf("Failed to get tarsim status\n");
        return false;
    }

    return true;
}

bool TarsimClient::close()
{
    printf("Closing TarsimClient...\n");

    printf("Disconnecting from server...\n");
    if (!m_eitOsMsgClientSender->notifyDisconnect()) {
        printf("Failed to notify disconnect\n");
        return false;
    }

    printf("Closing receiver...\n");
    delete m_eitOsMsgClientReceiver;
    m_eitOsMsgClientReceiver = nullptr;

    printf("Closing sender...\n");
    m_eitOsMsgClientSender->destroy();

    printf("Successfully closed TarsimClient\n");
    return true;
}

bool TarsimClient::shutdown(unsigned int msgPriority)
{
    if (!m_eitOsMsgClientSender->notifyDisconnect()) {
        printf("Failed to notify disconnect\n");
        return false;
    }

    usleep(1000);

    return m_eitOsMsgClientSender->sendShutdown();
}

bool TarsimClient::sendJointPositions(
        JointPositions_t &msg, unsigned int msgPriority)
{
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendJointPositions(msg);
}

bool TarsimClient::sendJointPosition(
        JointPosition_t &msg, unsigned int msgPriority)
{
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendJointPosition(msg);
}

bool TarsimClient::sendBaseFrame(Frame_t &msg, unsigned int msgPriority)
{
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendBaseFrame(msg);
}

bool TarsimClient::sendCamera(Camera_t &msg, unsigned int msgPriority)
{
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendCamera(msg);
}

bool TarsimClient::lockObjectToRigidBody(
        int indexObject, int indexRigidBody, unsigned int msgPriority)
{
    LockObjectToRigidBody_t msg;
    msg.indexObject = (int32_t)indexObject;
    msg.indexRigidBody = (int32_t)indexRigidBody;
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendLockObjectToRigidBody(msg);
}

bool TarsimClient::unlockObjectFromRigidBody(
        int indexObject, unsigned int msgPriority)
{
    UnlockObjectFromRigidBody_t msg;
    msg.indexObject = (int32_t)indexObject;
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendUnlockObjectFromRigidBody(msg);
}

bool TarsimClient::executeForwardKinematics(unsigned int msgPriority)
{
    return m_eitOsMsgClientSender->executeForwardKinematics();
}

bool TarsimClient::sendRequestEndEffectorFrame(
        RequestEndEffectorFrame_t &msg, unsigned int msgPriority)
{
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendRequestEndEffectorFrame(msg);
}

bool TarsimClient::getEndEffectorFrame(
        Frame_t &msg, int timeout_period_us, unsigned int msgPriority)
{
    RequestEndEffectorFrame_t out;
    out.msgCounter = getMsgStamp();
    if (!m_eitOsMsgClientSender->sendRequestEndEffectorFrame(out)) {
        printf("Failed to send request to get joint values\n");
        return false;
    }

    int counter = 0;
    // Wait here until the message
    while (true) {
        msg = m_eitOsMsgClientReceiver->getEndEffectorFrame();
        if (msg.msgCounter == out.msgCounter) {
            break;
        }

        if (10 * counter > timeout_period_us) {
            printf("Failed to get end-effector frame in time\n");
            return false;
        }
        usleep(k_sleepTimeUs);
        counter++;
    }
    return true;
}

bool TarsimClient::sendRequestRigidBodyFrame(
        RequestRigidBodyFrame_t &msg, unsigned int msgPriority)
{
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendRequestRigidBodyFrame(msg);
}

bool TarsimClient::getRigidBodyFrame(
        int32_t indexRigidBody, int32_t indexFrame,
        Frame_t &msg, int timeout_period_us, unsigned int msgPriority)
{
    RequestRigidBodyFrame_t out;
    out.msgCounter = getMsgStamp();
    out.indexRigidBody = indexRigidBody;
    out.indexFrame = indexFrame;
    if (!m_eitOsMsgClientSender->sendRequestRigidBodyFrame(out)) {
        printf("Failed to send request to get joint values\n");
        return false;
    }

    int counter = 0;
    // Wait here until the message
    while (true) {
        msg = m_eitOsMsgClientReceiver->getRigidBodyFrame();
        if (msg.msgCounter == out.msgCounter) {
            break;
        }

        if (10 * counter > timeout_period_us) {
            printf("Failed to get rigid body frame in time\n");
            return false;
        }
        usleep(k_sleepTimeUs);
        counter++;
    }
    return true;
}

bool TarsimClient::sendRequestObjectFrame(
        RequestObjectFrame_t &msg, unsigned int msgPriority)
{
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendRequestObjectFrame(msg);
}

bool TarsimClient::getObjectFrame(
    int32_t indexObject, Frame_t &msg, int timeout_period_us,
    unsigned int msgPriority)
{
    RequestObjectFrame_t out;
    out.indexObject = indexObject;
    out.msgCounter = getMsgStamp();
    if (!m_eitOsMsgClientSender->sendRequestObjectFrame(out)) {
        printf("Failed to send request to get joint values\n");
        return false;
    }

    int counter = 0;
    // Wait here until the message
    while (true) {
        msg = m_eitOsMsgClientReceiver->getObjectFrame();
        if (msg.msgCounter == out.msgCounter) {
            break;
        }

        if (10 * counter > timeout_period_us) {
            printf("Failed to get object frame in time\n");
            return false;
        }
        usleep(k_sleepTimeUs);
        counter++;
    }
    return true;
}

bool TarsimClient::setObjectFrame(
        int32_t indexObject, Frame_t &msg, unsigned int msgPriority)
{
    msg.frameId = indexObject;
    msg.msgCounter = getMsgStamp();
    if (!m_eitOsMsgClientSender->sendObjectFrame(msg)) {
        printf("Failed to send object frame\n");
        return false;
    }
    return true;
}

bool TarsimClient::sendRequestJointValues(
        RequestJointValues_t &msg,unsigned int msgPriority)
{
    msg.msgCounter = getMsgStamp();
    return m_eitOsMsgClientSender->sendRequestJointValues(msg);
}

bool TarsimClient::getJointValues(
        JointPositions_t &msg, int timeout_period_us, unsigned int msgPriority)
{
    RequestJointValues_t out;
    out.msgCounter = getMsgStamp();
    if (!m_eitOsMsgClientSender->sendRequestJointValues(out)) {
        printf("Failed to send request to get joint values\n");
        return false;
    }

    int counter = 0;
    // Wait here until the message
    while (true) {
        msg = m_eitOsMsgClientReceiver->getJointValues();
        if (msg.msgCounter == out.msgCounter) {
            break;
        }

        if (10 * counter > timeout_period_us) {
            printf("Failed to get joint values in time\n");
            return false;
        }
        usleep(k_sleepTimeUs);
        counter++;
    }
    return true;
}

ErrorMessage_t TarsimClient::getErrorMessage(unsigned int msgPriority)
{
    return m_eitOsMsgClientReceiver->getErrorMessage();
}

int32_t TarsimClient::getMsgStamp()
{
    return ++m_counter;
}

bool TarsimClient::isSimulatorRunning(unsigned int msgPriority)
{
    SimulatorStatus_t out;
    out.msgCounter = getMsgStamp();
    if (!m_eitOsMsgClientSender->sendRequestSimulatorStatus(out)) {
        printf("Failed to request tarsim status\n");
        return false;
    }
    return m_eitOsMsgClientReceiver->getIsSimulatorRunning();
}

bool TarsimClient::startRecording(unsigned int msgPriority)
{
    RequestRecord_t out;
    out.msgCounter = getMsgStamp();
    out.isStart = true;
    if (!m_eitOsMsgClientSender->sendRequestRecord(out)) {
        printf("Failed to request tarsim status\n");
        return false;
    }
    return true;
}

bool TarsimClient::stopRecording(unsigned int msgPriority)
{
    RequestRecord_t out;
    out.msgCounter = getMsgStamp();
    out.isStart = false;
    if (!m_eitOsMsgClientSender->sendRequestRecord(out)) {
        printf("Failed to request tarsim status\n");
        return false;
    }
    return true;
}

bool TarsimClient::displayMessage(
        const char* faultMessage,
        FaultLevels faultLevel,
        unsigned int msgPriority)
{
    GuiStatusMessage_t out;
    out.msgCounter = getMsgStamp();
    out.faultLevel = faultLevel;
    out.faultType = FAULT_TYPE_CLIENT;

    strncpy(out.statusMessage, faultMessage,
            std::min(sizeof(out.statusMessage), MAX_STATUS_TEXT_SIZE));
    out.statusMessage[MAX_STATUS_TEXT_SIZE - 1] = '\0';

    if (!m_eitOsMsgClientSender->sendFaultMessage(out)) {
        printf("Failed to send fault message\n");
        return false;
    }
    return true;
}

void TarsimClient::getIncrementalCommand(
        int32_t &type, int32_t &index, int32_t &incCmd)
{
    m_eitOsMsgClientReceiver->getIncrementalCommand(type, index, incCmd);
}

float TarsimClient::getSpeed()
{
    return m_eitOsMsgClientReceiver->getSpeed();
}

std::vector<std::pair<int32_t, int32_t>> TarsimClient::getSelfCollisions()
{
    return m_eitOsMsgClientReceiver->getSelfCollisions();
}

std::vector<std::pair<int32_t, int32_t>> TarsimClient::getExternalCollisions()
{
    return m_eitOsMsgClientReceiver->getExternalCollisions();
}


} // end of namespace tarsim
