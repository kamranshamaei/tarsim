/**
 * @file: eitOsMsgClientReceiver.cpp
 *
 * @Created on: May 05, 2018
 * @Author: Kamran Shamaei
 *
 *
 * @brief - Receives reply from the simulator package.
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


#include "eitOsMsgClientReceiver.h"

#include <iostream>
#include <cstring>
#include "serverDefs.h"
#include "ipcMessages.h"
#include "eitErrors.h"
#include "logClient.h"

namespace tarsim {
/**
 * @brief constructor for the EitOsMsgClientReceiver
 */
EitOsMsgClientReceiver::EitOsMsgClientReceiver(
        const std::string &mqName, int policy, int priority):
        MsgQServer(mqName, policy, priority)
{
    ErrorMessage_t msg;
    msg.msgCounter = -1;
    msg.errorId = NO_ERR;
    memset (msg.errorMsg, ' ', sizeof(char) * LOG_MAX_DATA_SIZE);
    msg.errorMsg[0] = 0;
    msg.errorMsg[LOG_MAX_DATA_SIZE - 1] = 0;
    msg.msgLength = 0;
    setErrorMessage(msg);
}

/**
 * @brief destructor for the EitOsMsgClientReceiver
 */
EitOsMsgClientReceiver::~EitOsMsgClientReceiver()
{

}

/**
 * @brief wakup EitOsMsgClientReceiver with a timer
 */
Errors EitOsMsgClientReceiver::start()
{
    MsgQServer::start();
    return NO_ERR;
}

/**
 * @process the incoming data to the EitOsMsgClientReceiver Server
 * supported message id:
 * @param[in] inComingData -
 */
void EitOsMsgClientReceiver::onMessage(const GenericData_t &inComingData)
{
    switch (inComingData.simpleMsg.msgId)
    {
        case END_EFFECTOR_FRAME:
        {
            Frame_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setEndEffectorFrame(in);
        }
        break;

        case OBJECT_FRAME:
        {
            Frame_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setObjectFrame(in);
        }
        break;

        case FAULT_MESSAGE:
        {
            ErrorMessage_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setErrorMessage(in);
        }
        break;

        case RIGID_BODY_FRAME:
        {
            Frame_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setRigidBodyFrame(in);
        }
        break;

        case ROBOT_JOINT_POSITIONS:
        {
            JointPositions_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setJointValues(in);
        }
        break;

        case SIMULATOR_STATUS:
        {
            SimulatorStatus_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setIsSimulatorRunning(in.isSimRunning);
        }
        break;

        case INCREMENTAL_COMMAND:
        {
            IncrementalCommandMessage_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setIncrementalCommand(in.type, in.index, in.incCmd);
        }
        break;

        case SPEED_COMMAND:
        {
            SpeedMessage_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setSpeed(in.speed);
        }
        break;

        case COLLISION:
        {
            CollisionMessage_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            setCollisions(in);
        }
        break;

        default:
            break;
    }
}

void EitOsMsgClientReceiver::setEndEffectorFrame(const Frame_t &msg)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_frameEndEffector = msg;
}


void EitOsMsgClientReceiver::setRigidBodyFrame(const Frame_t &msg)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_frameRigidBody = msg;
}

void EitOsMsgClientReceiver::setJointValues(const JointPositions_t &msg)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_jointPositions = msg;
}

void EitOsMsgClientReceiver::setObjectFrame(const Frame_t &msg)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_frameObject = msg;
}

void EitOsMsgClientReceiver::setErrorMessage(const ErrorMessage_t &msg)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_faultMessage = msg;
}

Frame_t EitOsMsgClientReceiver::getEndEffectorFrame()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    Frame_t msg = m_frameEndEffector;
    return msg;
}


Frame_t EitOsMsgClientReceiver::getRigidBodyFrame()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    Frame_t msg = m_frameRigidBody;
    return msg;
}

Frame_t EitOsMsgClientReceiver::getObjectFrame()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    Frame_t msg = m_frameObject;
    return msg;
}

ErrorMessage_t EitOsMsgClientReceiver::getErrorMessage()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    ErrorMessage_t msg = m_faultMessage;
    return msg;
}

JointPositions_t EitOsMsgClientReceiver::getJointValues()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    JointPositions_t msg = m_jointPositions;
    return msg;
}

bool EitOsMsgClientReceiver::getIsSimulatorRunning()
{
    std::unique_lock<std::mutex> lock(m_mutexIsSimRunning);
    bool isSimRunning = m_isSimRunning;
    return isSimRunning;
}

void EitOsMsgClientReceiver::setIsSimulatorRunning(bool isSimRunning)
{
    std::unique_lock<std::mutex> lock(m_mutexIsSimRunning);
    m_isSimRunning = isSimRunning;
}

void EitOsMsgClientReceiver::getIncrementalCommand(
        int32_t &type, int32_t &index, int32_t &incCmd)
{
    std::unique_lock<std::mutex> lock(m_mutexIncCmd);
    incCmd = m_incCmd;
    index = m_incCmdIndex;
    type = (int32_t)m_incCmdType;
}

void EitOsMsgClientReceiver::setIncrementalCommand(
        IncrementalCommandTypes type, int32_t index, int32_t incCmd)
{
    std::unique_lock<std::mutex> lock(m_mutexIncCmd);
    m_incCmd = incCmd;
    m_incCmdType = type;
    m_incCmdIndex = index;
}

void EitOsMsgClientReceiver::setSpeed(float speed)
{
    std::unique_lock<std::mutex> lock(m_mutexSpeed);
    m_speed = speed;
}

float EitOsMsgClientReceiver::getSpeed()
{
    std::unique_lock<std::mutex> lock(m_mutexSpeed);
    float speed = m_speed;
    return speed;
}

std::vector<std::pair<int32_t, int32_t>> EitOsMsgClientReceiver::getSelfCollisions()
{
    std::unique_lock<std::mutex> lock(m_mutexCollisions);
    std::vector<std::pair<int32_t, int32_t>> selfCollisions = m_selfCollisions;
    return selfCollisions;
}

std::vector<std::pair<int32_t, int32_t>> EitOsMsgClientReceiver::getExternalCollisions()
{
    std::unique_lock<std::mutex> lock(m_mutexCollisions);
    std::vector<std::pair<int32_t, int32_t>> externalCollisions = m_externalCollisions;
    return externalCollisions;
}

void EitOsMsgClientReceiver::setCollisions(const CollisionMessage_t &msg)
{
    std::unique_lock<std::mutex> lock(m_mutexCollisions);
    m_selfCollisions.clear();
    m_externalCollisions.clear();
    for (int32_t i = 0; i < msg.numCollisions; i++) {
        for (int32_t j = 0; j < msg.collisions[i].numCollisions; j++) {
            if (msg.collisions[i].isSelfCollision[j]) {
                m_selfCollisions.push_back(std::make_pair(
                        msg.collisions[i].robotLink, msg.collisions[i].rigidBody[j]));
            } else {
                m_externalCollisions.push_back(std::make_pair(
                        msg.collisions[i].robotLink, msg.collisions[i].rigidBody[j]));
            }

        }
    }
}

} // end of namespace tarsim





