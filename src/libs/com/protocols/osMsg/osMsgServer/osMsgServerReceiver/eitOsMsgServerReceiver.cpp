/*
 * @file: eitOsMsgServerReceiver.cpp
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


#include "eitOsMsgServerReceiver.h"

#include <iostream>
#include <cstring>
#include "serverDefs.h"
#include "ipcMessages.h"
#include "eitErrors.h"
#include "logClient.h"
#include "simulatorMessages.h"
#include "kinematics.h"
#include "gui.h"
#include "configParser.h"
#include <algorithm>
#include "fileSystem.h"
#include "exitThread.h"
#include <algorithm>

using namespace std;

namespace tarsim {
/**
 * @brief constructor for the EitOsMsgServerReceiver
 */
EitOsMsgServerReceiver::EitOsMsgServerReceiver(ConfigParser* cp, Kinematics* kin,
        Gui* gui, int policy, int priority, unsigned int msgPriority):
        MsgQServer(RobotJointsReceiverThreadName, policy, priority)
{
    m_kinematics = kin;
    m_gui = gui;
    m_cp = cp;
    m_msgPriority = msgPriority;
}

/**
 * @brief destructor for the EitOsMsgServerReceiver
 */
EitOsMsgServerReceiver::~EitOsMsgServerReceiver()
{
    delete m_runTimer;
    m_runTimer = nullptr;
    EitOsMsgServerSender *sendUserReply;
    for (auto i : m_listofUsers)
    {
        SimulatorStatus_t sts;
        sts.msgId = SIMULATOR_STATUS;
        sts.isSimRunning = false;
        sendUserReply = i.second;
        if (sendUserReply->send(&sts, sizeof(sts), m_msgPriority) != NO_ERR)
        {
        	printf("Message queue %d not available\n", i.first);
        }

        SimpleMsg_t out;
        out.msgId = MSG_EXIT_EVENT;
        sendUserReply = i.second;
        if (sendUserReply->send(&out, sizeof(out), m_msgPriority) != NO_ERR)
        {
        	printf("Message queue %d not available\n", i.first);
        }
	}
}

/**
 * @brief wakup EitOsMsgServerReceiver with a timer
 */
Errors EitOsMsgServerReceiver::start()
{
    MsgQServer::start();
    return NO_ERR;
}

EitOsMsgServerSender *EitOsMsgServerReceiver::getUserConnection(
        const int32_t userPid )
{
	if (userPid <= 0)
	{
		return nullptr;
	}
	if (m_listofUsers.find(userPid) == m_listofUsers.end())
	{
		std::string mqName = FileSystem::getMQNamePid(userPid);
		m_listofUsers[userPid] = new EitOsMsgServerSender(mqName, m_msgPriority);
	}
    return m_listofUsers.at(userPid);
}
/**
 * @process the incoming data to the EitOsMsgServerReceiver Server
 * supported message id:
 * @param[in] inComingData -
 */
void EitOsMsgServerReceiver::onMessage(const GenericData_t &inComingData)
{
	int32_t userPid = inComingData.simpleMsg.srcPid;

	EitOsMsgServerSender *sendUserReply = getUserConnection(userPid);

	m_msgCounter = inComingData.simpleMsg.msgCounter;

	switch (inComingData.simpleMsg.msgId)
    {
        case MSG_CLIENT_DISCONNECTED_EVENT:
        {
          m_listofUsers.erase(inComingData.simpleMsg.srcPid);
          if (sendUserReply != nullptr)
          {
            sendUserReply->disconnect();
          }

          break;
        }
        case ROBOT_JOINT_POSITIONS:
        {
            JointPositions_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            updateRobotJointPositions(sendUserReply, in);
        }
        break;

        case ROBOT_JOINT_POSITION:
        {
            JointPosition_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            updateRobotJointPosition(sendUserReply, in);
        }
        break;

        case ROBOT_BASE_POSE:
        {
            Frame_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            Matrix4d m = Matrix4d::Identity();
            m << in.mij[ 0], in.mij[ 1], in.mij[ 2], in.mij[ 3],
                 in.mij[ 4], in.mij[ 5], in.mij[ 6], in.mij[ 7],
                 in.mij[ 8], in.mij[ 9], in.mij[10], in.mij[11],
                 in.mij[12], in.mij[13], in.mij[14], in.mij[15];

            m_kinematics->getRoot()->setXfm(m);
        }
        break;

        case CAMERA_DATA:
        {
            Camera_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            m_kinematics->getCameraDataQueue()->push(in);

        }
        break;

        case LOCK_OBJECT_TO_RIGID_BODY:
        {
            LockObjectToRigidBody_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            m_kinematics->lockObjectsToRb(in.indexObject, in.indexRigidBody);

        }
        break;

        case UNLOCK_OBJECT_FROM_RIGID_BODY:
        {
            UnlockObjectFromRigidBody_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            m_kinematics->unlockObjectsFromRigidBody(in.indexObject);

        }
        break;

        case REQUEST_EXECUTE_FORWARD_KINEMATICS:
        {
            GuiStatusMessage_t in;
            std::map<int32_t, Collision> collisions;
            if (NO_ERR != m_kinematics->executeForwardKinematics(in, collisions))
            {
                LOG_WARNING("Failed to execute forward kinematics");
            }

            if (in.faultLevel > FaultLevels::FAULT_LEVEL_NOFAULT) {
                m_gui->setStatusMessage(in);
            }

            sendCollision(collisions);
        }
        break;

        case REQUEST_END_EFFECTOR_FRAME:
        {
            Matrix4d xfm = m_kinematics->getXfmEndEffector();
            Frame_t out;
            out.frameId = 0;
            out.msgCounter = m_msgCounter;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    int index = (i * 4) + j;
                    out.mij[index] = xfm(i, j);
                }
            }

            if (sendUserReply != nullptr)
            {
                sendUserReply->sendEndEffectorFrame(out);
            }
        }
        break;

        case REQUEST_RIGID_BODY_FRAME:
        {
            RequestRigidBodyFrame_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            Matrix4d xfm = Matrix4d::Zero();
            if (NO_ERR != m_kinematics->getRigidBodyFrame(
                    (int)in.indexRigidBody, (int)in.indexFrame, xfm)) {
                LOG_WARNING("Failed to get rigid body frame");
            }

            Frame_t out;
            out.frameId = in.indexRigidBody;
            out.msgCounter = m_msgCounter;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    int index = (i * 4) + j;
                    out.mij[index] = xfm(i, j);
                }
            }

            if (sendUserReply != nullptr)
            {
                sendUserReply->sendRigidBodyFrame(out);
            }

        }
        break;

        case REQUEST_OBJECT_FRAME:
        {
            RequestObjectFrame_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));
            Matrix4d xfm = Matrix4d::Zero();
            if (NO_ERR != m_kinematics->getObjectFrame((int)in.indexObject, xfm)) {
                LOG_WARNING("Failed to get object frame");
            }

            Frame_t out;
            out.frameId = in.indexObject;
            out.msgCounter = m_msgCounter;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    int index = (i * 4) + j;
                    out.mij[index] = xfm(i, j);
                }
            }

            if (sendUserReply != nullptr)
            {
                sendUserReply->sendObjectFrame(out);
            }

        }
        break;

        case OBJECT_FRAME:
        {
            Frame_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));

            Matrix4d xfm = Matrix4d::Zero();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    int index = (i * 4) + j;
                    xfm(i, j) = in.mij[index];
                }
            }

            if (NO_ERR != m_kinematics->setObjectFrame((int)in.frameId, xfm)) {
                LOG_WARNING("Failed to get object frame");
            }
        }
        break;

        case REQUEST_JOINT_VALUES:
        {
            std::map<int, double> jointValues;
            if (NO_ERR != m_kinematics->getJointValues(jointValues)) {
                LOG_WARNING("Failed to get joint values from kinematics");
            }

            JointPositions_t out;
            out.msgCounter = m_msgCounter;
            out.numJoints = (int32_t)jointValues.size();
            int index = 0;
            for (std::map<int, double>::iterator it = jointValues.begin();
                    it != jointValues.end(); ++it) {
                out.indices[index] = it->first;
                out.positions[index] = it->second;
                index++;
            }


            if (sendUserReply != nullptr)
            {
                sendUserReply->sendJointValues(out);
            }

        }
        break;

        case SIMULATOR_STATUS:
        {
            SimulatorStatus_t out;
            out.msgCounter = m_msgCounter;
            out.isSimRunning = true;
            if (sendUserReply != nullptr)
            {
                sendUserReply->sendSimulatorStatus(out);
            }
        }
        break;

        case REQUEST_START_RECORD:
        {
            m_gui->setRecordRobotScene(true);
        }
        break;

        case REQUEST_STOP_RECORD:
        {
            m_gui->setRecordRobotScene(false);
        }
        break;

        case GUI_STATUS_MESSAGE:
        {
            GuiStatusMessage_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));

            m_gui->setStatusMessage(in);
        }
        break;

        case SHUTDOWN:
        {
            m_listofUsers.erase(inComingData.simpleMsg.srcPid);
            m_gui->destroy();
        }
        break;

        case INSTALL_TOOL:
        {
            RequestInstallTool_t in;
            std::memcpy(&in, &inComingData.blobOfData, sizeof(in));

            installTool(in);
        }
        break;

        case MSG_TIMER_EVENT:
            LOG_INFO("Timer Event in RobotServer.....");
            break;

        default:
            break;
    }
}

void EitOsMsgServerReceiver::updateRobotJointPositions(
        EitOsMsgServerSender *sendUserReply, const JointPositions_t& pos)
{
    Errors maxError = NO_ERR;
    int32_t jntIndex = 0;
    for (int32_t i = 0; i < pos.numJoints; i++) {
        Node* node = m_cp->getNodeOfMate((int)pos.indices[i]);

        if (nullptr == node) {
            LOG_FAILURE("Invalid joint index %d was received", pos.indices[i]);
            return;
        }

        Errors error = node->setTargetJointValue(pos.positions[i]);

        if (error > maxError) {
            maxError = error;
            jntIndex = pos.indices[i];
        }
    }

    // Send error back
    if (sendUserReply != nullptr)
    {
        ErrorMessage_t out;
        out.errorId = (int32_t)maxError;
        out.msgCounter = pos.msgCounter;
        memset(out.errorMsg, ' ', sizeof(char) * LOG_MAX_DATA_SIZE);
        std::string message = "No faults";
        if (ERR_JOINT_POSITION_LIMIT == maxError) {
            message = "Fault: Position limit for joint #" + std::to_string(jntIndex);
        } else if (ERR_JOINT_VELOCITY_LIMIT == maxError) {
            message = "Fault: Velocity limit for joint #" + std::to_string(jntIndex);
        } else if (ERR_JOINT_ACCELERATION_LIMIT == maxError) {
            message = "Fault: Acceleration limit for joint #" + std::to_string(jntIndex);
        }
        auto copy_len = std::min((int)message.size(), LOG_MAX_DATA_SIZE - 1);
        out.errorMsg[copy_len] = 0;
        out.errorMsg[LOG_MAX_DATA_SIZE - 1] = 0;
        out.msgLength = copy_len;
        std::copy(
                message.begin(),
                message.begin() + copy_len,
                out.errorMsg);
        sendUserReply->sendErrorMessage(out);
    }
}


void EitOsMsgServerReceiver::updateRobotJointPosition(
        EitOsMsgServerSender *sendUserReply, const JointPosition_t& pos)
{
    Node* node = m_cp->getNodeOfMate((int)pos.index);

    if (nullptr == node) {
        LOG_FAILURE("Invalid joint index %d was received", pos.index);
        return;
    }

    Errors error = node->setTargetJointValue(pos.position);
    m_msgCounter = pos.msgCounter;

    // Send error back
    if (sendUserReply != nullptr)
    {
        ErrorMessage_t out;
        out.errorId = (int32_t)error;
        out.msgCounter = pos.msgCounter;
        memset(out.errorMsg, ' ', sizeof(char) * LOG_MAX_DATA_SIZE);
        std::string message = "No faults";
        if (ERR_JOINT_POSITION_LIMIT == error) {
            message = "Fault: Position limit for joint #" + std::to_string(pos.index);
        } else if (ERR_JOINT_VELOCITY_LIMIT == error) {
            message = "Fault: Velocity limit for joint #" + std::to_string(pos.index);
        } else if (ERR_JOINT_ACCELERATION_LIMIT == error) {
            message = "Fault: Acceleration limit for joint #" + std::to_string(pos.index);
        }
        auto copy_len = std::min((int)message.size(), LOG_MAX_DATA_SIZE - 1);
        out.errorMsg[copy_len] = 0;
        out.errorMsg[LOG_MAX_DATA_SIZE - 1] = 0;
        out.msgLength = copy_len;
        std::copy(
                message.begin(),
                message.begin() + copy_len,
                out.errorMsg);
        sendUserReply->sendErrorMessage(out);
    }
}

Errors EitOsMsgServerReceiver::sendIncrementalCommand(int32_t incCmd)
{
    IncrementalCommandMessage_t msg;
    msg.incCmd = incCmd;
    msg.type = INC_CMD_TYPE_CARTESIAN;
    for (auto pair: m_listofUsers) {
        if (NO_ERR != pair.second->sendIncrementalCommand(msg)) {
            LOG_FAILURE("Failed to send incremental command to process %d", (int)pair.first);
            return ERR_INVALID;
        }
    }
    return NO_ERR;
}

Errors EitOsMsgServerReceiver::sendSpeed(float speed)
{
    SpeedMessage_t msg;
    msg.speed = speed;
    for (auto pair: m_listofUsers) {
        if (NO_ERR != pair.second->sendSpeed(msg)) {
            LOG_FAILURE("Failed to send speed rate to process %d", (int)pair.first);
            return ERR_INVALID;
        }
    }
    return NO_ERR;
}

Errors EitOsMsgServerReceiver::sendJointIncrementalCommand(
        int32_t jntIndex, int32_t incCmd)
{
    IncrementalCommandMessage_t msg;
    msg.incCmd = incCmd;
    msg.type = INC_CMD_TYPE_JOINT;
    msg.index = jntIndex;
    for (auto pair: m_listofUsers) {
        if (NO_ERR != pair.second->sendIncrementalCommand(msg)) {
            LOG_FAILURE("Failed to send incremental command to process %d", (int)pair.first);
            return ERR_INVALID;
        }
    }
    return NO_ERR;
}

Errors EitOsMsgServerReceiver::sendCollision(
        const std::map<int32_t, Collision> &collisions)
{
    CollisionMessage_t msg;
    int32_t numCollisions = std::min(MAX_COLLISIONS, (int32_t)collisions.size());
    msg.numCollisions = numCollisions;

    int index = 0;
    for (auto pair: collisions) {
        memcpy(&msg.collisions[index], &pair.second, sizeof(pair.second));
        index++;
        if (index == MAX_COLLISIONS) {
            break;
        }
    }

    for (auto pair: m_listofUsers) {
        if (NO_ERR != pair.second->sendCollisions(msg)) {
            LOG_FAILURE("Failed to send collision to process %d", (int)pair.first);
            return ERR_INVALID;
        }
    }
    return NO_ERR;
}

void EitOsMsgServerReceiver::installTool(RequestInstallTool_t &msg)
{
    if (m_gui->removeTool()) {
        LOG_FAILURE("Failed to remove tool");
        return;
    }

    std::string toolName = std::string(msg.toolName);
    if (NO_ERR != m_cp->loadTool(toolName)) {
        LOG_FAILURE("Failed to load tool %s", toolName.c_str());
        return;
    }

    if (NO_ERR != m_kinematics->installTool()) {
        LOG_FAILURE("Failed to install tool in kinematics");
        return;
    }

    GuiStatusMessage_t in;
    std::map<int32_t, Collision> collisions;
    if (NO_ERR != m_kinematics->executeForwardKinematics(in, collisions))
    {
        LOG_WARNING("Failed to execute forward kinematics");
    }

    if (NO_ERR != m_gui->installTool(m_cp->getTool())) {
        LOG_FAILURE("Failed to install tool in gui");
        return;
    }

    m_kinematics->incCounter();
}

} // end of namespace tarsim
