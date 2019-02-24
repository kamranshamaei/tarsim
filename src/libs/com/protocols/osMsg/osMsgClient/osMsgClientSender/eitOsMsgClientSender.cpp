/*
 * @file: eitOsMsgClientSender.cpp
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - implementation for EitOsMsgClientSender
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


#include "eitOsMsgClientSender.h"
#include <cstdarg>
#include "simulatorMessages.h"
#include "serverDefs.h"
#include "logClient.h"
#include "fileSystem.h"

namespace tarsim {


//globals
EitOsMsgClientSender* EitOsMsgClientSender::m_instance = nullptr;
std::mutex EitOsMsgClientSender::m_mtx;

/**
 * @brief initialize m_instance
 */
EitOsMsgClientSender::EitOsMsgClientSender() :
        MsgQClient(RobotJointsReceiverThreadName)
{
    m_instance = nullptr;
}

/**
 * @brief should call destroy to delete m_instance
 */
EitOsMsgClientSender::~EitOsMsgClientSender()
{

}

void EitOsMsgClientSender::destroy()
{
    delete m_instance;
    m_instance = nullptr;
}

/**
 * @brief provide access to singleton object
 * @return a pointer ConfiguratorClient object
 */
EitOsMsgClientSender* EitOsMsgClientSender::getInstance()
{
    if (!m_instance)
    {
        std::unique_lock<std::mutex> lck(m_mtx);
        m_instance = new EitOsMsgClientSender();
    }
    return m_instance;
}

bool EitOsMsgClientSender::notifyDisconnect(unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    SimpleMsg_t msg;
    msg.msgId = MSG_CLIENT_DISCONNECTED_EVENT;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::isConnected()
{
    if (m_msgSender.isConnected() != NO_ERR)
    {
        if (m_msgSender.connect() != NO_ERR)
        {
            printf ("Failed to connect to RobotServer\n");
            return false;
        }
    }
    return true;
}

bool EitOsMsgClientSender::sendJointPositions(
        JointPositions_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}
    msg.msgId = ROBOT_JOINT_POSITIONS;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
    	printf ("Failed to send data to RobotServer\n");
    	return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendJointPosition(
        JointPosition_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}
    msg.msgId = ROBOT_JOINT_POSITION;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendBaseFrame(
        Frame_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}
    msg.msgId = ROBOT_BASE_POSE;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendCamera(
        Camera_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}
    msg.msgId = CAMERA_DATA;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendLockObjectToRigidBody(
        LockObjectToRigidBody_t &msg, unsigned int msgPriority)
{
    bool connected;
    if (m_msgSender.isConnected() != NO_ERR)
    {
        if (m_msgSender.connect() != NO_ERR)
        {
            printf ("Failed to connect to RobotServer\n");
            return false;
        }
    }
    msg.msgId = LOCK_OBJECT_TO_RIGID_BODY;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendUnlockObjectFromRigidBody(
        UnlockObjectFromRigidBody_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    msg.msgId = UNLOCK_OBJECT_FROM_RIGID_BODY;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::executeForwardKinematics(
        unsigned int msgPriority)
{
    RequestExecuteForwardKinematics_t msg;
    if (!isConnected()) {return false;}
    msg.msgId = REQUEST_EXECUTE_FORWARD_KINEMATICS;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendRequestEndEffectorFrame(
        RequestEndEffectorFrame_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    msg.msgId = REQUEST_END_EFFECTOR_FRAME;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendRequestRigidBodyFrame(
        RequestRigidBodyFrame_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    msg.msgId = REQUEST_RIGID_BODY_FRAME;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendRequestObjectFrame(
        RequestObjectFrame_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    msg.msgId = REQUEST_OBJECT_FRAME;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendObjectFrame(
        Frame_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    msg.msgId = OBJECT_FRAME;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendRequestJointValues(
        RequestJointValues_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    msg.msgId = REQUEST_JOINT_VALUES;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendRequestSimulatorStatus(
        SimulatorStatus_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    msg.msgId = SIMULATOR_STATUS;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendShutdown(unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    RequestShutdown_t msg;
    msg.msgId = SHUTDOWN;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendRequestRecord(
        RequestRecord_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    if (msg.isStart) {
        msg.msgId = REQUEST_START_RECORD;
    } else {
        msg.msgId = REQUEST_STOP_RECORD;
    }

    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

bool EitOsMsgClientSender::sendFaultMessage(
        GuiStatusMessage_t &msg, unsigned int msgPriority)
{
    if (!isConnected()) {return false;}

    msg.msgId = GUI_STATUS_MESSAGE;
    msg.srcPid = FileSystem::getPid();

    if (m_msgSender.send(&msg, sizeof(msg), msgPriority) != NO_ERR)
    {
        printf ("Failed to send data to RobotServer\n");
        return false;
    }

    return true;
}

} // end of namespace tarsim
