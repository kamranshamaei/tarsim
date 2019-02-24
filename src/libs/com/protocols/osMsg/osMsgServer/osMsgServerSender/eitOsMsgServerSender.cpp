/**
 * @file: robotConntrolClient.cpp
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - implementation for RobotJointsSender
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


#include "eitOsMsgServerSender.h"

#include <cstdarg>
#include "simulatorMessages.h"
#include "serverDefs.h"
#include "logClient.h"

namespace tarsim {

/**
 * @brief initialize m_instance
 */

EitOsMsgServerSender::EitOsMsgServerSender(
        std::string &mqName, unsigned int msgPriority):
    MsgQClient(mqName, msgPriority, msgPriority),
    m_msgPriority(msgPriority)
{
}

/**
 * @brief should call destroy to delete m_instance
 */
EitOsMsgServerSender::~EitOsMsgServerSender()
{

}

Errors EitOsMsgServerSender::sendEndEffectorFrame(Frame_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
        	LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = END_EFFECTOR_FRAME;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
    	LOG_FAILURE ("Failed to send data to client");
    	return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

Errors EitOsMsgServerSender::sendRigidBodyFrame(Frame_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = RIGID_BODY_FRAME;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
        LOG_FAILURE ("Failed to send data to client");
        return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

Errors EitOsMsgServerSender::sendObjectFrame(Frame_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = OBJECT_FRAME;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
        LOG_FAILURE ("Failed to send data to client");
        return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

Errors EitOsMsgServerSender::sendErrorMessage(ErrorMessage_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = FAULT_MESSAGE;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
        LOG_FAILURE ("Failed to send data to client");
        return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

Errors EitOsMsgServerSender::sendJointValues(JointPositions_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = ROBOT_JOINT_POSITIONS;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
        LOG_FAILURE ("Failed to send data to client");
        return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

Errors EitOsMsgServerSender::sendSimulatorStatus(SimulatorStatus_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = SIMULATOR_STATUS;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
        LOG_FAILURE ("Failed to send data to client");
        return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

Errors EitOsMsgServerSender::sendIncrementalCommand(
        IncrementalCommandMessage_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = INCREMENTAL_COMMAND;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
        LOG_FAILURE ("Failed to send data to client");
        return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

Errors EitOsMsgServerSender::sendSpeed(SpeedMessage_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = SPEED_COMMAND;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
        LOG_FAILURE ("Failed to send data to client");
        return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

Errors EitOsMsgServerSender::sendCollisions(CollisionMessage_t &msg)
{
    bool connected;
    if (isConnected() != NO_ERR)
    {
        if (connect() != NO_ERR)
        {
            LOG_FAILURE ("Failed to connect to client");
            return Errors::ERR_MQ_FAILED_OPEN;
        }
    }
    msg.msgId = COLLISION;
    msg.srcPid = -1 ; //nothing significant for the receiver to know

    if (send(&msg, sizeof(msg), m_msgPriority) != NO_ERR)
    {
        LOG_FAILURE ("Failed to send data to client");
        return ERR_MQ_FAILED_SEND;
    }

    return NO_ERR;
}

} // end of namespace tarsim




