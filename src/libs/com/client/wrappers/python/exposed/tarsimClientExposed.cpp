/**
 * @file: eitSimExposed.cpp
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
#include <cstdio>
#include <stdexcept>
#include "tarsimClientExposed.h"

namespace tarsim {

// Global singleton used to keep internal state
TarsimClientExposed* g_tarsimClientExposed = nullptr;


// Exposed C Interface ---------------------------------------------------------
extern "C" {

bool initialize()
{
    try {
        g_tarsimClientExposed = new TarsimClientExposed();
    } catch (const std::invalid_argument& e) {
        printf("Connection timeout.\n");
        return false;
    } catch(...) {
        printf("Unknown fault occurred\n");
        return false;
    }

    return true;
}

bool stop()
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->close();
}

bool connect()
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->connect();
}

bool shutdown()
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->shutdown();
}

bool isSimulatorRunning()
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->isSimulatorRunning();
}

bool sendJointPositions(JointPositions_t &msg)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->sendJointPositions(msg);
}

bool sendJointPosition(JointPosition_t &msg)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->sendJointPosition(msg);
}

bool sendBaseFrame(Frame_t &msg)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->sendBaseFrame(msg);
}

bool sendCamera(Camera_t &msg)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->sendCamera(msg);
}

bool lockObjectToRigidBody(
        int indexObject,
        int indexRigidBody)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->lockObjectToRigidBody(indexObject, indexRigidBody);
}

bool unlockObjectFromRigidBody(int indexObject)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->unlockObjectFromRigidBody(indexObject);
}

bool executeForwardKinematics()
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->executeForwardKinematics();
}

bool getEndEffectorFrame(Frame_t &msg)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->getEndEffectorFrame(msg);
}

bool getRigidBodyFrame(int32_t indexRigidBody, int32_t indexFrame, Frame_t &msg)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->getRigidBodyFrame(indexRigidBody, indexFrame, msg);
}

bool getObjectFrame(int32_t indexObject, Frame_t &msg)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->getObjectFrame(indexObject, msg);
}

bool setObjectFrame(int32_t indexObject, Frame_t &msg)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->setObjectFrame(indexObject, msg);
}

bool getJointValues(JointPositions_t &msg, int timeout_period_us = 100000)
{
    if (g_tarsimClientExposed == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return g_tarsimClientExposed->getJointValues(msg, timeout_period_us);
}

ErrorMessage_t getErrorMessage()
{
    return g_tarsimClientExposed->getErrorMessage();
}


bool displayMessage(
		const char* message,
		int level)
{
	return g_tarsimClientExposed->displayMessage(message, (FaultLevels)level);
}

void getIncrementalCommand(
		int32_t &type, int32_t &index, int32_t &incCmd)
{
	return g_tarsimClientExposed->getIncrementalCommand(type, index, incCmd);
}

float getSpeed()
{
	return g_tarsimClientExposed->getSpeed();
}

std::vector<std::pair<int32_t, int32_t>> getSelfCollisions()
{
	return g_tarsimClientExposed->getSelfCollisions();
}

std::vector<std::pair<int32_t, int32_t>> getExternalCollisions()
{
	return g_tarsimClientExposed->getExternalCollisions();
}


} // extern "C" ----------------------------------------------------------------

TarsimClientExposed::TarsimClientExposed()
{
    try {
        delete m_tarsimClient;
        m_tarsimClient = nullptr;
        m_tarsimClient = new TarsimClient();
    } catch (const std::invalid_argument& e) {
        delete m_tarsimClient;
        m_tarsimClient = nullptr;
        throw std::invalid_argument("Failed initiate connection");
    } catch(...) {
        delete m_tarsimClient;
        m_tarsimClient = nullptr;
        throw std::invalid_argument("Unknown fault occurred");
    }
}

TarsimClientExposed::~TarsimClientExposed()
{
    delete m_tarsimClient;
    m_tarsimClient = nullptr;
}

bool TarsimClientExposed::close()
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->close();
}

bool TarsimClientExposed::connect()
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->connect();
}

bool TarsimClientExposed::shutdown()
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->shutdown();
}

bool TarsimClientExposed::startRecording()
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->startRecording();
}

bool TarsimClientExposed::stopRecording()
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->stopRecording();
}

bool TarsimClientExposed::isSimulatorRunning()
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->isSimulatorRunning();
}

bool TarsimClientExposed::sendJointPositions(JointPositions_t &robotPosition)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->sendJointPositions(robotPosition);
}

bool TarsimClientExposed::sendJointPosition(JointPosition_t &robotPosition)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->sendJointPosition(robotPosition);
}

bool TarsimClientExposed::sendBaseFrame(Frame_t &msg)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->sendBaseFrame(msg);
}

bool TarsimClientExposed::sendCamera(Camera_t &msg)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->sendCamera(msg);
}

bool TarsimClientExposed::lockObjectToRigidBody(
        int indexObject,
        int indexRigidBody)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->lockObjectToRigidBody(indexObject, indexRigidBody);
}

bool TarsimClientExposed::unlockObjectFromRigidBody(int indexObject)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->unlockObjectFromRigidBody(indexObject);
}

bool TarsimClientExposed::executeForwardKinematics()
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->executeForwardKinematics();
}

bool TarsimClientExposed::getEndEffectorFrame(Frame_t &msg)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->getEndEffectorFrame(msg);
}

bool TarsimClientExposed::getRigidBodyFrame(
        int32_t indexRigidBody, int32_t indexFrame, Frame_t &msg)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->getRigidBodyFrame(indexRigidBody, indexFrame, msg);
}

bool TarsimClientExposed::getObjectFrame(int32_t indexObject, Frame_t &msg)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->getObjectFrame(indexObject, msg);
}

bool TarsimClientExposed::setObjectFrame(int32_t indexObject, Frame_t &msg)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->setObjectFrame(indexObject, msg);
}

bool TarsimClientExposed::getJointValues(
    JointPositions_t &msg, int timeout_period_us)
{
    if (m_tarsimClient == nullptr) {
        printf("TarsimClient was not initialized\n");
        return false;
    }
    return m_tarsimClient->getJointValues(msg, timeout_period_us);
}

ErrorMessage_t TarsimClientExposed::getErrorMessage()
{
    return m_tarsimClient->getErrorMessage();
}

bool TarsimClientExposed::displayMessage(
		const char* message,
		FaultLevels level)
{
	return m_tarsimClient->displayMessage(message, level);
}

void TarsimClientExposed::getIncrementalCommand(
		int32_t &type, int32_t &index, int32_t &incCmd)
{
	return m_tarsimClient->getIncrementalCommand(type, index, incCmd);
}

float TarsimClientExposed::getSpeed()
{
	return m_tarsimClient->getSpeed();
}

std::vector<std::pair<int32_t, int32_t>> TarsimClientExposed::getSelfCollisions()
{
	return m_tarsimClient->getSelfCollisions();
}

std::vector<std::pair<int32_t, int32_t>> TarsimClientExposed::getExternalCollisions()
{
	return m_tarsimClient->getExternalCollisions();
}


}; // end of namespace tarsim

