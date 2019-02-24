/**
 *
 * @file: eitSimExposed.h
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
#ifndef EIT_CLIENT_WRAPPER_H
#define EIT_CLIENT_WRAPPER_H
#include <tarsimClient.h>
#include "ipcMessages.h"
#include "simulatorMessages.h"

namespace tarsim {

extern "C" {
bool initialize();
bool stop();
bool connect();
bool shutdown();
bool isSimulatorRunning();
bool startRecording();
bool stopRecording();

// Send functions
bool sendJointPositions(JointPositions_t &robotPosition);
bool sendJointPosition(JointPosition_t &robotPosition);
bool sendBaseFrame(Frame_t &msg);
bool sendCamera(Camera_t &msg);

bool lockObjectToRigidBody(
        int indexObject,
        int indexRigidBody);
bool unlockObjectFromRigidBody(
        int indexObject);

bool executeForwardKinematics();

bool getEndEffectorFrame(Frame_t &msg);

bool getRigidBodyFrame(int32_t indexRigidBody, int32_t indexFrame, Frame_t &msg);

bool getObjectFrame(int32_t indexObject, Frame_t &msg);
bool setObjectFrame(int32_t indexObject, Frame_t &msg);
bool getJointValues(JointPositions_t &msg, int timeout_period_us);

// Receive functions
ErrorMessage_t getErrorMessage();

bool displayMessage(
		const char* message,
		int level);

void getIncrementalCommand(
		int32_t &type, int32_t &index, int32_t &incCmd);

float getSpeed();

std::vector<std::pair<int32_t, int32_t>> getSelfCollisions();

std::vector<std::pair<int32_t, int32_t>> getExternalCollisions();

} // extern "C"



class TarsimClientExposed {
public:
    TarsimClientExposed();
    ~TarsimClientExposed();
    bool close();
    bool connect();
    bool shutdown();
    bool startRecording();
    bool stopRecording();
    bool isSimulatorRunning();

    // Send functions
    bool sendJointPositions(JointPositions_t &msg);
    bool sendJointPosition(JointPosition_t &robotPosition);

    bool sendBaseFrame(Frame_t &msg);
    bool sendCamera(Camera_t &msg);

    bool lockObjectToRigidBody(
            int indexObject,
            int indexRigidBody);
    bool unlockObjectFromRigidBody(int indexObject);

    bool executeForwardKinematics();

    bool getEndEffectorFrame(Frame_t &msg);
    bool getRigidBodyFrame(int32_t indexRigidBody, int32_t indexFrame, Frame_t &msg);
    bool getObjectFrame(int32_t indexObject, Frame_t &msg);
    bool setObjectFrame(int32_t indexObject, Frame_t &msg);

    bool getJointValues(
        JointPositions_t &msg, int timeout_period_us);

    // Receive functions
    ErrorMessage_t getErrorMessage();

	bool displayMessage(
			const char* message,
			FaultLevels level);

	void getIncrementalCommand(
			int32_t &type, int32_t &index, int32_t &incCmd);

	float getSpeed();

	std::vector<std::pair<int32_t, int32_t>> getSelfCollisions();

	std::vector<std::pair<int32_t, int32_t>> getExternalCollisions();

private:
    TarsimClient* m_tarsimClient = nullptr;
};

}; // end of namespace tarsim
#endif /* EIT_CLIENT_WRAPPER_H */
