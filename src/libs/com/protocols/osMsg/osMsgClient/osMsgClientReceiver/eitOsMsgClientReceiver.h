/**
 *
 * @file: eitOsMsgClientReceiver.h
 *
 * @Created on: May 05, 2018
 * @Author: Kamran Shamaei
 *
 *
 * @brief - Header definition for EitOsMsgClientReceiver
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

#ifndef EIT_RECEIVER_H
#define EIT_RECEIVER_H

#include "msgQServer.h"
#include "timerUtils.h"
#include "simulatorMessages.h"
#include <mutex>
#include <vector>
#include <utility>

namespace tarsim {
class EitOsMsgClientReceiver : public MsgQServer
{
public:
    EitOsMsgClientReceiver(
            const std::string &mqName,
            int policy = DEFAULT_RT_THREAD_POLICY,
            int priority = DEFAULT_RT_THREAD_PRIORITY);
	virtual ~EitOsMsgClientReceiver();
	virtual Errors start();

    void setEndEffectorFrame(const Frame_t &msg);
    void setRigidBodyFrame(const Frame_t &msg);
    void setObjectFrame(const Frame_t &msg);
    void setErrorMessage(const ErrorMessage_t &msg);
    void setJointValues(const JointPositions_t &msg);

	Frame_t getEndEffectorFrame();
	Frame_t getRigidBodyFrame();
	Frame_t getObjectFrame();
	ErrorMessage_t getErrorMessage();
	JointPositions_t getJointValues();
	bool getIsSimulatorRunning();

	void getIncrementalCommand(
	        int32_t &type, int32_t &index, int32_t &incCmd);

	float getSpeed();

	std::vector<std::pair<int32_t, int32_t>> getSelfCollisions();
	std::vector<std::pair<int32_t, int32_t>> getExternalCollisions();

private:
	virtual void onMessage(const GenericData_t &inComingData) override;
	TimerUtils *m_runTimer = nullptr;

    /**
    * Sets the value of m_isSimulatorRunning
    * @param isSimRunning Whether the simulator is running
    * @return void
    */
    void setIsSimulatorRunning(bool isSimRunning);

    void setCollisions(const CollisionMessage_t &msg);

    void setIncrementalCommand(
            IncrementalCommandTypes type, int32_t index, int32_t incCmd);

    void setSpeed(float speed);

	mutable std::mutex m_mutex;
	Frame_t m_frameEndEffector {};
	Frame_t m_frameRigidBody {};
	Frame_t m_frameObject {};
	ErrorMessage_t m_faultMessage {};
	JointPositions_t m_jointPositions {};

	int32_t m_incCmd = -1;
	IncrementalCommandTypes m_incCmdType = INC_CMD_TYPE_UNKNOWN;
    int32_t m_incCmdIndex = -1;

	mutable std::mutex m_mutexIncCmd;

	mutable std::mutex m_mutexCollisions;
	std::vector<std::pair<int32_t, int32_t>> m_selfCollisions;
	std::vector<std::pair<int32_t, int32_t>> m_externalCollisions;


	mutable std::mutex m_mutexSpeed;
	float m_speed = 50.0;
    /**
    * Flag whether the simulator is running, set by the simulator
    */
    bool m_isSimRunning = false;

    /**
    * Mutex for flag whether the simulator is running.
    */
    mutable std::mutex m_mutexIsSimRunning;
};
} // end of namespace tarsim
#endif /* EIT_RECEIVER_H */
