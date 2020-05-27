/**
 *
 * @file: eitOsMsgServerReceiver.h
 *
 * @Created on: Jul 22, 2017
 * @Author: Kamran Shamaei
 *
 *
 * @brief - Header definition for RobotJointRecevier
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

#ifndef SRC_LIBS_ROBOTCONTROL_SERVER_H
#define SRC_LIBS_ROBOTCONTROL_SERVER_H

#include "eitOsMsgServerSender.h"
#include "msgQServer.h"
#include "timerUtils.h"
#include <map>

namespace tarsim {
class Kinematics;
class Gui;
class ConfigParser;
class Node;

class EitOsMsgServerReceiver : public MsgQServer
{
public:

	EitOsMsgServerReceiver(
	        ConfigParser* cp, Kinematics* kin, Gui* gui,
	        int policy, int priority, unsigned int msgPriority);
	virtual ~EitOsMsgServerReceiver();
    virtual Errors start();
    Errors sendIncrementalCommand(int32_t incCmd);
    Errors sendSpeed(float speed);
    Errors sendJointIncrementalCommand(int32_t jntIndex, int32_t incCmd);
    Errors sendCollision(const std::map<int32_t, Collision> &collisions);

private:
	virtual void onMessage(const GenericData_t &inComingData) override;

	EitOsMsgServerSender *getUserConnection(const int32_t userPid);

	void updateRobotJointPositions(
	        EitOsMsgServerSender *sendUserReply, const JointPositions_t& pos);

	void updateRobotJointPosition(
	            EitOsMsgServerSender *sendUserReply, const JointPosition_t& pos);

  void installTool(RequestInstallTool_t &msg);
  void setEndEffector(SetEndEffector_t &msg);

	TimerUtils *m_runTimer = nullptr;
	Kinematics* m_kinematics = nullptr;
	Gui* m_gui = nullptr;
	ConfigParser* m_cp = nullptr;
	int32_t m_msgCounter = 0;
	std::map <int32_t , EitOsMsgServerSender *> m_listofUsers;//list of participants,
	unsigned int m_msgPriority = 0;
};
} // end of namespace tarsim
#endif /* SRC_LIBS_ROBOTCONTROL_SERVER_H */
