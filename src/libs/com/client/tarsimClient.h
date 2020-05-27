/*
* @file: tarsimClient.h
*
* @Created on: March 31, 2018
* @Author: Kamran Shamaei
*
*
* @brief - This class can be used to communicate with the simulator. All you need
* to do is to run simulatgor and instnatiate this class.
* <Requirement Doc Reference>
* <Design Doc Reference>
*
* @copyright Copyright [2017-2018] Kamran Shamaei .
* All Rights Reserved.
*
* This file is subject to the terms and conditions defined in
* file 'LICENSE', which is part of this source code package.
*/


// IFNDEF
#ifndef TARSIM_CLIENT_H
#define TARSIM_CLIENT_H

//INCLUDES
#include "simulatorMessages.h"
class EitOsMsgClientSender;
class EitOsMsgClientReceiver;


namespace tarsim {
class TarsimClient
{
public:
    /**
     * Constructor
     * @param policy Server thread scheduling policy
     * @param priority Server thread policy
     */
    TarsimClient(
            int32_t index = 0,
            int policy = DEFAULT_RT_THREAD_POLICY,
            int priority = DEFAULT_RT_THREAD_PRIORITY);

    /**
     * Destructor
     */
    virtual ~TarsimClient();

    /**
     * Connects to simulator, must be used before any other function only once
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool connect(unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends all desired joint positions to the simulator
     * @param robotPosition desired joint positions
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool sendJointPositions(
            JointPositions_t &robotPosition,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends one desired joint position to the simulator
     * @param robotPosition desired joint position
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool sendJointPosition(
            JointPosition_t &robotPosition,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends desired pose of the base frame. You can use this to move the
     * robot base in space
     * @param msg desired base link frame
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool sendBaseFrame(
            Frame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends desired camera info. You could use this to move the camera and all
     * its options.
     * @param msg Camera attributes
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool sendCamera(
            Camera_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends a command to lock an object to a robot link. You could use this
     * to grasp an object with the robot.
     * @param indexObject The index of the object to lock
     * @param indexRigidBody The index of the robot link to lock to
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool lockObjectToRigidBody(
            int indexObject,
            int indexRigidBody,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends a command to unlock an object from a robot link. You could use this
     * to release an object from the robot.
     * @param indexObject The index of the object to unlock
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool unlockObjectFromRigidBody(
            int indexObject,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends a command to execute forward kinematics. You should send this
     * command everytime you send joint positions, otherwise the robot
     * configuration wont be updated.
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool executeForwardKinematics(
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Gets the latest pose of the end-effector
     * @param msg The pose of the end-effector
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool getEndEffectorFrame(
        Frame_t &msg, int timeout_period_us = k_defaultTimeoutPeriodUs,
        unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Gets a frame of a rigid-body (robot link) in world coordinate frame
     * @param indexRigidBody The rigid body index
     * @param indexFrame The frame index (a rigid body might have multiple
     * frames defined by the user)
     * @param msg The frame in world coordinate
     * @param timeout_period_us How long we should wait for a response
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool getRigidBodyFrame(
        int32_t indexRigidBody, int32_t indexFrame,
        Frame_t &msg, int timeout_period_us = k_defaultTimeoutPeriodUs,
        unsigned int msgPriority = DEFAULT_MSG_PRIORITY);


    /**
     * Gets a frame of an object in world coordinate frame
     * @param indexObject The object index
     * @param msg The object frame in world coordinate
     * @param timeout_period_us How long we should wait for a response
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool getObjectFrame(
        int32_t indexObject, Frame_t &msg,
        int timeout_period_us = k_defaultTimeoutPeriodUs,
        unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sets the frame of an object in world coordinate frame
     * @param indexObject The object index
     * @param msg The object frame in world coordinate. You only need to set mij's
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool setObjectFrame(
            int32_t indexObject,
            Frame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Gets the latest joint values
     * @param msg The latest joint values returned
     * @param timeout_period_us How long we should wait for a response
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool getJointValues(
        JointPositions_t &msg,
        int timeout_period_us = k_defaultTimeoutPeriodUs,
        unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Gets the error message of the simulator
     * @param msgPriority Message priority
     * @return the error message of simulator
     */
    ErrorMessage_t getErrorMessage(
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends a message to simulator to shutdown
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool shutdown(
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * If the simulator is currently running
     * @param msgPriority Message priority
     * @return whether the simulator is running
     */
    bool isSimulatorRunning(
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Request the gui to start recording robot scene
     * @param msgPriority Message priority
     * @return whether the simulator is running
     */
    bool startRecording(
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Request the gui to stop recording robot scene
     * @param msgPriority Message priority
     * @return whether the simulator is running
     */
    bool stopRecording(
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Display a text message on the fault scene to the user
     * @param message Message to display
     * @param level Level of how critical the message is. This defines
     * the color of the circle that shows up next to the text on the gui.
     * A fault levelt of FAULT_LEVEL_NOFAULT will show not messages
     * @param msgPriority Message priority
     * @return whether the simulator is running
     */
    bool displayMessage(
            const char* message,
            FaultLevels level,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Closes the connected with simulator
     * @return the error message of simulator
     */
    bool close();

    /**
     * Get current incremental command. This command comes from the six dof
     * scene or the joint values scene. Every time a button is pressed or
     * release in those scenes, these values change. The values are integers
     * that correspond to the button that was pressed. For example, if the x+
     * button is pressed, the values you get would be type = 0,
     * index = -1, and incCmd would be 0. Index in unused for Cartesian
     * commands. If button x- is pressed, the value you get would be type = 0,
     * index = -1, and incCmd would be 0. If joint 3 increment button is
     * pressed, the values would be type = 1, index = 3, and
     * incCmd would be 1. If joint 4 decrement button is
     * pressed, the values would be type = 1, index = 4, and
     * incCmd would be 0.     *
     * The Cartesian buttons are in this order:
     * x+, x-, y+, y-, z+, z-, roll+, roll-, pitch+, pitch-, yaw+, yaw-
     * If this value is -1, then no button is pressed.
     * The joint buttons are in this order:
     * decrement, increment
     */
    void getIncrementalCommand(
            int32_t &type, int32_t &index, int32_t &incCmd);

    /**
     * Gets the value of the speed slider. This can be used to set how fast
     * the robot moves or an action takes place. The value ranges from 0 to 100
     */
    float getSpeed();

    /**
     * Get a vector of indices of robot links (i.e. rigid bodes) that are
     * currently in collision. If no collision is detected, the vector size
     * would be zero. This only include robot links colliding with other robot
     * links
     */
    std::vector<std::pair<int32_t, int32_t>> getSelfCollisions();

    /**
     * Get a vector of indices of robot links (i.e. rigid bodes) with external
     * objects. If no collision is detected, the vector size
     * would be zero. This only include robot links colliding with external
     * objects
     */
    std::vector<std::pair<int32_t, int32_t>> getExternalCollisions();

    /**
     * Install a tool on the robot's end-effector link.
     * @param name Tool's name as appear as a file in the config folder
     * @return true if successful, false if it fails
     */
    bool installTool(const std::string &name);

    /**
     * Sets a rigid body on the arm as the end-effector
     * @param robotLink The index of the link that has the end effector frame
     * @param linkFrame The index of the end effector frame
     * @return true if successful, false if it fails
     */
    bool setEndEffector(
        int32_t robotLink,
        int32_t linkFrame,
        unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

private:
    /**
     * Returns a time stamp for the message
     * @return The message time stamp
     */
    int32_t getMsgStamp();

    /**
     * Sends request to get the end-effector pose
     * @param msgThe message request to the end-effector pose
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool sendRequestEndEffectorFrame(
            RequestEndEffectorFrame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends request to get the rigid-body pose
     * @param msgThe message request to the rigid body pose
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool sendRequestRigidBodyFrame(
            RequestRigidBodyFrame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends request to get the object pose
     * @param msgThe message request to the object pose
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool sendRequestObjectFrame(
            RequestObjectFrame_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Sends request to get joint values
     * @param msgThe message request to the joint values
     * @param msgPriority Message priority
     * @return true if successful, false if it fails
     */
    bool sendRequestJointValues(
            RequestJointValues_t &msg,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);


    /**
     * The tarsimClient message queue receiver. It receives messages from simulator
     */
    EitOsMsgClientReceiver* m_eitOsMsgClientReceiver = nullptr;

    /**
     * The tarsimClient message queue sender. It sends messages to simulator
     */
    EitOsMsgClientSender* m_eitOsMsgClientSender = nullptr;

    /**
     * How long we sleep between every time we check if we received a response.
     */
    const int k_sleepTimeUs = 10;

    /**
     * Default timeout duration in us. It is used when we query data from
     * simulator and wait for a response
     */
    static const int k_defaultTimeoutPeriodUs = 100000;

    /**
     * Number of messages sent
     */
    int32_t m_counter = 0;
};
} // end of namespace tarsim
// ENDIF
#endif /* TARSIM_CLIENT_H */
