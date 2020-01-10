/**
* @file:simulatorMessages.h
*
* @Created on: Jul 22, 2017
* @Author: Kamran Shamaei
*
*
* @brief - All the message a user needs to communicate with the simulator
* ar defined here.
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

#ifndef SRC_LIBS_INC_SIMULATOR_MESSAGES_H_
#define SRC_LIBS_INC_SIMULATOR_MESSAGES_H_

#include <cstdint>
#include <string>
#include <vector>
#include "ipcMessages.h"

namespace tarsim {
/**
 * Maximum number of joints in a robot
 */
const int32_t MAX_JOINTS = 20;

/**
 * The number of elements in a 4x4 matrix
 */
const int32_t FRAME_INDICES = 16;

/**
 * Message type used for error communication
 */
struct ErrorMessage_t : MessageHeader_t
{
    char errorMsg[LOG_MAX_DATA_SIZE];
    int32_t msgLength = 0;
    int32_t errorId = 0;
};

/**
 * Maximum number of collisions for each rigid body
 */
const int32_t MAX_COLLISIONS = 5;

/**
 * Message type used for communication of all robot joint values
 */
struct JointPositions_t : MessageHeader_t
{
    int32_t numJoints = 0;
    int32_t indices [MAX_JOINTS];
    float positions [MAX_JOINTS];
};

/**
 * Message type used for communication of one robot joint value
 */
struct JointPosition_t : MessageHeader_t
{
    int32_t index = 0;
    float position = 0.0;
};

/**
 * Message type used for communication of a frame (pose/matrix)
 */
struct Frame_t : MessageHeader_t
{
    float mij[FRAME_INDICES];
    int32_t frameId = 0;
};

/**
 * Message type used for communication of camera attributes
 */
struct Camera_t : MessageHeader_t
{
    float position[3];
    float focalPoint[3];
    float viewUp[3];
    float clippingRange[2];
};

/**
 * Message type used for communication of grasping of an object with a robot
 * link.
 */
struct LockObjectToRigidBody_t : MessageHeader_t
{
    int32_t indexObject = 0;
    int32_t indexRigidBody = 0;
};

/**
 * Message type used for communication of releasing of an object from the robot
 */
struct UnlockObjectFromRigidBody_t : MessageHeader_t
{
    int32_t indexObject = 0;
};

/**
 * Message type used for communication of requesting to execute forward
 * kinematics
 */
struct RequestExecuteForwardKinematics_t : MessageHeader_t
{
};

/**
 * Message type used for communication of requesting to get end-effector frame
 */
struct RequestEndEffectorFrame_t : MessageHeader_t
{
};

/**
 * Message type used for communication of requesting a rigid-body frame. Each
 * robot has multiple rigid-bodies and each rigid body can have multiple frames
 */
struct RequestRigidBodyFrame_t : MessageHeader_t
{
    int32_t indexRigidBody = 0;
    int32_t indexFrame = 0;
};

/**
 * Message type used for communication of requesting an object frame. The scene
 * can have multiple objects, and each object has one frame.
 */
struct RequestObjectFrame_t : MessageHeader_t
{
    int32_t indexObject = 0;
};

typedef enum IncrementalCommandType
{
    INC_CMD_TYPE_UNKNOWN = -1,
    INC_CMD_TYPE_CARTESIAN,
    INC_CMD_TYPE_JOINT,

    INC_CMD_MIN = INC_CMD_TYPE_CARTESIAN,
    INC_CMD_MAX = INC_CMD_TYPE_JOINT,
    INC_CMD_TOTAL = INC_CMD_MAX + 1,
} IncrementalCommandTypes;

typedef enum FaultLevel
{
    FAULT_LEVEL_NOFAULT,
    FAULT_LEVEL_INFO,
    FAULT_LEVEL_WARNING,
    FAULT_LEVEL_MINOR,
    FAULT_LEVEL_MAJOR,
    FAULT_LEVEL_CRITICAL,

    FAULT_LEVEL_MIN = FAULT_LEVEL_NOFAULT,
    FAULT_LEVEL_MAX = FAULT_LEVEL_CRITICAL,
    FAULT_LEVEL_TOTAL = FAULT_LEVEL_CRITICAL + 1,
} FaultLevels;


typedef enum FaultType
{
    FAULT_TYPE_NOFAULT,
    FAULT_TYPE_CLIENT,
    FAULT_TYPE_LICENSE,
    FAULT_TYPE_KIN_CYCLE,
    FAULT_TYPE_EXTREME_FEED_RATE,
    FAULT_TYPE_CONTROL_CYCLE,

    FAULT_TYPE_MIN = FAULT_TYPE_NOFAULT,
    FAULT_TYPE_MAX = FAULT_TYPE_CONTROL_CYCLE,
    FAULT_TYPE_TOTAL = FAULT_TYPE_CONTROL_CYCLE + 1,
} FaultTypes;

/**
 * Message type used for communication of the simulator gui status
 */
struct GuiStatusMessage_t : MessageHeader_t
{
    char statusMessage[MAX_STATUS_TEXT_SIZE];
    FaultLevels faultLevel = FaultLevels::FAULT_LEVEL_NOFAULT;
    FaultTypes faultType = FaultTypes::FAULT_TYPE_NOFAULT;
};

/**
 * Message type used for communication of requesting all robot joint values
 */
struct RequestJointValues_t : MessageHeader_t
{
};

/**
 * Message type used for communication of requesting all robot joint values
 */
struct RequestInstallTool_t : MessageHeader_t
{
  char toolName[MAX_STATUS_TEXT_SIZE];
};


/**
 * Message type used for communication of requesting simulator to shutdown
 */
struct RequestShutdown_t : MessageHeader_t
{
};

/**
 * Message type used to request recording robot scene
 */
struct RequestRecord_t : MessageHeader_t
{
    bool isStart = false;
};

/**
 * Message type used for communication of simulator status
 */
struct SimulatorStatus_t : MessageHeader_t
{
    bool isSimRunning = false;
};

/**
 * Message type used for communication of simulator status. It consists of one
 * integer per DOF, which is incremented or decremented
 */
struct IncrementalCommandMessage_t : MessageHeader_t
{
    int32_t incCmd = -1;
    int32_t index = -1;
    IncrementalCommandTypes type = INC_CMD_TYPE_UNKNOWN;
};

/**
 * Message type used for communication of speed. It consists of one
 * float that can be used to set the robot movement speed. The value
 * ranges from 0 to 100.
 */
struct SpeedMessage_t : MessageHeader_t
{
    float speed = 50.0;
};

struct Collision
{
    int32_t robotLink = 0;
    int32_t numCollisions = 0;
    int32_t rigidBody[MAX_COLLISIONS];
    bool isSelfCollision[MAX_COLLISIONS];
};

/**
 * Message type used for communication of a collision. indices provides the
 * indices of robot's links (rigid bodies) that are in collision with another
 * robot link or an object
 */
struct CollisionMessage_t : MessageHeader_t
{
    int32_t numCollisions = 0;
    Collision collisions[MAX_JOINTS];
};

/**
 * Union of all data structure
 */
union SimulatorData_t
{
    GenericData_t   genericMessage;
    JointPositions_t  robotJointsPositions;
};

/**
 * Enums for all simulator message types
 */
enum SimulatorMessageId_t
{
    SIM_FIRST_MSG = MSG_FIRST_APPLICATION,

    FAULT_MESSAGE,

    ROBOT_JOINT_POSITIONS,
    ROBOT_JOINT_POSITION,
    ROBOT_BASE_POSE,

    CAMERA_DATA,

    END_EFFECTOR_FRAME,
    RIGID_BODY_FRAME,
    OBJECT_FRAME,

    LOCK_OBJECT_TO_RIGID_BODY,
    UNLOCK_OBJECT_FROM_RIGID_BODY,

    REQUEST_EXECUTE_FORWARD_KINEMATICS,
    REQUEST_END_EFFECTOR_FRAME,
    REQUEST_RIGID_BODY_FRAME,
    REQUEST_OBJECT_FRAME,
    REQUEST_JOINT_VALUES,
    REQUEST_START_RECORD,
    REQUEST_STOP_RECORD,

    INCREMENTAL_COMMAND,
    SPEED_COMMAND,
    COLLISION,

    GUI_STATUS_MESSAGE,
    SIMULATOR_STATUS,
    SHUTDOWN,
    INSTALL_TOOL
};
} // end of namespace tarsim
#endif /* SRC_LIBS_INC_SIMULATOR_MESSAGES_H_ */
