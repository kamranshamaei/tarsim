/**
 *
 * @file: kinematics.h
 *
 * @Created on: April 15, 2018
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

// IFNDEF
#ifndef KINEMATICS_H
#define KINEMATICS_H

//INCLUDES
#include <string>
#include <vector>
#include <stdexcept>
#include <Eigen>

#include "eitErrors.h"
#include "node.h"
#include "object.h"
#include "collisionDetection.h"

#include "eitServer.h"
#include "simulatorMessages.h"
#include "configParser.h"
#include <chrono>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS

using namespace Eigen;

// CLASS DEFINITION
class Kinematics
{
public:
    // FUNCTIONS
    Kinematics(ConfigParser* cp);
    virtual ~Kinematics();

    Errors executeForwardKinematics(
            GuiStatusMessage_t &statusMessage,
            std::map<int32_t, Collision> &collisions);
    Node* getRoot();
    ThreadQueue<Camera_t>* getCameraDataQueue();

    unsigned int getCounter();
    void setCounter(unsigned int counter);
    void incCounter();

    Errors lockObjectsToRb(int indexObject, int indexRb);
    Errors unlockObjectsFromRigidBody(int indexObject);

    Errors getObjectFrame(int indexObject, Matrix4d &xfm);
    Errors setObjectFrame(int indexObject, const Matrix4d &xfm);
    Errors getRigidBodyFrame(int indexRigidBody, int indexFrame, Matrix4d &xfm);

    Matrix4d getXfmEndEffector();

    Errors getJointValues(std::map<int, double> &jointValues);

    std::map<int, Object*> getObjects();

    Errors installTool();
    // MEMBERS
private:
    // FUNCTIONS
    static void* wrapperKinematicsThreadFunction(void* object);
    Errors kinematicsThreadFunction();
    Errors calculateChildrenXfm(Node* node, Matrix4d &xfmEndEffector);
    Errors calculateNodeToParentXfm(Node* node, Matrix4d &xfmNodeToParent);
    Errors calculateObjectsXfm();

    Errors calculateXfmRevoluteJoint(
            Matrix4d &xfmCurrentJointToParentJoint, Node* node);

    Errors calculateXfmPrismaticJoint(
            Matrix4d &xfmCurrentJointToParentJoint, Node* node);

    Errors initializeObjectsXfms();
    void setXfmEndEffector(const Matrix4d &m);

    GuiStatusMessage_t extractStatusMessage(double jvDuration, double fkDuration);

    Errors getNodeJointValue(Node* node, std::map<int, double> &jointValues);


    bool isCollisionDetected();
    Errors detectCollisionNode(Node* node, bool &isCollisionDetected);
    Errors detectCollisionNodeObjects(Node* node, bool &isCollisionDetected);
    void clearCollisions(Node* node);
    Errors detectCollisionNodeCluster(
            Node* node, Node* cluster, bool &isCollisionDetected);
    Errors detectCollisionNodeNode(Node* node1, Node* node2, bool &isCollision);
    bool isInCollisionDetectionList(Node* node);

    void updateCurrentXfms(Node* node);
    void updateCurrentJointValues(Node* node);

    // MEMBERS
    ConfigParser* m_cp = nullptr;
    Node* m_root = nullptr;

    mutable std::mutex m_mutexXfmEndEffector;
    Matrix4d m_xfmEndEffector = Matrix4d::Zero();

    unsigned int m_counter = 0;
    mutable std::mutex m_mutexCounter;

    ThreadQueue<Camera_t> m_cameraDataQueue;

    mutable std::mutex m_mutexObjects;
    std::map<int, Object*> m_mapObjects;

    double m_avgKinematicsCycle = 0.0;
    double m_minKinematicsCycle = 1e6;
    double m_maxKinematicsCycle = -1e6;
    double m_controlCycle = 1.0;
    double m_controlCycleTolerance = 0.01;
    const double k_epsilon = 1e-12;

    std::chrono::high_resolution_clock::time_point m_timePreviousJointValues =
            std::chrono::high_resolution_clock::now();

    CollisionDetection m_cd;

    std::map<int32_t, Collision> m_collisions;

    Object* m_tool = nullptr;
};
} // end of namespace tarsim
// ENDIF
#endif /* KINEMATICS_H */
