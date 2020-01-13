/**
 * @file: kinematics.cpp
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

//INCLUDES
#include "kinematics.h"
#include <map>
#include <chrono>
#include <cmath>
#include "logClient.h"


namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
#define RAD(angleDegrees) (angleDegrees * M_PI / 180.0)
const int NO_FAULT_MESSAGE_SILENCE_DURATION = 10; //sec
const int FAULT_MESSAGE_SILENCE_DURATION = 1; //sec
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
Kinematics::Kinematics(ConfigParser* cp)
{
    if (cp == nullptr) {
        throw std::invalid_argument("No config parser was received");
    }
    m_cp = cp;

    if (cp->getRoot() == nullptr) {
        throw std::invalid_argument("No tree was received");
    }
    m_root = cp->getRoot();

    for (unsigned int i = 0; i < m_cp->getRbs()->rigid_bodies_size(); i++) {
        int index = m_cp->getRbs()->rigid_bodies(i).index();
        if (m_cp->getNodeOfRigidBody(index) == nullptr) {
            throw std::invalid_argument("At least one rigid body is undefined");
        }
    }

    if (m_cp->getRbs()->control_cycle() > 0.0) {
        m_controlCycle = m_cp->getRbs()->control_cycle();
    }

    if (m_cp->getRbs()->control_cycle_tolerance() > 0.0) {
        m_controlCycleTolerance = m_cp->getRbs()->control_cycle_tolerance();
    }

    GuiStatusMessage_t msg;
    std::map<int32_t, Collision> collisions;
    if (NO_ERR != executeForwardKinematics(msg, collisions))
    {
        throw std::invalid_argument("Failed to calculate initial kinematics");
    }

    if (NO_ERR != initializeObjectsXfms())
    {
        throw std::invalid_argument("Failed to calculate initial object xfms");
    }
}

Kinematics::~Kinematics()
{
    for (auto pair: m_mapObjects) {
        delete pair.second;
        pair.second = nullptr;
    }
}

Errors Kinematics::executeForwardKinematics(
        GuiStatusMessage_t &statusMessage,
        std::map<int32_t, Collision> &collisions)
{
    incCounter();
    using namespace std::chrono;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    Matrix4d m = Matrix4d::Zero();
    if (NO_ERR != calculateChildrenXfm(m_root, m)) {
	    LOG_FAILURE("Failed to calculate forward kinematics");
	    return ERR_INVALID;
	  }

    if (NO_ERR != calculateObjectsXfm()) {
        LOG_FAILURE("Failed to calculate objects xfm");
        return ERR_INVALID;
    }

    if (m_cp->getRbs()->collision_detection().is_active() && getCounter() > 1) {
        if (!isCollisionDetected()) {
            updateCurrentJointValues(m_root);
            updateCurrentXfms(m_root);
            m_collisions.clear();
            setXfmEndEffector(m);
            if (m_tool) {
                m_tool->setXfm(m);
                m_tool->updateFrames(m);
            }
        }
    } else {
        updateCurrentJointValues(m_root);
        updateCurrentXfms(m_root);
        setXfmEndEffector(m);
        if (m_tool) {
            m_tool->setXfm(m);
            m_tool->updateFrames(m);
        }
    }

    collisions = m_collisions;

    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    duration<double> time_span =
            duration_cast<duration<double>>(t2 - t1);
    double fkDuration = 1000.0 * time_span.count();

    // Calculate joint receipt duration
    time_span = duration_cast<duration<double>>(t1 - m_timePreviousJointValues);
    double jvDuration = 1000.0 * time_span.count();

    statusMessage = extractStatusMessage(jvDuration, fkDuration);

    m_timePreviousJointValues = t1;
	  return NO_ERR;
}

bool Kinematics::isCollisionDetected()
{
    bool isCollisionDetected = false;

    clearCollisions(m_root);
    m_collisions.clear();
    if (NO_ERR != detectCollisionNode(m_root, isCollisionDetected)) {
        LOG_WARNING("Failed to execute collision detection algorithm");
    }

    return isCollisionDetected;
}

void Kinematics::clearCollisions(Node* node)
{
    node->setIsCollisionDetected(false);
    for (size_t i = 0; i < node->getChildren().size(); i++) {
        clearCollisions(node->getChildren().at(i));
    }
}

Errors Kinematics::detectCollisionNode(Node* node, bool &isCollisionDetected)
{
    if (NO_ERR != detectCollisionNodeCluster(node, node, isCollisionDetected)) {
        LOG_WARNING("Failed to execute collision detection algorithm");
    }

    if (NO_ERR != detectCollisionNodeObjects(node, isCollisionDetected)) {
        LOG_WARNING("Failed to execute collision detection algorithm");
    }

    for (size_t i = 0; i < node->getChildren().size(); i++) {
        Node* child = node->getChildren().at(i);
        if (NO_ERR != detectCollisionNode(child, isCollisionDetected)) {
            LOG_FAILURE("Failed to check for collisions for node %s and %s",
                    node->getName().c_str(), child->getName().c_str());
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

Errors Kinematics::detectCollisionNodeObjects(
        Node* node, bool &isCollisionDetected)
{
    for (size_t i = 0; i < node->getBbs()->size(); i++) {
        BoundingBoxBase* bb1 =  node->getBbs()->at(i);
        bb1->updateVertices(node->getTargetXfm());
        // Check for collision with external objects
        for (auto pair: m_mapObjects) {
            for (size_t j = 0; j < pair.second->getBbs()->size(); j++) {
                BoundingBoxBase* bb2 =  pair.second->getBbs()->at(j);
                // Update xfm of vertices
                bb2->updateVertices(m_mapObjects[pair.first]->getXfm());

                bool result = false;
                if (NO_ERR != m_cd.check(bb1, bb2, result)) {
                    LOG_FAILURE("Failed to check for collisions");
                    return ERR_INVALID;
                }

                if (result) {
                    isCollisionDetected = result;
                    node->setIsCollisionDetected(result);

                    int32_t robotLink = node->getRigidBody().index();
                    if (m_collisions.find(robotLink) == m_collisions.end()) {
                        Collision collision;
                        collision.robotLink = robotLink;
                        collision.rigidBody[0] = pair.first;
                        collision.isSelfCollision[0] = false;
                        collision.numCollisions = 1;
                        m_collisions[robotLink] = collision;
                    } else {
                        Collision collision = m_collisions[robotLink];
                        size_t ind = std::min((size_t)collision.numCollisions, (size_t)(MAX_COLLISIONS));
                        collision.rigidBody[ind] = pair.first;
                        collision.isSelfCollision[ind] = false;
                        collision.numCollisions++;
                        m_collisions[robotLink] = collision;
                    }
                }
            }
        }
    }
    return NO_ERR;
}

Errors Kinematics::detectCollisionNodeCluster(
        Node* node, Node* cluster, bool &isCollisionDetected)
{
    for (size_t i = 0; i < cluster->getChildren().size(); i++) {
        Node* child = cluster->getChildren().at(i);
        if (NO_ERR != detectCollisionNodeNode(node, child, isCollisionDetected)) {
            LOG_FAILURE("Failed to check for collisions for node %s and %s",
                    node->getName().c_str(),
                    child->getName().c_str());
            return ERR_INVALID;
        }

        if (NO_ERR != detectCollisionNodeCluster(node, child, isCollisionDetected)) {
            LOG_FAILURE("Failed to check for collisions for nodes %s and %s",
                    node->getName().c_str(), child->getName().c_str());
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

Errors Kinematics::detectCollisionNodeNode(
        Node* node1, Node* node2, bool &isCollision)
{
    if (!isInCollisionDetectionList(node1) ||
        !isInCollisionDetectionList(node2)) {
        isCollision = false;
        return NO_ERR;
    }

    for (size_t i = 0; i < node1->getBbs()->size(); i++) {
        BoundingBoxBase* bb1 =  node1->getBbs()->at(i);

        // Check for self-collisions
        for (size_t j = 0; j < node2->getBbs()->size(); j++) {
            BoundingBoxBase* bb2 =  node2->getBbs()->at(j);

            // Update xfm of vertices
            bb1->updateVertices(node1->getTargetXfm());
            bb2->updateVertices(node2->getTargetXfm());

            bool result = false;
            if (NO_ERR != m_cd.check(bb1, bb2, result)) {
                LOG_FAILURE("Failed to check for collisions");
                return ERR_INVALID;
            }

            if (result) {
                isCollision = result;
                node1->setIsCollisionDetected(result);
                node2->setIsCollisionDetected(result);

                int32_t robotLink = node1->getRigidBody().index();
                if (m_collisions.find(robotLink) == m_collisions.end()) {
                    Collision collision;
                    collision.robotLink = robotLink;
                    collision.rigidBody[0] = node2->getRigidBody().index();
                    collision.isSelfCollision[0] = true;
                    collision.numCollisions = 1;
                    m_collisions[robotLink] = collision;
                } else {
                    Collision collision = m_collisions[robotLink];
                    size_t ind = std::min((size_t)collision.numCollisions, (size_t)(MAX_COLLISIONS));
                    collision.rigidBody[ind] = node2->getRigidBody().index();
                    collision.isSelfCollision[ind] = true;
                    collision.numCollisions++;
                    m_collisions[robotLink] = collision;
                }
            }
        }
    }
    return NO_ERR;
}

bool Kinematics::isInCollisionDetectionList(Node* node)
{
    for (size_t i = 0; i < m_cp->getRbs()->collision_detection().self_collisions().size(); i++) {
        if (node->getRigidBody().index() ==
                m_cp->getRbs()->collision_detection().self_collisions(i).first_rigid_body_index() ||
            node->getRigidBody().index() ==
                m_cp->getRbs()->collision_detection().self_collisions(i).second_rigid_body_index()) {
            return true;
        }
    }

    return false;
}

void Kinematics::updateCurrentXfms(Node* node)
{
    node->setXfm(node->getTargetXfm());
    for (size_t i = 0; i < node->getChildren().size(); i++) {
        updateCurrentXfms(node->getChildren().at(i));
    }
}

void Kinematics::updateCurrentJointValues(Node* node)
{
    node->setCurrentJointValue(node->getTargetJointValue());
    for (size_t i = 0; i < node->getChildren().size(); i++) {
        updateCurrentJointValues(node->getChildren().at(i));
    }
}

GuiStatusMessage_t Kinematics::extractStatusMessage(
        double jvDuration, double fkDuration)
{
    static double sumTime = 0.0;

    if (m_minKinematicsCycle > fkDuration) {
        m_minKinematicsCycle = fkDuration;
    }

    if (m_maxKinematicsCycle < fkDuration) {
        m_maxKinematicsCycle = fkDuration;
    }

    sumTime += fkDuration;

    m_avgKinematicsCycle = sumTime / getCounter();

    GuiStatusMessage_t statusMessage;
    statusMessage.faultLevel = FAULT_LEVEL_NOFAULT;
    statusMessage.faultType = FAULT_TYPE_NOFAULT;
    std::ostringstream avg, min, max;
    avg << std::fixed << std::setprecision(2) << m_avgKinematicsCycle;
    min << std::fixed << std::setprecision(2) << m_minKinematicsCycle;
    max << std::fixed << std::setprecision(2) << m_maxKinematicsCycle;

    std::string txt = std::string(
            "Average forward kinematics cycle is " + avg.str() +
            " ms [min = " + min.str() + ", max = " + max.str() + "]");

    if (getCounter() > 2) {
        if (jvDuration < fkDuration) {
            std::ostringstream jvCycle, fkCycle;
            jvCycle << std::fixed << std::setprecision(2) << jvDuration;
            fkCycle << std::fixed << std::setprecision(2) << fkDuration;

            statusMessage.faultLevel = FAULT_LEVEL_CRITICAL;
            statusMessage.faultType = FAULT_TYPE_EXTREME_FEED_RATE;
            txt = std::string("Joint values "
                    "received at " + jvCycle.str() + " ms, " +
                    "processed at " + fkCycle.str() + " ms");
            LOG_INFO("Timing Statistics: %s", txt.c_str());
        } else if (fabs(jvDuration - m_controlCycle) > m_controlCycleTolerance) {
            std::ostringstream cycle;
            cycle << std::fixed << std::setprecision(2) << jvDuration;

            statusMessage.faultLevel = FAULT_LEVEL_MAJOR;
            statusMessage.faultType = FAULT_TYPE_CONTROL_CYCLE;
            txt = std::string(
                    "Joint values were received after " + cycle.str() + " ms");
            LOG_INFO("Timing Statistics: %s", txt.c_str());
        } else if ((fkDuration > 1.0) && (getCounter() % 1000 == 0) &&
                (m_cp->getRbs()->control_cycle() > 0.0)) {
            std::ostringstream cycle;
            cycle << std::fixed << std::setprecision(2) << fkDuration;
            txt = std::string(
                    "Forward kinematics cycle (" + cycle.str() + " ms) exceeded + "
                    + std::to_string(m_cp->getRbs()->control_cycle()) + " ms");
            statusMessage.faultLevel = FAULT_LEVEL_MAJOR;
            statusMessage.faultType = FAULT_TYPE_KIN_CYCLE;
            LOG_INFO("Timing Statistics: %s", txt.c_str());
        }
    }

    const char* ctxt = txt.c_str();

    strncpy(statusMessage.statusMessage, ctxt, sizeof(statusMessage.statusMessage));
    statusMessage.statusMessage[MAX_STATUS_TEXT_SIZE - 1] = '\0';

    return statusMessage;
}

void Kinematics::setXfmEndEffector(const Matrix4d &m)
{
    std::unique_lock<std::mutex> lock(m_mutexXfmEndEffector);
    m_xfmEndEffector = m;
}

Matrix4d Kinematics::getXfmEndEffector()
{
    std::unique_lock<std::mutex> lock(m_mutexXfmEndEffector);
    Matrix4d m = m_xfmEndEffector;
    if (m_tool) {
        if (NO_ERR != m_tool->getFrame(0, m)) {
          return m_tool->getXfm();
        }
    }
    return m;
}

Errors Kinematics::calculateChildrenXfm(Node* node, Matrix4d &xfmEndEffector)
{
    if (node == nullptr) {
        LOG_INFO("Node is not defined");
        return ERR_INVALID;
    }
    // Calculate the current node transformation matrix
    if (node == m_root) {
        Matrix4d m = node->getXfm();
        node->setTargetXfm(m);
        node->updateFrames(m);

    } else {
        // Calculate the xfm wrt the parent frame
        Matrix4d xfmNodeToParent;
        if (NO_ERR != calculateNodeToParentXfm(node, xfmNodeToParent)) {
            LOG_FAILURE("Failed to calculate xfm for rigid body %d\n",
                    node->getRigidBody().index());
            return ERR_INVALID;
        }

        Matrix4d m = node->getParent()->getTargetXfm() * xfmNodeToParent;
        node->setTargetXfm(m);
        node->updateFrames(m);
    }

    // Get end-effector frame
    for (unsigned int i = 0; i < node->getRigidBody().frames_size(); i++) {
        if (node->getRigidBody().frames(i).is_end_effector()) {
            node->getFrame(i, xfmEndEffector);
        }
    }

    // Calculate the children nodes transformation matrix
    if (node->getChildren().size() > 0) {
        for (unsigned int i = 0; i < node->getChildren().size(); i++) {
            if (NO_ERR != calculateChildrenXfm(node->getChildren().at(i), xfmEndEffector)) {
                LOG_FAILURE("Failed to calculate children xfm for rigid body %d\n",
                        node->getChildren().at(i)->getRigidBody().index());
                return ERR_INVALID;
            }
        }
    }

    return NO_ERR;
}

Errors Kinematics::calculateNodeToParentXfm(
        Node* node, Matrix4d &xfmNodeToParent)
{
    Matrix4d xfmJmJn;
    if (Joint_JointType_REVOLUTE == node->getJointType()) {
        if (NO_ERR != calculateXfmRevoluteJoint(
                xfmJmJn, node)) {
            LOG_FAILURE("Failed to calculate revolute joint xfm for rigid body %d\n",
                    node->getRigidBody().index());
            return ERR_INVALID;
        }

    } else if (Joint_JointType_PRISMATIC == node->getJointType()) {
        if (NO_ERR != calculateXfmPrismaticJoint(
                xfmJmJn, node)) {
            LOG_FAILURE("Failed to calculate prismatic joint xfm for rigid body %d\n",
                    node->getRigidBody().index());
            return ERR_INVALID;
        }
    } else {
        LOG_FAILURE("Joint type %d is invalid/unsupported for rigid body %d\n",
                node->getJointType(), node->getRigidBody().index());
        return ERR_INVALID;
    }

    xfmNodeToParent = node->getXfm_m_jm() * xfmJmJn * node->getXfm_jn_n();
    return NO_ERR;
}

Errors Kinematics::calculateXfmRevoluteJoint(
        Matrix4d &xfmCurrentJointToParentJoint, Node* node)
{
    double value = node->getTargetJointValue() * node->getGearRatio();

    if (node->getMateToParent()->value_unit() == Mate_Unit_DEG) {
        value = RAD(value);
    }

    double angularOffset = node->getMateToParent()->angular_offset();

    if (node->getMateToParent()->angular_offset_unit() == Mate_Unit_DEG) {
        angularOffset = RAD(angularOffset);
    }

    // Calculate:
    // 1. Rotation matrix around z
    // 2. Translation along z axis of the joint
    double c = cos(value + angularOffset);
    double s = sin(value + angularOffset);
    double t = node->getMateToParent()->linear_offset();

    if (node->getMateToParent()->linear_offset_unit() == Mate_Unit_M) {
        t *= 1000.0;
    }

    Matrix4d m;
    m << c, -s,  0,  0,
         s,  c,  0,  0,
         0,  0,  1,  t,
         0,  0,  0,  1;

    xfmCurrentJointToParentJoint = m;
    return NO_ERR;
}


Errors Kinematics::calculateXfmPrismaticJoint(
        Matrix4d &xfmCurrentJointToParentJoint, Node* node)
{
    double value = node->getTargetJointValue() * node->getGearRatio();

    if (node->getMateToParent()->value_unit() == Mate_Unit_M) {
        value *= 1000.0;
    }

    double angularOffset = node->getMateToParent()->angular_offset();

    if (node->getMateToParent()->angular_offset_unit() == Mate_Unit_DEG) {
        angularOffset = RAD(angularOffset);
    }

    double linearOffset = node->getMateToParent()->linear_offset();
    if (node->getMateToParent()->linear_offset_unit() == Mate_Unit_M) {
        linearOffset *= 1000.0;
    }

    // Calculate:
    // 1. Rotation matrix around z
    // 2. Translation along z axis of the joint
    double c = cos(angularOffset);
    double s = sin(angularOffset);
    double t = value + linearOffset;
    Matrix4d m;
    m << c, -s,  0,  0,
         s,  c,  0,  0,
         0,  0,  1,  t,
         0,  0,  0,  1;

    xfmCurrentJointToParentJoint = m;
    return NO_ERR;
}

Errors Kinematics::initializeObjectsXfms()
{
    std::unique_lock<std::mutex> lock(m_mutexObjects);
    for (size_t i = 0; i < m_cp->getRbs()->objects_size(); i++) {
        ExternalObject obj = m_cp->getRbs()->objects(i);
        if (obj.has_appearance()) {
            Object* object = new Object(obj, m_cp->getConfigFolderName());
            m_mapObjects[obj.index()] = object;
        }
    }

    return NO_ERR;
}

Errors Kinematics::lockObjectsToRb(int indexObject, int indexRb)
{
    std::unique_lock<std::mutex> lock(m_mutexObjects);

    std::map<int, Object*>::iterator it = m_mapObjects.find(indexObject);

    if (it == m_mapObjects.end()) {
        LOG_FAILURE("Failed to find object %d", indexObject);
        return ERR_INVALID;
    }

    it->second->setIsLocked(true, indexRb);

    Matrix4d xfm_rb_world = m_cp->getNodeOfRigidBody(indexRb)->getXfm().inverse();
    it->second->setXfmObjectToRb(xfm_rb_world * it->second->getXfm());

    return NO_ERR;
}

Errors Kinematics::unlockObjectsFromRigidBody(int indexObject)
{
    std::unique_lock<std::mutex> lock(m_mutexObjects);

    std::map<int, Object*>::iterator it = m_mapObjects.find(indexObject);

    if (it == m_mapObjects.end()) {
        LOG_FAILURE("Failed to find object %d", indexObject);
        return ERR_INVALID;
    }

    it->second->setIsLocked(false, 0);

    return NO_ERR;
}

Errors Kinematics::getObjectFrame(int indexObject, Matrix4d &xfm)
{
    std::unique_lock<std::mutex> lock(m_mutexObjects);
    std::map<int, Object*>::iterator it = m_mapObjects.find(indexObject);

    if (it == m_mapObjects.end()) {
        LOG_FAILURE("Failed to find object %d", indexObject);
        return ERR_INVALID;
    }

    xfm = it->second->getXfm();
    return NO_ERR;
}

Errors Kinematics::setObjectFrame(int indexObject, const Matrix4d &xfm)
{
    std::unique_lock<std::mutex> lock(m_mutexObjects);
    std::map<int, Object*>::iterator it = m_mapObjects.find(indexObject);

    if (it == m_mapObjects.end()) {
        LOG_FAILURE("Failed to find object %d", indexObject);
        return ERR_INVALID;
    }

    it->second->setXfm(xfm);
    return NO_ERR;
}

Errors Kinematics::calculateObjectsXfm()
{
    std::unique_lock<std::mutex> lock(m_mutexObjects);

    for (auto pair: m_mapObjects) {
        int indexRigidBody = 0;
        bool isLocked = pair.second->getIsLocked(indexRigidBody);
        if (isLocked) {
            Node* node = m_cp->getNodeOfRigidBody(indexRigidBody);
            if (nullptr == node) {
                LOG_FAILURE("Failed to find rigid body %d",
                        indexRigidBody);
                return ERR_INVALID;
            }
            pair.second->setXfm(node->getXfm() * pair.second->getXfmObjectToRb());
        }
    }

    return NO_ERR;
}

Errors Kinematics::getRigidBodyFrame(
        int indexRigidBody, int indexFrame, Matrix4d &xfm)
{
    Node* node = m_cp->getNodeOfRigidBody(indexRigidBody);
    if (node == nullptr) {
        LOG_FAILURE("Frame %d does not exist in rigid body %d",
                indexFrame, indexRigidBody);
        return ERR_INVALID;
    }

    return node->getFrame(indexFrame, xfm);
}

Node* Kinematics::getRoot()
{
    return m_root;
}

ThreadQueue<Camera_t>* Kinematics::getCameraDataQueue()
{
    return &m_cameraDataQueue;
}

unsigned int Kinematics::getCounter()
{
    std::unique_lock<std::mutex> lock(m_mutexCounter);
    unsigned int counter = m_counter;
    return counter;
}

void Kinematics::setCounter(unsigned int counter)
{
    std::unique_lock<std::mutex> lock(m_mutexCounter);
    m_counter = counter;
}

void Kinematics::incCounter()
{
    std::unique_lock<std::mutex> lock(m_mutexCounter);
    m_counter++;
}


Errors Kinematics::getJointValues(std::map<int, double> &jointValues)
{
    if (NO_ERR != getNodeJointValue(m_root, jointValues)) {
        LOG_FAILURE("Failed to get joint values");
        return ERR_INVALID;
    }
    return NO_ERR;
}

Errors Kinematics::getNodeJointValue(
        Node* node, std::map<int, double> &jointValues)
{
    if (node != m_root) {
        jointValues.insert(std::pair<int, double>(
            node->getMateToParent()->index(), node->getCurrentJointValue()));
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != getNodeJointValue(node->getChildren().at(i), jointValues)) {
            LOG_FAILURE("Failed to get joint value for node %d",
                    node->getChildren().at(i));
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

std::map<int, Object*> Kinematics::getObjects()
{
    return m_mapObjects;
}

Errors Kinematics::installTool()
{
    m_tool = m_cp->getTool();
    return NO_ERR;
}

} // end of namespace tarsim
