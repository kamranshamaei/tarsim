/**
 * @file: node.cpp
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
#include "node.h"
#include <algorithm>
#include "logClient.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
Node::Node(
        Node* parent,
        const RigidBody& rigidBody,
        const Mate& mateToParent,
        bool isBase,
        const std::string &configFolderName,
        double initialValue):
        m_isBase(isBase)
{
    m_configFolderName = configFolderName;
    m_targetJointValue = initialValue;
    m_jointValuePrevious = m_targetJointValue;

    if (setRigidBody(rigidBody) != NO_ERR) {
        throw std::invalid_argument("No rigid body specified for current node");
    }

    if (setName(m_rigidBody.name()) != NO_ERR) {
        throw std::invalid_argument("No name specified for current node");
    }

    if (!m_isBase) {
        if (setParent(parent) != NO_ERR) {
            throw std::invalid_argument("No parent specified for current node");
        }
    }

    if (!m_isBase) {
        if (setMateToParent(mateToParent) != NO_ERR) {
            throw std::invalid_argument("No mate specified for current node");
        }
    }

    if (mateNode() != NO_ERR) {
    	throw std::invalid_argument("Failed to mate current node");
    }

    if (m_parent != nullptr) {
    	if (m_parent->AddChild(this) != NO_ERR) {
    		throw std::invalid_argument(
    				"Failed to add to the parent children");
    	}
    }

    // Add collision detection for lines
    for (size_t i = 0; i < m_rigidBody.appearance().lines_size(); i++) {
        double collisionDetectionDistance =
            m_rigidBody.appearance().lines(i).collision_detection_distance();
        if (collisionDetectionDistance > k_epsilon) {

            std::vector<Vector4d> vertices;
            Vector4d u, v;
            u <<
                m_rigidBody.appearance().lines(i).from().x(),
                m_rigidBody.appearance().lines(i).from().y(),
                m_rigidBody.appearance().lines(i).from().z(),
                1.0;

            v <<
                m_rigidBody.appearance().lines(i).to().x(),
                m_rigidBody.appearance().lines(i).to().y(),
                m_rigidBody.appearance().lines(i).to().z(),
                1.0;

            vertices.push_back(u);
            vertices.push_back(v);

            BoundingBoxCapsule* bb = new BoundingBoxCapsule(
                    collisionDetectionDistance, vertices);
            m_bbs.push_back(bb);
        }
    }

    // Add collision detection for points
    for (size_t i = 0; i < m_rigidBody.appearance().points_size(); i++) {
        double collisionDetectionDistance =
            m_rigidBody.appearance().points(i).collision_detection_distance();
        if (collisionDetectionDistance > k_epsilon) {

            Vector4d p(
                    m_rigidBody.appearance().points(i).location().x(),
                    m_rigidBody.appearance().points(i).location().y(),
                    m_rigidBody.appearance().points(i).location().z(),
                    1.0);


            BoundingBoxSphere* bb = new BoundingBoxSphere(
                    collisionDetectionDistance, p);
            m_bbs.push_back(bb);
        }
    }
}

Node::~Node()
{
    for (size_t i = 0; i < m_bbs.size(); i++) {
        delete m_bbs[i];
        m_bbs[i] = nullptr;
    }
    m_bbs.clear();
}

std::string Node::getName() const
{
    return m_name;
}

Node* Node::getParent() const
{
    return m_parent;
}

std::vector<Node*> Node::getChildren() const
{
    return m_children;
}

RigidBody Node::getRigidBody() const
{
    return m_rigidBody;
}

std::vector<CoordinateFrame>* Node::getCoordinateFrames()
{
    return &m_coordinateFrames;
}


RigidBodyAppearance Node::getRigidBodyAppearance() const
{
    return m_rigidBody.appearance();
}

Matrix4d Node::getXfm_jn_n() const
{
    return m_xfm_jn_n;
}

Matrix4d Node::getXfm_m_jm() const
{
    return m_xfm_m_jm;
}

Joint_JointType Node::getJointType() const
{
    return m_jointType;
}

const Mate* Node::getMateToParent() const
{
    return &m_mateToParent;
}

Errors Node::setName(const std::string &name)
{
    if (name.empty()) {
        LOG_FAILURE("Node name is empty");
        return ERR_SET;
    }
    m_name = name;
    return NO_ERR;
}

Errors Node::setParent(Node* parent)
{
    if (parent == nullptr) {
        LOG_FAILURE("Parent does not exist");
        return ERR_SET;
    }
    m_parent = parent;
    return NO_ERR;
}

Errors Node::AddChild(Node* child)
{
	if (child == nullptr) {
		LOG_FAILURE("Child cannot be added, as it does not exist");
		return ERR_SET;
	}
	m_children.push_back(child);
	return NO_ERR;
}

Errors Node::setMateToParent(const Mate& mateToParent)
{
    if ((mateToParent.index() < 0) || (mateToParent.sides_size() != 2)) {
        LOG_FAILURE("Specified mate is invalid");
        return ERR_SET;
    }
    m_mateToParent = mateToParent;
    return NO_ERR;
}

Errors Node::setRigidBody(const RigidBody& rigidBody)
{
    if (verifyRigidBody(rigidBody) != NO_ERR) {
        LOG_FAILURE("Failed to verify rigid body");
        return ERR_SET;
    }
    m_rigidBody = rigidBody;
    m_frames.resize(m_rigidBody.frames_size());
    m_coordinateFrames.resize(m_rigidBody.frames_size());
    return NO_ERR;
}

Errors Node::setXfm_jn_n(const Matrix4d &xfm)
{
    if (verifyXfm(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to verify transformation matrix");
        return ERR_SET;
    }
    m_xfm_jn_n = xfm;
    return NO_ERR;
}

Errors Node::setXfm_m_jm(const Matrix4d &xfm)
{
    if (verifyXfm(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to verify transformation matrix");
        return ERR_SET;
    }
    m_xfm_m_jm = xfm;
    return NO_ERR;
}

Errors Node::setJointType(const Joint_JointType &jointType)
{
    if ((jointType != Joint_JointType_REVOLUTE) ||
        (jointType != Joint_JointType_PRISMATIC)) {
        LOG_FAILURE("No rigid body specified");
        return ERR_SET;
    }
    m_jointType = jointType;
    return NO_ERR;
}

Matrix4d Node::getXfm() const
{
    std::unique_lock<std::mutex> lock(m_mutexXfm);
    Matrix4d m = m_xfm;
    return m;
}

void Node::setXfm(const Matrix4d &m)
{
    std::unique_lock<std::mutex> lock(m_mutexXfm);
    m_xfm = m;
}

Matrix4d Node::getTargetXfm() const
{
    std::unique_lock<std::mutex> lock(m_mutexTargetXfm);
    Matrix4d m = m_targetXfm;
    return m;
}

void Node::setTargetXfm(const Matrix4d &m)
{
    std::unique_lock<std::mutex> lock(m_mutexTargetXfm);
    m_targetXfm = m;
}

bool Node::getIsCollisionDetected() const
{
    std::unique_lock<std::mutex> lock(m_mutexIsCollisionDetected);
    bool collision = m_isCollisionDetected;
    return collision;
}

void Node::setIsCollisionDetected(bool collision)
{
    std::unique_lock<std::mutex> lock(m_mutexIsCollisionDetected);
    m_isCollisionDetected = collision;
}

Errors Node::verifyRigidBody(const RigidBody& rigidBody)
{
    if (!rigidBody.has_appearance()) {
        LOG_FAILURE("Rigid body does not have appearance");
        return ERR_INVALID;
    } else {
        if (verifyAppearance(rigidBody.appearance()) != NO_ERR) {
            LOG_FAILURE("Failed to verify rigid body appearance");
            return ERR_INVALID;
        }
    }

    if (m_isBase) {
        if (!rigidBody.has_xfm_rigid_body_to_world()) {
            LOG_FAILURE("Base rigid body does not have transformation matrix");
            return ERR_INVALID;
        }
    }

    if (rigidBody.joints_size() == 0) {
    	LOG_FAILURE("Rigid body does not have any joints");
		return ERR_INVALID;
    }

    return NO_ERR;
}

Errors Node::verifyXfm(const Matrix4d &xfm)
{
    if ((fabs(xfm(3, 0)) > k_epsilon) ||
        (fabs(xfm(3, 1)) > k_epsilon) ||
        (fabs(xfm(3, 2)) > k_epsilon) ||
        (fabs(xfm(3, 3) - 1) > k_epsilon)) {
        LOG_FAILURE("Not a transformation matrix");
        return ERR_SET;
    }

    if (fabs(xfm.determinant() - 1) > k_epsilon) {
        LOG_FAILURE("Transformation matrix determinant is not 1");
        return ERR_SET;
    }
    return NO_ERR;
}

Errors Node::verifyAppearance(const RigidBodyAppearance &appearance)
{
    if (appearance.cad_size() > 0) {
        for (size_t i = 0; i < appearance.cad_size(); i++) {
          if (verifyCad(appearance.cad(i)) != NO_ERR) {
            LOG_FAILURE("Failed to verify rigid body cad model");
            return ERR_INVALID;
          }
        }
    }

    if (!appearance.points_size() > 0) {
        for (int i = 0; i < appearance.points_size(); i++) {
            if (!appearance.points(i).has_location()) {
                LOG_FAILURE("Appearance point has no location");
                return ERR_INVALID;
            }

            if (!appearance.points(i).radius() < 0.0) {
                LOG_FAILURE("Appearance point has negative radius");
                return ERR_INVALID;
            }
        }
    }
    return NO_ERR;
}


Errors Node::verifyCad(const CadModel &cad)
{
    if (cad.path().empty()) {
        LOG_FAILURE("No path was specified for CAD model");
        return ERR_INVALID;
    } else {
        if (verifyCadFile(cad.path()) != NO_ERR) {
            LOG_FAILURE("Failed to verify CAD file");
            return ERR_INVALID;
        }
    }

    if (!cad.has_xfm_cad_to_rigid_body()) {
        LOG_FAILURE("Appearance CAD model has no transformation matrix");
        return ERR_INVALID;
    } else {
        if (verifyXfm(SimXfmToMatrix(cad.xfm_cad_to_rigid_body())) != NO_ERR) {
            LOG_FAILURE("Failed to verify CAD model transformation matrix");
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

Errors Node::verifyCadFile(const std::string &path)
{
    return NO_ERR;
}

Matrix4d Node::SimXfmToMatrix(const Xfm &xfm)
{
    Matrix4d m;
    m <<
            xfm.rxx(), xfm.rxy(), xfm.rxz(), xfm.tx(),
            xfm.ryx(), xfm.ryy(), xfm.ryz(), xfm.ty(),
            xfm.rzx(), xfm.rzy(), xfm.rzz(), xfm.tz(),
                  0.0,       0.0,       0.0,      1.0;
    return m;
}

Errors Node::mateNode()
{
	// If it is the base node, just fix it in space
	if (m_isBase) {
		Matrix4d xfmWldRb = SimXfmToMatrix(m_rigidBody.xfm_rigid_body_to_world());
	    setXfm(xfmWldRb);
	    updateFrames(xfmWldRb);
	} else {
	    // Set initial joint value
	    if (fabs(m_mateToParent.gear_ratio()) > 0.000001) {
	        m_gearRatio = m_mateToParent.gear_ratio();
	    }

	    if (m_mateToParent.sides_size() != 2) {
	        LOG_FAILURE("Number of sides is not 2 for mate %s",
                    m_mateToParent.name().c_str());
            return ERR_INVALID;
	    }

	    Joint_JointType bearingJointType = Joint_JointType_UNKNOWN;
	    Joint_JointType shaftJointType = Joint_JointType_UNKNOWN;

	    for (int i = 0; i < 2 ; i++) {
		    int bearing_index = i;
		    int shaft_index = 1 - i;

            // i = 0: The current node is the bearing and the parent node shaft
		    // i = 1: The current node is the shaft and the parent node bearing
            if ((m_mateToParent.sides(bearing_index).rigid_body_index() ==
                    m_rigidBody.index()) &&
                (m_mateToParent.sides(shaft_index).rigid_body_index() ==
                    m_parent->getRigidBody().index())) {

                // Find the associated bearing joint among the current
                // rigid body joints
                for (int i = 0; i < m_rigidBody.joints_size(); i++) {
                    Joint it = m_rigidBody.joints(i);
                    if (it.index() == m_mateToParent.sides(bearing_index).joint_index()) {
                        m_xfm_jn_n = SimXfmToMatrix(
                                it.xfm_joint_to_rigid_body()).inverse();
                        bearingJointType = it.joint_type();
                    }
                }

                // Find the associated shaft joint among the parent's
                // rigid body joints
                for (int i = 0; i < m_parent->getRigidBody().joints_size(); i++) {
                    Joint it = m_parent->getRigidBody().joints(i);
                    if (it.index() == m_mateToParent.sides(shaft_index).joint_index()) {
                        m_xfm_m_jm = SimXfmToMatrix(it.xfm_joint_to_rigid_body());
                        shaftJointType = it.joint_type();
                    }
                }
                // If we have reached here, we should not check again
                break;
            }
		}

        if ((bearingJointType == Joint_JointType_UNKNOWN) ||
            (shaftJointType   == Joint_JointType_UNKNOWN)) {
            LOG_FAILURE("Invalid joint type assigned for mate %s bearing side %d "
                "shaft side %d node %s parent %s\n",
                m_mateToParent.name().c_str(), (int)bearingJointType, (int)shaftJointType,
                getName().c_str(), m_parent->getName().c_str());
            return ERR_INVALID;
        }

        if (bearingJointType != shaftJointType) {
            LOG_FAILURE("Mate types are inconsistent for the sides of mate %s\n",
                    m_mateToParent.name().c_str());
            return ERR_INVALID;
        }

        m_jointType = bearingJointType;

        if ((m_xfm_jn_n.isZero(k_epsilon)) || (m_xfm_m_jm.isZero(k_epsilon))) {
        	LOG_FAILURE("Mate joint matrices were invalid for mate %s\n",
        	        m_mateToParent.name().c_str());
			return ERR_INVALID;
        }
    }
	return NO_ERR;

}

Errors Node::setTargetJointValue(const double &value)
{
    Errors error = NO_ERR;
    m_timePrevious = m_time;
    m_time = std::chrono::high_resolution_clock::now();

    std::unique_lock<std::mutex> lock(m_mutexTargetJointValue);
    m_jointValuePrevious = m_targetJointValue;
    m_targetJointValue = value;
    if (m_mateToParent.is_limited()) {
        if (m_targetJointValue > m_mateToParent.max()) {
            m_targetJointValue = m_mateToParent.max();
            error = ERR_JOINT_POSITION_LIMIT;
        } else if (m_targetJointValue < m_mateToParent.min()) {
            m_targetJointValue = m_mateToParent.min();
            error = ERR_JOINT_POSITION_LIMIT;
        }
    }
    std::chrono::duration<double> time_span =
            std::chrono::duration_cast<std::chrono::duration<double>>(m_time - m_timePrevious);
    double dt = 1000.0 * time_span.count();
    m_jointVelocityPrevious = m_jointVelocity;
    // Limit velocity
    m_jointVelocity = (m_targetJointValue - m_jointValuePrevious) / dt;
    if (m_mateToParent.is_velocity_limited()) {
        if (m_jointVelocity > m_mateToParent.max_velocity()) {
            m_jointVelocity = m_mateToParent.max_velocity();
            m_targetJointValue = m_jointValuePrevious + m_jointVelocity * dt;
            error = ERR_JOINT_VELOCITY_LIMIT;
        } else if (m_jointVelocity < m_mateToParent.min_velocity()) {
            m_jointVelocity = m_mateToParent.min_velocity();
            m_targetJointValue = m_jointValuePrevious + m_jointVelocity * dt;
            error = ERR_JOINT_VELOCITY_LIMIT;
        }
    }
    // Limit Acceleration
    m_jointAcceleration = (m_jointVelocity - m_jointVelocityPrevious) / dt;
    if (m_mateToParent.is_acceleration_limited()) {
        if (m_jointAcceleration > m_mateToParent.max_acceleration()) {
            m_jointAcceleration = m_mateToParent.max_acceleration();
            m_jointVelocity = m_jointVelocityPrevious + m_jointAcceleration * dt;
            m_targetJointValue = m_jointValuePrevious + m_jointVelocity * dt;
            error = ERR_JOINT_ACCELERATION_LIMIT;
        } else if (m_jointAcceleration < m_mateToParent.min_acceleration()) {
            m_jointAcceleration = m_mateToParent.min_acceleration();
            m_jointVelocity = m_jointVelocityPrevious + m_jointAcceleration * dt;
            m_targetJointValue = m_jointValuePrevious + m_jointVelocity * dt;
            error = ERR_JOINT_ACCELERATION_LIMIT;
        }
    }
    return error;
}

double Node::getTargetJointValue() const
{
    std::unique_lock<std::mutex> lock(m_mutexTargetJointValue);
    double jointValue = m_targetJointValue;
    return jointValue;
}

void Node::setCurrentJointValue(const double &value)
{
    std::unique_lock<std::mutex> lock(m_mutexCurrentJointValue);
    m_currentJointValue = value;
}

double Node::getCurrentJointValue() const
{
    std::unique_lock<std::mutex> lock(m_mutexCurrentJointValue);
    double jointValue = m_currentJointValue;
    return jointValue;
}

double Node::getGearRatio() const
{
    return m_gearRatio;
}

void Node::updateFrames(const Matrix4d &xfmWldRb)
{
    std::unique_lock<std::mutex> lock(m_mutexFrames);
    for (unsigned int i = 0; i < m_rigidBody.frames_size(); i++) {
        m_coordinateFrames[i] = m_rigidBody.frames(i);
        if (m_rigidBody.frames(i).has_xfm()) {
            Matrix4d xfmRbFr = SimXfmToMatrix(m_rigidBody.frames(i).xfm());
            m_frames[i] = xfmWldRb * xfmRbFr;
        }
    }
}

Errors Node::getFrame(unsigned int i, Matrix4d &m) const
{
    std::unique_lock<std::mutex> lock(m_mutexFrames);
    if (i >= m_frames.size()) {
        LOG_FAILURE("Index exceeded the size of frames");
        return ERR_INVALID;
    }

    m = m_frames[i];
    return NO_ERR;
}

ActorsRigidBody* Node::getActorsRigidBody()
{
    return m_actorsRigidBody;
}

Errors Node::setActorsRigidBody(ActorsRigidBody* actorsRigidBody)
{
    if (nullptr == actorsRigidBody) {
        LOG_FAILURE("No actors were specified for node %", getName().c_str());
        return ERR_INVALID;
    }
    m_actorsRigidBody = actorsRigidBody;
    return NO_ERR;
}

} // end of namespace tarsim


