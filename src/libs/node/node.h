/**
 *
 * @file: node.h
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

// IFNDEF
#ifndef NODE_H
#define NODE_H

//INCLUDES
#include <string>
#include <vector>
#include <stdexcept>
#include <Eigen>
#include "threadQueue.h"
#include <chrono>

#include "rbs.pb.h"
#include "eitErrors.h"
#include "actorsRigidBody.h"
#include "boundingBoxCapsule.h"
#include "boundingBoxSphere.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
using namespace Eigen;
using namespace SIM;

// CLASS DEFINITION
class Node
{
public:
    // FUNCTIONS
    Node(Node* parent,
         const RigidBody& rigidBody,
         const Mate& mateToParent,
         bool isBase,
         const std::string &configFolderName,
         double initialValue);
    virtual ~Node();

    std::string getName() const;
    Node* getParent() const;
    std::vector<Node*> getChildren() const;
    RigidBody getRigidBody() const;
    std::vector<CoordinateFrame>* getCoordinateFrames();
    virtual RigidBodyAppearance getRigidBodyAppearance() const;
    Matrix4d getXfm_jn_n() const;
    Matrix4d getXfm_m_jm() const;

    Joint_JointType getJointType() const;
    const Mate* getMateToParent() const;

    Errors setName(const std::string &name);
    Errors setParent(Node* parent);
    Errors AddChild(Node* child);
    Errors setMateToParent(const Mate& mateToParent);
    Errors setRigidBody(const RigidBody& rigidBody);
    Errors setXfm_jn_n(const Matrix4d &xfm_jn_n);
    Errors setXfm_m_jm(const Matrix4d &xfm_m_jm);
    Errors setJointType(const Joint_JointType &jointType);

    Matrix4d getXfm() const;
    void setXfm(const Matrix4d &m);

    Matrix4d getTargetXfm() const;

    void setTargetXfm(const Matrix4d &m);

    Errors setTargetJointValue(const double &value);
    double getTargetJointValue() const;

    void setCurrentJointValue(const double &value);
    double getCurrentJointValue() const;

    bool getIsCollisionDetected() const;

    void setIsCollisionDetected(bool collision);

    double getGearRatio() const;

    void updateFrames(const Matrix4d &m);
    Errors getFrame(unsigned int i, Matrix4d &m) const;

    ActorsRigidBody* getActorsRigidBody();
    Errors setActorsRigidBody(ActorsRigidBody* actorsRigidBody);
    std::string getConfigFolderName() {return m_configFolderName;}
    std::vector<int>* getLockedObjects() {return &m_lockedObjects;}

    std::vector<BoundingBoxBase*>* getBbs() {return &m_bbs;}

    // MEMBERS

protected:
    Node(const std::string &configFolderName) {m_configFolderName = configFolderName;}
    // FUNCTIONS
    Errors verifyXfm(const Matrix4d &xfm);
    Errors verifyRigidBody(const RigidBody& rigidBody);
    Errors verifyAppearance(const RigidBodyAppearance &appearance);
    Errors verifyCad(const CadModel &cad);
    Errors verifyCadFile(const std::string &path) ;
    Matrix4d SimXfmToMatrix(const Xfm &xfm);
    Errors mateNode();
    // MEMBERS
    std::string m_name {};
    Joint_JointType m_jointType = Joint_JointType_UNKNOWN;

    Mate m_mateToParent;
    Node* m_parent = nullptr;
    std::vector<Node*> m_children {};
    RigidBody m_rigidBody;
    Matrix4d m_xfm_jn_n = Matrix4d::Zero();
    Matrix4d m_xfm_m_jm = Matrix4d::Zero();
    std::vector<Matrix4d> m_frames;
    mutable std::mutex m_mutexFrames;

    mutable std::mutex m_mutexXfm;
    Matrix4d m_xfm;

    mutable std::mutex m_mutexTargetXfm;
    Matrix4d m_targetXfm;

    std::chrono::high_resolution_clock::time_point m_time =
            std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point m_timePrevious =
            std::chrono::high_resolution_clock::now();
    double m_jointVelocity = 0.0;
    double m_jointVelocityPrevious = 0.0;
    double m_jointAcceleration = 0.0;
    double m_targetJointValue = 0.0;
    double m_currentJointValue = 0.0;
    double m_jointValuePrevious = 0.0;
    double m_gearRatio = 1.0;
    mutable std::mutex m_mutexTargetJointValue;
    mutable std::mutex m_mutexCurrentJointValue;
    ActorsRigidBody* m_actorsRigidBody = nullptr;

    const double k_epsilon = 1e-12;
    bool m_isBase = false;
    std::string m_configFolderName = "";

    std::vector<int> m_lockedObjects;

    std::vector<BoundingBoxBase*> m_bbs;

    mutable std::mutex m_mutexIsCollisionDetected;
    bool m_isCollisionDetected = false;
    std::vector<CoordinateFrame> m_coordinateFrames;
};
} // end of namespace tarsim
// ENDIF
#endif /* NODE_H */
