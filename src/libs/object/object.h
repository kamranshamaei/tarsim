/**
 *
 * @file: object.h
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
#ifndef OBJECT_H
#define OBJECT_H

//INCLUDES
#include <string>
#include <vector>
#include <stdexcept>
#include <Eigen>
#include "threadQueue.h"
#include "node.h"
#include <chrono>

#include "rbs.pb.h"
#include "eitErrors.h"
#include "actorsRigidBody.h"
#include "boundingBoxCapsule.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
using namespace Eigen;
using namespace SIM;

// CLASS DEFINITION
class Object: public Node
{
public:
    // FUNCTIONS
    Object(
            const ExternalObject& externalObject,
            const std::string &configFolderName);
    virtual ~Object();

    ExternalObject getExternalObject() const;

    Errors setExternalObject(const ExternalObject& externalObject);

    bool getIsLocked(int& rb);
    void setIsLocked(bool isLocked, int indexRigidBody);

    void setXfmObjectToRb(const Matrix4d &m);
    Matrix4d getXfmObjectToRb();
    virtual RigidBodyAppearance getRigidBodyAppearance() const override;

    std::vector<BoundingBoxBase*>* getBbs();

    virtual void updateFrames(const Matrix4d &m) override;

    // MEMBERS

private:
    // FUNCTIONS
    Errors verifyExternalObject(const ExternalObject& externalObject);
    // MEMBERS
    ExternalObject m_externalObject;

    std::vector<BoundingBoxBase*> m_bbs;
    bool m_isLocked; // Whether the object was locked to a rigid body
    int m_indexRigidBody; // If locked, what rigid body it was locked to
    Matrix4d m_xfmObjectToRb; // Transformation matrix from object to rigid body frames
};
} // end of namespace tarsim
// ENDIF
#endif /* OBJECT_H */
