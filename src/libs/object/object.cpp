/**
 * @file: object.cpp
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
#include "object.h"
#include <algorithm>
#include "logClient.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
Object::Object(
        const ExternalObject& externalObject,
        const std::string &configFolderName):
        Node(configFolderName),
        m_externalObject(externalObject)
{
    m_isLocked = false;
    m_indexRigidBody = 0;
    m_xfmObjectToRb = Matrix4d::Identity();

    Xfm xfm = externalObject.xfm_object_to_world();
    Matrix4d xfm_world_object;
    xfm_world_object <<
            xfm.rxx(), xfm.rxy(), xfm.rxz(), xfm.tx(),
            xfm.ryx(), xfm.ryy(), xfm.ryz(), xfm.ty(),
            xfm.rzx(), xfm.rzy(), xfm.rzz(), xfm.tz(),
                  0.0,       0.0,       0.0,      1.0;

    m_xfm = xfm_world_object;

    if (setExternalObject(externalObject) != NO_ERR) {
        throw std::invalid_argument("No rigid body specified for current object");
    }

    if (setName(m_externalObject.name()) != NO_ERR) {
        throw std::invalid_argument("No name specified for current object");
    }

    for (size_t i = 0; i < m_externalObject.appearance().lines_size(); i++) {
        double collisionDetectionDistance =
            m_externalObject.appearance().lines(i).collision_detection_distance();
        if (collisionDetectionDistance > 1.0e-6) {

            std::vector<Vector4d> vertices;
            Vector4d u, v;
            u <<
                m_externalObject.appearance().lines(i).from().x(),
                m_externalObject.appearance().lines(i).from().y(),
                m_externalObject.appearance().lines(i).from().z(),
                1.0;

            v <<
                m_externalObject.appearance().lines(i).to().x(),
                m_externalObject.appearance().lines(i).to().y(),
                m_externalObject.appearance().lines(i).to().z(),
                1.0;

            vertices.push_back(u);
            vertices.push_back(v);

            BoundingBoxCapsule* bb = new BoundingBoxCapsule(
                    collisionDetectionDistance, vertices);
            m_bbs.push_back(bb);
        }
    }
}

Object::~Object()
{
    for (size_t i = 0; i < m_bbs.size(); i++) {
        delete m_bbs[i];
        m_bbs[i] = nullptr;
    }
    m_bbs.clear();
}

ExternalObject Object::getExternalObject() const
{
    return m_externalObject;
}

Errors Object::setExternalObject(const ExternalObject& externalObject)
{
    if (verifyExternalObject(externalObject) != NO_ERR) {
        LOG_FAILURE("Failed to verify rigid body");
        return ERR_SET;
    }
    m_externalObject = externalObject;
    m_frames.resize(m_externalObject.frames_size());
    m_coordinateFrames.resize(m_externalObject.frames_size());

    for (size_t i = 0; i < m_externalObject.frames_size(); i++) {
        m_coordinateFrames[i] = m_externalObject.frames(i);
    }

    return NO_ERR;
}

Errors Object::verifyExternalObject(const ExternalObject& externalObject)
{
    if (!externalObject.has_appearance()) {
        LOG_FAILURE("Rigid body does not have appearance");
        return ERR_INVALID;
    } else {
        if (verifyAppearance(externalObject.appearance()) != NO_ERR) {
            LOG_FAILURE("Failed to verify rigid body appearance");
            return ERR_INVALID;
        }
    }

    if (!externalObject.has_xfm_object_to_world()) {
        LOG_FAILURE("Base rigid body does not have transformation matrix");
        return ERR_INVALID;
    }

    return NO_ERR;
}

bool Object::getIsLocked(int& rb)
{
    if (m_isLocked) {
        rb = m_indexRigidBody;
    }
    return m_isLocked;
}

void Object::setIsLocked(bool isLocked, int indexRigidBody)
{
    m_isLocked = isLocked;
    m_indexRigidBody = indexRigidBody;
}

void Object::setXfmObjectToRb(const Matrix4d &m)
{
    m_xfmObjectToRb = m;
}

Matrix4d Object::getXfmObjectToRb()
{
    return m_xfmObjectToRb;
}

RigidBodyAppearance Object::getRigidBodyAppearance() const
{
    return m_externalObject.appearance();
}

std::vector<BoundingBoxBase*>* Object::getBbs()
{
    return &m_bbs;
}

} // end of namespace tarsim


