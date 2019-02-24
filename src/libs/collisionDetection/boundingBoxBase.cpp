/**
 * @file: boundingBoxBase.cpp
 *
 * @Created on: Jan 6, 2019
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
#include "boundingBoxBase.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
BoundingBoxBase::BoundingBoxBase(
    BoundingBoxType type,
    double collisionDetectionDistance,
    std::vector<Vector4d> vertices):
        m_type(type),
        m_collisionDetectionDistance(collisionDetectionDistance),
        m_localVertices(vertices),
        m_globalVertices(vertices)
{
}

BoundingBoxType BoundingBoxBase::getType()
{
    return m_type;
}

std::vector<Vector4d>* BoundingBoxBase::getVertices()
{
    return &m_globalVertices;
}

double BoundingBoxBase::getCollisionDetectionDistance()
{
    return m_collisionDetectionDistance;
}

void BoundingBoxBase::updateVertices(const Matrix4d &m)
{
    for (size_t i = 0; i < m_globalVertices.size(); i++) {
        m_globalVertices[i] = m * m_localVertices[i];
    }
}

} // end of namespace tarsim
