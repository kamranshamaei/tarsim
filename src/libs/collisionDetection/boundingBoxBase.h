/**
 *
 * @file: boundingBoxBase.h
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
 *
 */

// IFNDEF
#ifndef BOUNDING_BOX_BASE_H
#define BOUNDING_BOX_BASE_H

//INCLUDES
#include "eitErrors.h"
#include "rbs.pb.h"

#include <Eigen>

namespace tarsim {
// FORWARD DECLARATIONS

// TYPEDEFS AND DEFINES

// ENUMS
enum class BoundingBoxType
{
    UNKNOWN,
    CAPSULE,
    SPHERE,
    CUBOID,
};
// NAMESPACES AND STRUCTS
using namespace Eigen;
using namespace SIM;
// CLASS DEFINITION
class BoundingBoxBase
{
public:
    // FUNCTIONS
    BoundingBoxBase(
            BoundingBoxType type,
            double collisionDetectionDistance,
            std::vector<Vector4d> vertices);
    virtual ~BoundingBoxBase() = default;

    // MEMBERS
    BoundingBoxType getType();
    std::vector<Vector4d>* getVertices();
    double getCollisionDetectionDistance();
    void updateVertices(const Matrix4d &m);
protected:
    // FUNCTIONS
    // MEMBERS
    BoundingBoxType m_type = BoundingBoxType::UNKNOWN;
    std::vector<Vector4d> m_globalVertices;
    std::vector<Vector4d> m_localVertices;

    // Collision detection distance
    double m_collisionDetectionDistance = 0.0;
};
} // end of namespace tarsim
// ENDIF
#endif /* BOUNDING_BOX_BASE_H */
