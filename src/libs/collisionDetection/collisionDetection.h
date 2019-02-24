/**
 *
 * @file: collisionDetection.h
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
#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

//INCLUDES
#include "boundingBoxBase.h"
#include "boundingBoxCapsule.h"
#include "boundingBoxSphere.h"
#include "boundingBoxCuboid.h"

namespace tarsim {
// FORWARD DECLARATIONS

// TYPEDEFS AND DEFINES

// ENUMS

// NAMESPACES AND STRUCTS
using namespace Eigen;
using namespace SIM;
// CLASS DEFINITION
class CollisionDetection
{
public:
    // FUNCTIONS
    CollisionDetection();
    virtual ~CollisionDetection() = default;

    // MEMBERS
    Errors check(BoundingBoxBase* bb1, BoundingBoxBase* bb2, bool &result);
protected:
    // FUNCTIONS
    double minDistanceCapsuleCapsule(
            BoundingBoxCapsule* bb1, BoundingBoxCapsule* bb2);

    double minDistanceCapsuleSphere(
            BoundingBoxCapsule* bb1, BoundingBoxSphere* bb2);

    double minDistanceCapsuleCuboid(
            BoundingBoxCapsule* bb1, BoundingBoxCuboid* bb2);

    double minDistanceSphereSphere(
            BoundingBoxSphere* bb1, BoundingBoxSphere* bb2);

    double minDistanceSphereCuboid(
            BoundingBoxSphere* bb1, BoundingBoxCuboid* bb2);

    double minDistanceCuboidCuboid(
            BoundingBoxCuboid* bb1, BoundingBoxCuboid* bb2);

    double minDistancePointLine(
            const Vector3d & p,
            const Vector3d & l0,
            const Vector3d & l1);

    double minDistancePointPlane(
            const Vector3d &p,
            const Vector3d &c,
            const Vector3d &u,
            const Vector3d &v,
            double &sign);

    // MEMBERS
    const double k_smallNumber = 1.0e-6;
};
} // end of namespace tarsim
// ENDIF
#endif /* COLLISION_DETECTION_H */
