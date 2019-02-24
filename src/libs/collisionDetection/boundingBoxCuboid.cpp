
/**
 * @file: boundingBoxCuboid.cpp
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
#include "boundingBoxCuboid.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
BoundingBoxCuboid::BoundingBoxCuboid(
    double collisionDetectionDistance,
    std::vector<Vector4d> vertices):
        BoundingBoxBase(
            BoundingBoxType::CUBOID, collisionDetectionDistance, vertices)
{
}

} // end of namespace tarsim
