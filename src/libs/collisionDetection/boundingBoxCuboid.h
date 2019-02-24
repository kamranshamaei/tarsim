/**
 *
 * @file: boundingBoxCuboid.h
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
#ifndef BOUNDING_BOX_CUBOID_H
#define BOUNDING_BOX_CUBOID_H

//INCLUDES
#include "boundingBoxBase.h"

namespace tarsim {
// FORWARD DECLARATIONS

// TYPEDEFS AND DEFINES

// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class BoundingBoxCuboid: public BoundingBoxBase
{
public:
    // FUNCTIONS
    BoundingBoxCuboid(
        double collisionDetectionDistance,
        std::vector<Vector4d> vertices);
    virtual ~BoundingBoxCuboid() = default;

    // MEMBERS
protected:
    // FUNCTIONS
    // MEMBERS

};
} // end of namespace tarsim
// ENDIF
#endif /* BOUNDING_BOX_CUBOID_H */
