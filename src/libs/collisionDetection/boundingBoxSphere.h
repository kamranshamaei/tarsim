/**
 *
 * @file: boundingBoxSphere.h
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
#ifndef BOUNDING_BOX_SPHERE_H
#define BOUNDING_BOX_SPHERE_H

//INCLUDES
#include "boundingBoxBase.h"

namespace tarsim {
// FORWARD DECLARATIONS

// TYPEDEFS AND DEFINES

// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class BoundingBoxSphere: public BoundingBoxBase
{
public:
    // FUNCTIONS
    BoundingBoxSphere(
        double collisionDetectionDistance,
        Vector4d vortex);
    virtual ~BoundingBoxSphere() = default;

    // MEMBERS
protected:
    // FUNCTIONS
    // MEMBERS

};
} // end of namespace tarsim
// ENDIF
#endif /* BOUNDING_BOX_SPHERE_H */
