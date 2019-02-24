/*
 *
 * @file: collisionDetection.cpp
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
#include "collisionDetection.h"
#include "logClient.h"
#include <math.h>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
CollisionDetection::CollisionDetection()
{
}

Errors CollisionDetection::check(
        BoundingBoxBase* bb1, BoundingBoxBase* bb2, bool &result)
{
    if (!bb1 || !bb1) {
        LOG_FAILURE("At least one bounding box was not provided");
        return ERR_INVALID;
    }

    if (!bb1->getVertices() || !bb2->getVertices()) {
        LOG_FAILURE("At least one bounding box does not have vertices");
        return ERR_INVALID;
    }

    double collisionDistance =
            bb1->getCollisionDetectionDistance() +
            bb2->getCollisionDetectionDistance();

    double minDistance = 1.0e9;

    if (bb1->getType() == BoundingBoxType::CAPSULE &&
        bb2->getType() == BoundingBoxType::CAPSULE) {
        minDistance = minDistanceCapsuleCapsule(
                static_cast<BoundingBoxCapsule*>(bb1),
                static_cast<BoundingBoxCapsule*>(bb2));
    } else if (
            bb1->getType() == BoundingBoxType::CAPSULE &&
            bb2->getType() == BoundingBoxType::SPHERE) {
        minDistance = minDistanceCapsuleSphere(
                static_cast<BoundingBoxCapsule*>(bb1),
                static_cast<BoundingBoxSphere*>(bb2));
    } else if (
            bb1->getType() == BoundingBoxType::SPHERE &&
            bb2->getType() == BoundingBoxType::CAPSULE) {
        minDistance = minDistanceCapsuleSphere(
                static_cast<BoundingBoxCapsule*>(bb2),
                static_cast<BoundingBoxSphere*>(bb1));
    } else if (
            bb1->getType() == BoundingBoxType::CAPSULE &&
            bb2->getType() == BoundingBoxType::CUBOID) {
        minDistance = minDistanceCapsuleCuboid(
                static_cast<BoundingBoxCapsule*>(bb1),
                static_cast<BoundingBoxCuboid*>(bb2));
    } else if (
            bb1->getType() == BoundingBoxType::CUBOID &&
            bb2->getType() == BoundingBoxType::CAPSULE) {
        minDistance = minDistanceCapsuleCuboid(
                static_cast<BoundingBoxCapsule*>(bb2),
                static_cast<BoundingBoxCuboid*>(bb1));
    } else if (
            bb1->getType() == BoundingBoxType::SPHERE &&
            bb2->getType() == BoundingBoxType::SPHERE) {
        minDistance = minDistanceSphereSphere(
                static_cast<BoundingBoxSphere*>(bb1),
                static_cast<BoundingBoxSphere*>(bb2));
    } else if (
            bb1->getType() == BoundingBoxType::SPHERE &&
            bb2->getType() == BoundingBoxType::CUBOID) {
        minDistance = minDistanceSphereCuboid(
                static_cast<BoundingBoxSphere*>(bb1),
                static_cast<BoundingBoxCuboid*>(bb2));
    } else if (
            bb1->getType() == BoundingBoxType::CUBOID &&
            bb2->getType() == BoundingBoxType::SPHERE) {
        minDistance = minDistanceSphereCuboid(
                static_cast<BoundingBoxSphere*>(bb2),
                static_cast<BoundingBoxCuboid*>(bb1));
    } else if (
            bb1->getType() == BoundingBoxType::CUBOID &&
            bb2->getType() == BoundingBoxType::CUBOID) {
        minDistance = minDistanceCuboidCuboid(
                static_cast<BoundingBoxCuboid*>(bb1),
                static_cast<BoundingBoxCuboid*>(bb2));
    }

    result = collisionDistance >= minDistance;
    return NO_ERR;
}

double CollisionDetection::minDistanceCapsuleCapsule(
        BoundingBoxCapsule* bb1, BoundingBoxCapsule* bb2)
{
    // From http://geomalgorithms.com/a07-_distance.html
    Vector4d u4 = bb1->getVertices()->at(1) - bb1->getVertices()->at(0);
    Vector4d v4 = bb2->getVertices()->at(1) - bb2->getVertices()->at(0);
    Vector4d w4 = bb1->getVertices()->at(0) - bb2->getVertices()->at(0);

    Vector3d u, v, w;
    u << u4(0), u4(1), u4(2);
    v << v4(0), v4(1), v4(2);
    w << w4(0), w4(1), w4(2);

    double a = u.dot(u);         // always >= 0
    double b = u.dot(v);
    double c = v.dot(v);         // always >= 0
    double d = u.dot(w);
    double e = v.dot(w);
    double D = a * c - b * b;        // always >= 0
    double sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    double tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < k_smallNumber) { // the lines are almost parallel
        sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    } else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    } else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (fabs(sN) < k_smallNumber ? 0.0 : sN / sD);
    tc = (fabs(tN) < k_smallNumber ? 0.0 : tN / tD);

    // get the difference of the two closest points
    Vector3d dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

    return dP.norm();   // return the closest distance
}

double CollisionDetection::minDistancePointLine(
        const Vector3d & p,
        const Vector3d & l0,
        const Vector3d & l1)
{
    // From http://geomalgorithms.com/a02-_lines.html
    Vector3d v = l1 - l0;
    Vector3d w = p - l0;

    double c1 = w.dot(v);
    if ( c1 <= 0.000001 ) {
         return (p - l0).norm();
    }

    double c2 = v.dot(v);
    if (c2 <= c1) {
         return (p - l1).norm();
    }

    double b = c1 / c2;
    Vector3d Pb = l0 + b * v;
    return (p - Pb).norm();
}

double CollisionDetection::minDistanceCapsuleSphere(
        BoundingBoxCapsule* bb1, BoundingBoxSphere* bb2)
{
    Vector3d l0(
            bb1->getVertices()->at(0)(0),
            bb1->getVertices()->at(0)(1),
            bb1->getVertices()->at(0)(2));

    Vector3d l1(
            bb1->getVertices()->at(1)(0),
            bb1->getVertices()->at(1)(1),
            bb1->getVertices()->at(1)(2));

    Vector3d p(
            bb2->getVertices()->at(0)(0),
            bb2->getVertices()->at(0)(1),
            bb2->getVertices()->at(0)(2));

    return minDistancePointLine(p, l0, l1);
}

double CollisionDetection::minDistanceSphereCuboid(
        BoundingBoxSphere* bb1, BoundingBoxCuboid* bb2)
{
    // point
    Vector3d p(
            bb1->getVertices()->at(0)(0),
            bb1->getVertices()->at(0)(1),
            bb1->getVertices()->at(0)(2));

    // plane center
    Vector3d c(
            bb2->getVertices()->at(0)(0),
            bb2->getVertices()->at(0)(1),
            bb2->getVertices()->at(0)(2));

    // plane first vector
    Vector3d u(
            bb2->getVertices()->at(1)(0),
            bb2->getVertices()->at(1)(1),
            bb2->getVertices()->at(1)(2));

    // plane second vector
    Vector3d v(
            bb2->getVertices()->at(2)(0),
            bb2->getVertices()->at(2)(1),
            bb2->getVertices()->at(2)(2));
    double sign = 0.0;
    return minDistancePointPlane(p, c, u, v, sign);
}

double CollisionDetection::minDistancePointPlane(
        const Vector3d &p,
        const Vector3d &c,
        const Vector3d &u,
        const Vector3d &v,
        double &sign)
{
    // Plane normal
    Vector3d n = u.cross(v);

    Vector3d cp = p - c;
    Vector3d pl = - cp.dot(n) * n;

    // Calculate the closest point (l) on the plane
    Vector3d l = c + cp + pl;
    Vector3d cl = l - c;

    sign = -cl.dot(n);

    // If outside, return the closest distance to the closest line segment
    if (cl.dot(u) > u.norm()) {
        // If closer to the perpendicular to u
        return minDistancePointLine(p, c + u + v, c + u - v);
    } else if (cl.dot(u) < -u.norm()) {
        // If closer to the perpendicular to u
        return minDistancePointLine(p, c - u + v, c - u - v);
    } else if (cl.dot(v) > v.norm()) {
        // If closer to the perpendicular to u
        return minDistancePointLine(p, c + v + u, c + v - u);
    } else if (cl.dot(v) < -v.norm()) {
        // If closer to the perpendicular to u
        return minDistancePointLine(p, c - v + u, c - v - u);
    } else {
        // If inside, return the distance between the closest point and the point
        return pl.norm();
    }
}

double CollisionDetection::minDistanceCapsuleCuboid(
        BoundingBoxCapsule* bb1, BoundingBoxCuboid* bb2)
{
    // plane center
    Vector3d c(
            bb2->getVertices()->at(0)(0),
            bb2->getVertices()->at(0)(1),
            bb2->getVertices()->at(0)(2));

    // plane first vector
    Vector3d u(
            bb2->getVertices()->at(1)(0),
            bb2->getVertices()->at(1)(1),
            bb2->getVertices()->at(1)(2));

    // plane second vector
    Vector3d v(
            bb2->getVertices()->at(2)(0),
            bb2->getVertices()->at(2)(1),
            bb2->getVertices()->at(2)(2));

    double minDistance = 1.0e9;
    for (size_t i = 0; i < 2; i++) {
        // point
        Vector3d p(
                bb1->getVertices()->at(i)(0),
                bb1->getVertices()->at(i)(1),
                bb1->getVertices()->at(i)(2));
        double sign = 1.0;
        double distance = minDistancePointPlane(p, c, u, v, sign);
        if (distance < minDistance) {
            minDistance = distance;
        }
    }
}

double CollisionDetection::minDistanceSphereSphere(
        BoundingBoxSphere* bb1, BoundingBoxSphere* bb2)
{
    return (bb1->getVertices()->at(0) - bb2->getVertices()->at(0)).norm();
}

double CollisionDetection::minDistanceCuboidCuboid(
        BoundingBoxCuboid* bb1, BoundingBoxCuboid* bb2)
{
    return 0.0;
}

} // end of namespace tarsim
