/**
 *
 * @file: actorsRigidBody.h
 *
 * @Created on: April 24, 2018
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
#ifndef ACTORS_RIGID_BODY_H
#define ACTORS_RIGID_BODY_H

//INCLUDES
#include "vtkSmartPointer.h"
#include "vtkActor.h"
#include <Eigen>
#include "eitErrors.h"
#include "vtkAxesActor.h"

namespace tarsim {
// FORWARD DECLARATIONS
class Node;

// TYPEDEFS AND DEFINES

// ENUMS

// NAMESPACES AND STRUCTS
using namespace Eigen;
// CLASS DEFINITION
class ActorsRigidBody
{
public:
    // FUNCTIONS
    ActorsRigidBody(Node* node);
    virtual ~ActorsRigidBody() = default;

    Errors updateNodeActors();

    std::vector<vtkSmartPointer<vtkActor>> getActorsPlanes();
    std::vector<vtkSmartPointer<vtkActor>> getActorsLines();
    std::vector<vtkSmartPointer<vtkActor>> getActorsPoints();
    std::vector<vtkSmartPointer<vtkActor>> getActorCad();
    std::vector<vtkSmartPointer<vtkAxesActor>> getActorsFrames();
    Errors updateLinesVisibility(bool linesVisibility);
    Errors updatePointsVisibility(bool pointsVisibility);
    Errors updateFrameVisibility(bool framesVisibility);
    Errors updatePlaneVisibility(bool planesVisibility);
    Errors updateCadVisibility(bool cadVisibility);

    // MEMBERS

private:
    // FUNCTIONS
    Errors createActors();
    Errors updateActors(const Matrix4d &xfm);
    Errors createNodeActors();
    Errors createPlaneActors(const Matrix4d &xfm);
    Errors updatePlaneActors(const Matrix4d &xfm);
    Errors createLineActors(const Matrix4d &xfm);
    Errors updateLineActors(const Matrix4d &xfm);
    Errors createPointActors(const Matrix4d &xfm);
    Errors updatePointActors(const Matrix4d &xfm);
    Errors createCadActors(const Matrix4d &xfm);
    Errors updateCadActors(const Matrix4d &xfm);
    Errors createFrameActors(const Matrix4d &xfmWldRb);
    Errors updateFrameActors(const Matrix4d &xfmWldRb);


    // MEMBERS
    std::vector<vtkSmartPointer<vtkActor>> m_actorsPlanes;
    std::vector<vtkSmartPointer<vtkActor>> m_actorsLines;
    std::vector<vtkSmartPointer<vtkActor>> m_actorsPoints;
    std::vector<vtkSmartPointer<vtkAxesActor>> m_actorsFrames;
    std::vector<vtkSmartPointer<vtkActor>> m_actorsCad;
    Node* m_node = nullptr;
};
} // end of namespace tarsim
// ENDIF
#endif /* ACTORS_RIGID_BODY_H */
