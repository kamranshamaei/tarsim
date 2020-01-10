/**
 *
 * @file: sceneRobot.h
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
#ifndef SCENE_ROBOT_H
#define SCENE_ROBOT_H

//INCLUDES
#include "sceneBase.h"
#include "node.h"
#include "object.h"
#include "vtkAxesActor.h"
#include "vtkTextActor.h"
#include "vtkCellArray.h"

namespace tarsim {
// FORWARD DECLARATIONS
class Gui;
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class SceneRobot: public SceneBase
{
public:
    // FUNCTIONS
    SceneRobot(Gui* gui, Node* root);
    virtual ~SceneRobot() = default;

    Errors update(bool dimsChanged = false) override;
    virtual Errors installTool(Object* tool) override;
    virtual Errors removeTool() override;

    // MEMBERS

private:
    // FUNCTIONS
    Errors addNodeActorsToScene(Node* node);
    Errors addObjectActorsToScene();
    Errors addActorsPlanesToScene(Node* node);
    Errors addActorsLinesToScene(Node* node);
    Errors addActorsPointsToScene(Node* node);
    Errors addActorCadToScene(Node* node);
    Errors addActorFramesToScene(Node* node);
    Errors updateTreeActors(Node* node);
    Errors updateFrameVisibility(Node* node, bool frameVisibility);
    Errors updatePlaneVisibility(Node* node, bool planeVisibility);
    Errors updateLinesVisibility(Node* node, bool linesVisibility);
    Errors updatePointsVisibility(Node* node, bool pointsVisibility);
    Errors updateCadVisibility(Node* node, bool cadVisibility);
    Errors addWorldFrameActorToScene();
    Errors addPlaneActorsToScene();
    Errors createCamera(const Camera &camera);
    Errors updateCamera() override;
    Errors addActorCompanyNameToScene();
    Errors updateActorCompanyName();
    Errors addActorEndEffectorPositionToScene();
    Errors updateActorEndEffectorPosition();
    Errors addActorObjectsToScene();
    Errors updateObjectsActors();

    // MEMBERS
    Node* m_root = nullptr;
    Node* m_endEffectorNode = nullptr;
    unsigned int m_endEffectorFrameNumber = 0;
    std::vector<vtkSmartPointer<vtkAxesActor>> m_axes;
    vtkSmartPointer<vtkTextActor> m_actorClientName;

    vtkSmartPointer<vtkActor> m_actorPath;
    vtkSmartPointer<vtkPoints> m_pointsPath;
    unsigned int m_kinCounter = 0;

    unsigned int m_currentPointOnPath = 0;
    unsigned int m_numPointsOnPath = 0;

    bool m_frameVisibility = true;
    bool m_planeVisibility = true;
    bool m_pointsVisibility = true;
    bool m_linesVisibility = true;
    bool m_cadVisibility = true;
    Camera m_cameraConfig;
    Object* m_tool = nullptr;

    std::map<int, vtkSmartPointer<vtkActor>> m_selfCollisionActors;
};
} // end of namespace tarsim
// ENDIF
#endif /* SCENE_ROBOT_H */
