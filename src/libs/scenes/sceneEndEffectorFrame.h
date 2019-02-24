/**
 *
 * @file: sceneEndEffectorFrame.h
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

// IFNDEF--
#ifndef SCENE_END_EFFECTOR_H
#define SCENE_END_EFFECTOR_H

//INCLUDES
#include "sceneBase.h"
#include "node.h"
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class SceneEndEffectorFrame: public SceneBase
{
public:
    // FUNCTIONS
    SceneEndEffectorFrame(Gui* gui, Node* root);
    virtual ~SceneEndEffectorFrame() = default;

    Errors update(bool dimsChanged = false) override;
    std::vector<vtkSmartPointer<vtkTextActor>> getActorsEndEffector();
    Errors positionEndEffectorActors();
    // MEMBERS

private:
    // FUNCTIONS
    Errors addActorsToScene();
    Errors addActorsEndEffectorToScene();
    Errors addActorFrameNameToScene();
    Errors updateActorsEndEffector();

    // MEMBERS
    Node* m_root = nullptr;
    Node* m_endEffectorNode = nullptr;
    unsigned int m_endEffectorFrameNumber = 0;
    vtkSmartPointer<vtkTextActor> m_actorFrameName;
    std::vector<vtkSmartPointer<vtkTextActor>> m_actorsEndEffector;
    bool m_isVisible = false;
};
} // end of namespace tarsim
// ENDIF
#endif /* SCENE_END_EFFECTOR_H */
