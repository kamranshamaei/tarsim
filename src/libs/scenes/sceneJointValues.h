/**
 *
 * @file: sceneJointValues.h
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
#ifndef SCENE_JOINT_VALUES_H
#define SCENE_JOINT_VALUES_H

//INCLUDES
#include "sceneBase.h"
#include "node.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"
#include <map>
#include "vtkSliderWidget.h"
#include "incCmdButton.h"

namespace tarsim {
// FORWARD DECLARATIONS
class Gui;
// TYPEDEFS AND DEFINES
// ENUMS
enum class JointIncCmdButtons {
    DEC,
    INC,
    NUM_BUTTONS = INC + 1,
};

const std::map<std::string, JointIncCmdButtons> JointIncCmdButtonsNames = {
        {"INC", JointIncCmdButtons::INC},
        {"DEC", JointIncCmdButtons::DEC},
};

// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class SceneJointValues: public SceneBase
{
public:
    // FUNCTIONS
    SceneJointValues(Gui* gui, Node* root);
    virtual ~SceneJointValues() = default;

    Errors update(bool dimsChanged = false) override;
    // MEMBERS

private:
    // FUNCTIONS
    Errors addActorsToScene();
    Errors addActorsSlidersToScene();
    Errors mapTreeNodes();
    Errors mapNode(Node* node);
    Errors updateSliderActors(bool dimsChanged);

    void setJointValues(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void incJoint(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void decJoint(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void toggleIncCmd(int32_t jntIndex, JointIncCmdButtons dofButton);
    // MEMBERS
    Node* m_root = nullptr;
    std::map<int, Node*> m_mapNodes;

    std::map<int, vtkSmartPointer<vtkSliderWidget>> m_jointSliders;
    std::map<int, vtkSmartPointer<IncCmdButton>> m_incButtons;
    std::map<int, vtkSmartPointer<IncCmdButton>> m_decButtons;

    std::map<JointIncCmdButtons, vtkSmartPointer<vtkImageData>> m_buttonImages;
    int32_t m_incCmd = -1;

};
} // end of namespace tarsim
// ENDIF
#endif /* SCENE_JOINT_VALUES_H */
