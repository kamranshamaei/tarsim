/**
 *
 * @file: sceneSixDof.h
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
#ifndef SCENE_SIX_DOF_H
#define SCENE_SIX_DOF_H

//INCLUDES
#include "sceneBase.h"
#include "node.h"
#include "vtkTextActor.h"
#include "vtkImageData.h"
#include "vtkTexturedButtonRepresentation2D.h"
#include <vtkActor2D.h>
#include <vtkButtonWidget.h>
#include <vtkWidgetCallbackMapper.h>
#include <vtkMutexLock.h>
#include <vtkWidgetEvent.h>
#include "incCmdButton.h"

namespace tarsim {
// FORWARD DECLARATIONS
class Gui;
// TYPEDEFS AND DEFINES

// ENUMS
enum class SixDofButtons {
    UNKNOWN = -1,
    X_POSITIVE,
    X_NEGATIVE,
    Y_POSITIVE,
    Y_NEGATIVE,
    Z_POSITIVE,
    Z_NEGATIVE,
    ROLL_POSITIVE,
    ROLL_NEGATIVE,
    PITCH_POSITIVE,
    PITCH_NEGATIVE,
    YAW_POSITIVE,
    YAW_NEGATIVE,
    NUM_BUTTONS = YAW_NEGATIVE + 1,
};

const std::map<std::string, SixDofButtons> SixDofButtonsNames = {
        {"UNKNOWN", SixDofButtons::UNKNOWN},
        {"X_POSITIVE", SixDofButtons::X_POSITIVE},
        {"X_NEGATIVE", SixDofButtons::X_NEGATIVE},
        {"Y_POSITIVE", SixDofButtons::Y_POSITIVE},
        {"Y_NEGATIVE", SixDofButtons::Y_NEGATIVE},
        {"Z_POSITIVE", SixDofButtons::Z_POSITIVE},
        {"Z_NEGATIVE", SixDofButtons::Z_NEGATIVE},
        {"ROLL_POSITIVE", SixDofButtons::ROLL_POSITIVE},
        {"ROLL_NEGATIVE", SixDofButtons::ROLL_NEGATIVE},
        {"PITCH_POSITIVE", SixDofButtons::PITCH_POSITIVE},
        {"PITCH_NEGATIVE", SixDofButtons::PITCH_NEGATIVE},
        {"YAW_POSITIVE", SixDofButtons::YAW_POSITIVE},
        {"YAW_NEGATIVE", SixDofButtons::YAW_NEGATIVE},
};

// NAMESPACES AND STRUCTS

// CLASS DEFINITION
class SceneSixDof: public SceneBase
{
public:
    // FUNCTIONS
    SceneSixDof(Gui* gui);
    virtual ~SceneSixDof() = default;

    Errors update(bool dimsChanged = false) override;
    // MEMBERS

private:
    // FUNCTIONS
    Errors createBackgroundImage();
    Errors createButtons();

    void xPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void xNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void yPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void yNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void zPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void zNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));



    void rollPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void rollNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void pitchPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void pitchNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void yawPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void yawNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void toggleIncCmd(SixDofButtons dofButton);

    // MEMBERS
    std::map<SixDofButtons, vtkSmartPointer<IncCmdButton>> m_buttons;

    vtkSmartPointer<vtkImageData> m_backgroundImage;

    vtkSmartPointer<vtkActor2D> m_backgroundImageActor =
        vtkSmartPointer<vtkActor2D>::New();

    const std::string m_backgroundImageFileName = "sixDof.png";

    std::map<SixDofButtons, vtkSmartPointer<vtkImageData>> m_buttonImages;

    int32_t m_incCmd = -1;

};
} // end of namespace tarsim

// ENDIF
#endif /* SCENE_SIX_DOF_H */
