/**
 *
 * @file: sceneButtons.h
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
#ifndef SCENE_BUTTONS_H
#define SCENE_BUTTONS_H

//INCLUDES
#include "sceneBase.h"
#include "node.h"
#include "vtkButtonWidget.h"
#include "vtkTextActor.h"
#include "vtkImageData.h"
#include "vtkTexturedButtonRepresentation2D.h"

namespace tarsim {
// FORWARD DECLARATIONS
class Gui;
// TYPEDEFS AND DEFINES

// ENUMS
enum class SceneButton {
    RESET_VIEW,
    SHOW_FRAMES,
    SHOW_PLANES,
    SHOW_LINES,
    SHOW_POINTS,
    SHOW_CAD,
    SHOW_PATH,
    RECORD,
    EXIT,
    NUM_BUTTONS = EXIT + 1,
};

const std::map<std::string, SceneButton> SceneButtonNames = {
        {"RESET_VIEW", SceneButton::RESET_VIEW},
        {"SHOW_FRAMES", SceneButton::SHOW_FRAMES},
        {"SHOW_PLANES", SceneButton::SHOW_PLANES},
        {"SHOW_LINES", SceneButton::SHOW_LINES},
        {"SHOW_POINTS", SceneButton::SHOW_POINTS},
        {"SHOW_CAD", SceneButton::SHOW_CAD},
        {"SHOW_PATH", SceneButton::SHOW_PATH},
        {"RECORD", SceneButton::RECORD},
        {"EXIT", SceneButton::EXIT}
};

// NAMESPACES AND STRUCTS

// CLASS DEFINITION
class SceneButtons: public SceneBase
{
public:
    // FUNCTIONS
    SceneButtons(Gui* gui);
    virtual ~SceneButtons() = default;

    Errors update(bool dimsChanged = false) override;
    // MEMBERS

private:
    // FUNCTIONS
    Errors createButtonBackgroundImage();
    Errors createButtons();


    void resetView(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void showFrames(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void showPlanes(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void showLines(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void showPoints(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void showCad(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void showPath(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    void startRecord(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));


    void exit(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    // MEMBERS
    std::map<SceneButton, vtkSmartPointer<vtkImageData>> m_buttonBackgroundImageOn;
    std::map<SceneButton, vtkSmartPointer<vtkImageData>> m_buttonBackgroundImageOff;
    std::map<SceneButton, vtkSmartPointer<vtkButtonWidget>> m_buttons;
};
} // end of namespace tarsim

// ENDIF
#endif /* SCENE_BUTTONS_H */
