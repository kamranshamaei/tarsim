/**
 *
 * @file: sceneSpeed.h
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
#ifndef SCENE_SPEED_H
#define SCENE_SPEED_H

//INCLUDES
#include "sceneBase.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"
#include "vtkSliderWidget.h"

namespace tarsim {
// FORWARD DECLARATIONS
class Gui;
// TYPEDEFS AND DEFINES
// ENUMS

// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class SceneSpeed: public SceneBase
{
public:
    // FUNCTIONS
    SceneSpeed(Gui* gui);
    virtual ~SceneSpeed() = default;

    Errors update(bool dimsChanged = false) override;
    // MEMBERS

private:
    // FUNCTIONS
    Errors addActorsToScene();
    Errors addActorsSlidersToScene();
    Errors updateSliderActors(bool dimsChanged);

    void setSpeed(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData));

    // MEMBERS
    vtkSmartPointer<vtkSliderWidget> m_slider;
    float m_speed = 50.0;
};
} // end of namespace tarsim
// ENDIF
#endif /* SCENE_SPEED_H */
