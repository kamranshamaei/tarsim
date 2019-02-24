/**
 *
 * @file: sceneBase.h
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
#ifndef SCENE_BASE_H
#define SCENE_BASE_H

//INCLUDES
#include "win.pb.h"
#include "eitErrors.h"
#include "configParser.h"
#include "kinematics.h"

#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkCamera.h"
#include <vtkCommand.h>
#include <vtkWidgetEventTranslator.h>
#include <vtkObjectFactory.h>
#include <vtkCallbackCommand.h>
#include <vtkButtonWidget.h>
#include <vtkWidgetCallbackMapper.h>
#include <vtkMutexLock.h>
#include <vtkWidgetEvent.h>

namespace tarsim {
// FORWARD DECLARATIONS
class Gui;
// TYPEDEFS AND DEFINES

// ENUMS

// NAMESPACES AND STRUCTS
using namespace SIM;
// CLASS DEFINITION
class SceneBase
{
public:
    // FUNCTIONS
    SceneBase(Gui* gui, const Scene &scene);
    virtual ~SceneBase() = default;

    virtual vtkSmartPointer<vtkRenderer> getRenderer() {return m_renderer;}
    virtual vtkSmartPointer<vtkCamera> getCamera() {return m_camera;}

    virtual Errors update(bool dimsChanged = false) {return NO_ERR;}
    virtual Errors updateCamera();
    virtual Errors updateCamera(
        float position[3],
        float focalPoint[3],
        float viewUp[3],
        float clippingRange[2]);

    // MEMBERS

protected:
    // FUNCTIONS
    virtual Errors createRenderer();
    virtual Errors createCamera();

    // MEMBERS
    // The renderer generates the image
    // which is then displayed on the render window.
    // It can be thought of as a scene to which the actor is added
    vtkSmartPointer<vtkRenderer> m_renderer = nullptr;

    vtkSmartPointer<vtkCamera> m_camera;

    Scene m_scene;

    Gui* m_gui = nullptr;
};
} // end of namespace tarsim
// ENDIF
#endif /* SCENE_BASE_H */
