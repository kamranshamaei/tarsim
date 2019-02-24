/**
 * @file: sceneBase.cpp
 *
 * @Created on: March 31, 2018
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
#include "sceneBase.h"
#include "gui.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
SceneBase::SceneBase(Gui* gui, const Scene &scene)
{
    if (gui == nullptr) {
        throw std::invalid_argument("No config was provided");
    }
    m_gui = gui;
    m_scene = scene;

    if (createRenderer() != NO_ERR) {
        throw std::invalid_argument("Failed to create renderer");
    }
}

Errors SceneBase::createRenderer()
{
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    if (m_scene.is_background_transparent()) {
        m_renderer->SetLayer(1);
    }

    m_renderer->SetViewport(
            m_scene.view_port().x_min(),
            m_scene.view_port().y_min(),
            m_scene.view_port().x_max(),
            m_scene.view_port().y_max());
    m_renderer->GradientBackgroundOn();

    m_renderer->SetBackground2(
            m_scene.up_background_color().r(),
            m_scene.up_background_color().g(),
            m_scene.up_background_color().b());
    m_renderer->SetBackground(
            m_scene.down_background_color().r(),
            m_scene.down_background_color().g(),
            m_scene.down_background_color().b());

    return NO_ERR;
}

Errors SceneBase::createCamera()
{
    m_camera = vtkSmartPointer<vtkCamera>::New();
    m_camera->SetPosition(0.0, 0.0, 1.0);

    m_camera->SetFocalPoint(0.0, 0.0, 0.0);

    m_camera->SetViewUp(0.0, 1.0, 0.0);
    m_camera->SetClippingRange(0.01, 1000.0);

    m_renderer->SetActiveCamera(m_camera);
    return NO_ERR;
}

Errors SceneBase::updateCamera()
{
    m_camera = vtkSmartPointer<vtkCamera>::New();
    m_camera->SetPosition(0.0, 0.0, 1.0);

    m_camera->SetFocalPoint(0.0, 0.0, 0.0);

    m_camera->SetViewUp(0.0, 1.0, 0.0);
    m_camera->SetClippingRange(0.01, 1000.0);
    return NO_ERR;
}

Errors SceneBase::updateCamera(
        float position[3],
        float focalPoint[3],
        float viewUp[3],
        float clippingRange[2])
{
    m_camera->SetPosition(
            position[0],
            position[1],
            position[2]);

    m_camera->SetFocalPoint(
            focalPoint[0],
            focalPoint[1],
            focalPoint[2]);

    m_camera->SetViewUp(
            viewUp[0],
            viewUp[1],
            viewUp[2]);

    m_camera->SetClippingRange(
                    clippingRange[0],
                    clippingRange[1]);
    return NO_ERR;
}
} // end of namespace tarsim
