/**
 * @file: sceneSpeed.cpp
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
#include "sceneSpeed.h"
#include "gui.h"
#include "logClient.h"
#include "vtkSliderRepresentation2D.h"
#include "vtkProperty2D.h"
#include "vtkPNGReader.h"
#include "vtkImageResize.h"
#include "fileSystem.h"
#include <vtkImageMapper.h>
#include <vtkProperty2D.h>
#include <vtkTexturedButtonRepresentation2D.h>
#include <vtkImageData.h>
#include <vtkImageResize.h>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
SceneSpeed::SceneSpeed(Gui* gui):
    SceneBase(gui, gui->getWin()->speed_scene())
{
    if (createCamera() != NO_ERR) {
        throw std::invalid_argument("Failed to create camera");
    }

    if (addActorsToScene() != NO_ERR) {
        throw std::invalid_argument("Failed to create actors");
    }
}

Errors SceneSpeed::update(bool dimsChanged)
{
    if (NO_ERR != updateSliderActors(dimsChanged)) {
        LOG_FAILURE("Failed to update position of slider actors");
        return ERR_INVALID;
    }

    return NO_ERR;
}

Errors SceneSpeed::addActorsToScene()
{
    if (NO_ERR != addActorsSlidersToScene()) {
        LOG_FAILURE("Failed to add slider actors to the scene");
        return ERR_INVALID;
    }
    return NO_ERR;
}


Errors SceneSpeed::addActorsSlidersToScene()
{
    m_renderer->WorldToView();

    // CREATE SLIDERS
    vtkSmartPointer<vtkSliderRepresentation2D> sliderRep =
            vtkSmartPointer<vtkSliderRepresentation2D>::New();

    sliderRep->SetTitleText("Speed");
    sliderRep->ShowSliderLabelOn();

    // Set min and max of slider
    sliderRep->SetMinimumValue(0);
    sliderRep->SetMaximumValue(100);

    if (m_gui->getWin()->speed_scene().has_slider_color()) {
        sliderRep->GetSliderProperty()->SetColor(
                m_gui->getWin()->speed_scene().slider_color().r(),
                m_gui->getWin()->speed_scene().slider_color().g(),
                m_gui->getWin()->speed_scene().slider_color().b());
        sliderRep->GetLabelProperty()->SetColor(
                m_gui->getWin()->speed_scene().slider_color().r(),
                m_gui->getWin()->speed_scene().slider_color().g(),
                m_gui->getWin()->speed_scene().slider_color().b());
    } else {
        sliderRep->GetSliderProperty()->SetColor(0.9, 0.5, 0.5);
        sliderRep->GetLabelProperty()->SetColor(0.9, 0.5, 0.5);
    }

    if (m_gui->getWin()->speed_scene().has_tube_color()) {
        sliderRep->GetTubeProperty()->SetColor(
                m_gui->getWin()->speed_scene().tube_color().r(),
                m_gui->getWin()->speed_scene().tube_color().g(),
                m_gui->getWin()->speed_scene().tube_color().b());

        sliderRep->GetTitleProperty()->SetColor(
                m_gui->getWin()->speed_scene().tube_color().r(),
                m_gui->getWin()->speed_scene().tube_color().g(),
                m_gui->getWin()->speed_scene().tube_color().b());

        sliderRep->GetCapProperty()->SetColor(
                m_gui->getWin()->speed_scene().tube_color().r(),
                m_gui->getWin()->speed_scene().tube_color().g(),
                m_gui->getWin()->speed_scene().tube_color().b());
    } else {
        sliderRep->GetTubeProperty()->SetColor(1.0, 1.0, 1.0);
        sliderRep->GetTitleProperty()->SetColor(0.9, 0.9, 0.9);
        sliderRep->GetCapProperty()->SetColor(1.0, 1.0, 1.0);
    }

    sliderRep->SetValue(m_speed);

    sliderRep->GetPoint1Coordinate()->SetCoordinateSystemToDisplay();
    sliderRep->GetPoint1Coordinate()->SetValue(0, 10);
    sliderRep->GetPoint2Coordinate()->SetCoordinateSystemToDisplay();
    sliderRep->GetPoint2Coordinate()->SetValue(100, 10);


    sliderRep->SetSliderWidth(0.05);
    sliderRep->SetSliderLength(0.05);
    sliderRep->SetEndCapLength(0.005);
    sliderRep->SetEndCapWidth(0.01);
    sliderRep->SetTubeWidth(0.005);

    sliderRep->GetLabelProperty()->SetVerticalJustificationToTop();
    sliderRep->GetTitleProperty()->SetVerticalJustificationToBottom();

    vtkSmartPointer<vtkSliderWidget> sliderWidget =
            vtkSmartPointer<vtkSliderWidget>::New();

    sliderWidget->SetCurrentRenderer(m_renderer);
    sliderWidget->SetRepresentation(sliderRep);
    sliderWidget->SetInteractor(m_gui->getRenderWindowInteractor());
    sliderWidget->SetAnimationModeToAnimate();
    sliderWidget->EnabledOn();
    sliderRep->SetTitleHeight(0.075);
    sliderRep->SetLabelHeight(0.075);

    sliderWidget->AddObserver(vtkCommand::InteractionEvent, this,
            &SceneSpeed::setSpeed);

    m_slider = sliderWidget;

    return NO_ERR;
}

Errors SceneSpeed::updateSliderActors(bool dimsChanged)
{
    if (!dimsChanged) {
        return NO_ERR;
    }

    int* org = m_renderer->GetOrigin();
    int* dim = m_renderer->GetSize();
    double w = (double)dim[0];
    double h = (double)dim[1];
    double x = (double)org[0];
    double y = (double)org[1];
    double w_border = 0.05 * w;
    double h_border = 0.05 * h;

    // Update slider dimensions
    vtkSmartPointer<vtkSliderRepresentation2D> sliderRep =
            vtkSliderRepresentation2D::SafeDownCast(
                    m_slider->GetRepresentation());

    sliderRep->GetPoint1Coordinate()->SetValue(w_border, h/2.0);
    sliderRep->GetPoint2Coordinate()->SetValue(w - w_border, h/2.0);


    sliderRep->SetSliderWidth(0.12);
    sliderRep->SetSliderLength(0.08);
    sliderRep->SetEndCapLength(0.04);
    sliderRep->SetEndCapWidth(0.08);
    sliderRep->SetTubeWidth(0.04);


    sliderRep->Modified();

    return NO_ERR;
}

void SceneSpeed::setSpeed(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    vtkSmartPointer<vtkSliderRepresentation2D> sliderRep =
            vtkSliderRepresentation2D::SafeDownCast(
                    m_slider->GetRepresentation());

    m_speed = sliderRep->GetValue();
    m_gui->getEitOsMsgServerReceiver()->sendSpeed(m_speed);
}
} // end of namespace tarsim
