/**
 * @file: sceneFaults.cpp
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
#include "sceneFaults.h"
#include "gui.h"
#include "logClient.h"
#include "vtkRegularPolygonSource.h"
#include "vtkPolyDataMapper2D.h"
#include "vtkAlgorithm.h"
#include "vtkAlgorithmOutput.h"
#include "vtkProperty2D.h"
#include <algorithm>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
SceneFaults::SceneFaults(Gui* gui):
    SceneBase(gui, gui->getWin()->faults_scene())
{
    if (createCamera() != NO_ERR) {
        throw std::invalid_argument("Failed to create camera");
    }

    if (addActorsToScene() != NO_ERR) {
        throw std::invalid_argument("Failed to create actors");
    }

    if (m_gui->getWin()->status_message_display_duration() > 1.0e-3) {
        m_updateDuration = m_gui->getWin()->status_message_display_duration();
    } else {
        m_updateDuration = 5.0;
    }

    strncpy(m_statusMessage.statusMessage, c_initText, sizeof(m_statusMessage.statusMessage));
    m_statusMessage.statusMessage[MAX_STATUS_TEXT_SIZE - 1] = '\0';
}

Errors SceneFaults::update(bool dimsChanged)
{
    std::chrono::high_resolution_clock::time_point t =
        std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span =
        std::chrono::duration_cast<std::chrono::duration<double>>(t - m_prevUpdateTime);
    double duration = time_span.count();

    if (duration > m_updateDuration || m_gui->getFrameNumber() == 0) {
        m_prevUpdateTime = t;

        m_statusMessage = m_gui->getHighestPriorityStatusMessage();

        if (FAULT_LEVEL_NOFAULT == m_statusMessage.faultLevel) {
            m_actorCircle->SetVisibility(false);
            m_actorFault->SetVisibility(false);
        } else {
            m_actorCircle->SetVisibility(true);
            m_actorFault->SetVisibility(true);

            if (NO_ERR != updateActorCircle(m_statusMessage)) {
                LOG_FAILURE("Failed to update circle actor");
                return ERR_INVALID;
            }

            if (NO_ERR != updateActorFault(m_statusMessage)) {
                LOG_FAILURE("Failed to update fault actor");
                return ERR_INVALID;
            }
        }
    }

    return NO_ERR;
}

Errors SceneFaults::addActorsToScene()
{
    if (NO_ERR != addActorCircleToScene()) {
        LOG_FAILURE("Failed to add circle actor to the scene");
        return ERR_INVALID;
    }
    if (NO_ERR != addActorFaultToScene()) {
        LOG_FAILURE("Failed to add fault actor to scene");
        return ERR_INVALID;
    }
    return NO_ERR;
}

Errors SceneFaults::addActorCircleToScene()
{
    // Create a circle
    vtkSmartPointer<vtkRegularPolygonSource> polygonSource =
          vtkSmartPointer<vtkRegularPolygonSource>::New();

    polygonSource->SetNumberOfSides(50);
    polygonSource->SetCenter(0, 0, 0);

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper2D> mapper =
          vtkSmartPointer<vtkPolyDataMapper2D>::New();
    mapper->SetInputConnection(polygonSource->GetOutputPort());

    m_actorCircle = vtkSmartPointer<vtkActor2D>::New();
    m_actorCircle->SetMapper(mapper);
    m_renderer->AddActor(m_actorCircle);
    return NO_ERR;
}

Errors SceneFaults::updateActorCircle(const GuiStatusMessage_t &fault)
{
    int* dim = m_renderer->GetSize();
    int w = dim[0];
    int h = dim[1];

    vtkSmartPointer<vtkAlgorithm> algorithm =
            m_actorCircle->GetMapper()->GetInputConnection(0, 0)->GetProducer();
    vtkSmartPointer<vtkRegularPolygonSource> srcReference =
            vtkRegularPolygonSource::SafeDownCast(algorithm);

    srcReference->SetRadius(h/2 * 0.8);
    srcReference->SetCenter(h/2, h/2, 0);

    switch (fault.faultLevel) {
    case FAULT_LEVEL_NOFAULT:
        m_actorCircle->GetProperty()->SetColor(0.0, 0.6, 0.2);
        break;
    case FAULT_LEVEL_INFO:
        m_actorCircle->GetProperty()->SetColor(0.0, 0.5, 0.2);
        break;
    case FAULT_LEVEL_WARNING:
        m_actorCircle->GetProperty()->SetColor(1.0, 1.0, 0.0);
        break;
    case FAULT_LEVEL_MINOR:
        m_actorCircle->GetProperty()->SetColor(0.8, 0.4, 0.4);
        break;
    case FAULT_LEVEL_MAJOR:
        m_actorCircle->GetProperty()->SetColor(1.0, 0.2, 0.2);
        break;
    case FAULT_LEVEL_CRITICAL:
        m_actorCircle->GetProperty()->SetColor(1.0, 0.1, 0.1);
        break;
    default:
        m_actorCircle->GetProperty()->SetColor(0.0, 0.1, 0.0);
    }

    return NO_ERR;
}

Errors SceneFaults::addActorFaultToScene()
{
    m_actorFault = vtkSmartPointer<vtkTextActor>::New();
    m_actorFault->SetInput(m_statusMessage.statusMessage);

    if (m_scene.font_size() > 0) {
        m_actorFault->GetTextProperty()->
                SetFontSize(m_scene.font_size());
    }

    if (m_scene.has_font_color()) {
        m_actorFault->GetTextProperty()->SetColor(
                m_scene.font_color().r(),
                m_scene.font_color().g(),
                m_scene.font_color().b());
    }
    m_actorFault->GetTextProperty()->SetJustificationToLeft();
    m_renderer->AddActor2D(m_actorFault);

    return NO_ERR;
}

Errors SceneFaults::updateActorFault(const GuiStatusMessage_t &fault)
{
    m_actorFault->SetInput(fault.statusMessage);

    int* dim = m_renderer->GetSize();
    int w = dim[0];
    int h = dim[1];
    double bbox[4];
    m_actorFault->GetBoundingBox(m_renderer, bbox);
    m_actorFault->SetConstrainedFontSize(m_renderer, 0.9 * w - h, 0.7 * h);
    m_actorFault->SetPosition(h + 0.005 * w, 0.15 *h);
    return NO_ERR;
}
} // end of namespace tarsim
