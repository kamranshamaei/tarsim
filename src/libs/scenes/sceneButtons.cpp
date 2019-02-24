/**
 * @file: sceneButtons.cpp
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
#include "sceneButtons.h"
#include "gui.h"
#include "logClient.h"
#include "vtkCoordinate.h"
#include "vtkButtonWidget.h"
#include <cmath>
#include "vtkPNGReader.h"
#include "vtkImageResize.h"
#include "fileSystem.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
SceneButtons::SceneButtons(Gui* gui):
    SceneBase(gui, gui->getWin()->buttons_scene())
{
    if (gui == nullptr) {
        throw std::invalid_argument("No gui was provided");
    }

    if (createCamera() != NO_ERR) {
        throw std::invalid_argument("Failed to create camera");
    }

    if (createButtons() != NO_ERR) {
        throw std::invalid_argument("Failed to create buttons representation");
    }

}

Errors SceneButtons::update(bool dimsChanged)
{
    if (dimsChanged) {
        int* dim = m_renderer->GetSize();
        int w = dim[0];
        int h = dim[1];

        if (0 != h % 2) {
            // We prefer even size otherwise aliasing may happen
            h -= 1;
        }

        for (int i = 0; i < m_gui->getConfigParser()->getWin()->buttons_scene().buttons_size(); i++) {
            SIM::Button button =
                m_gui->getConfigParser()->getWin()->buttons_scene().buttons(i);

            if (SceneButtonNames.end() == SceneButtonNames.find(button.name())) {
                continue;
            }

            SceneButton index = SceneButtonNames.at(button.name());

            double x_min = button.view_port().x_min();
            double x_max = button.view_port().x_max();
            double y_min = button.view_port().y_min();
            double y_max = button.view_port().y_max();

            int s = (int)std::min(
                    (double)h * std::abs(y_max - y_min),
                    (double)w * std::abs(x_max - x_min));

            vtkSmartPointer<vtkTexturedButtonRepresentation2D> br =
                    vtkTexturedButtonRepresentation2D::SafeDownCast(
                        m_buttons[index]->GetRepresentation());

            // Resize on button icon
            vtkSmartPointer<vtkImageResize> onResize =
                    vtkSmartPointer<vtkImageResize>::New();

            onResize->SetResizeMethodToOutputDimensions();
            onResize->SetInputData(m_buttonBackgroundImageOn[index]);
            onResize->SetOutputDimensions(s, s, 1);
            onResize->Update();
            br->SetButtonTexture(0, onResize->GetOutput());

            vtkSmartPointer<vtkImageResize> offResize =
                    vtkSmartPointer<vtkImageResize>::New();

            // Resize off button icon
            offResize->SetResizeMethodToOutputDimensions();
            offResize->SetInputData(m_buttonBackgroundImageOff[index]);
            offResize->SetOutputDimensions(s, s, 1);
            offResize->Update();
            br->SetButtonTexture(1, offResize->GetOutput());

            br->Modified();

            //Update button widgets position
            double bds[6];
            bds[0] = x_min * (double)w;
            bds[1] = x_min * (double)w + (double)s;
            bds[2] = y_min * (double)h;
            bds[3] = y_min * (double)h + (double)s;
            bds[4] = bds[5] = 0.0;

            m_buttons[index]->GetRepresentation()->PlaceWidget(bds);
        }
    }

    return NO_ERR;
}

Errors SceneButtons::createButtons()
{
    if (NO_ERR != createButtonBackgroundImage()) {
        LOG_FAILURE("Failed to create button background");
        return ERR_INVALID;
    }

    for (int i = 0; i < m_gui->getConfigParser()->getWin()->buttons_scene().buttons_size(); i++) {
        SIM::Button button =
            m_gui->getConfigParser()->getWin()->buttons_scene().buttons(i);

        if (SceneButtonNames.end() == SceneButtonNames.find(button.name())) {
            continue;
        }

        SceneButton index = SceneButtonNames.at(button.name());

        vtkSmartPointer<vtkTexturedButtonRepresentation2D> buttonRepresentation =
                vtkSmartPointer<vtkTexturedButtonRepresentation2D>::New();
        buttonRepresentation->SetNumberOfStates(2);
        buttonRepresentation->SetButtonTexture(0, m_buttonBackgroundImageOn[index]);
        buttonRepresentation->SetButtonTexture(1, m_buttonBackgroundImageOff[index]);

        m_buttons[index] = vtkSmartPointer<vtkButtonWidget>::New();
        m_buttons[index]->SetInteractor(m_gui->getRenderWindowInteractor());

        switch (index) {
            case SceneButton::RESET_VIEW:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::resetView);
                break;
            case SceneButton::SHOW_FRAMES:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::showFrames);
                break;
            case SceneButton::SHOW_PLANES:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::showPlanes);
                break;
            case SceneButton::SHOW_LINES:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::showLines);
                break;
            case SceneButton::SHOW_POINTS:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::showPoints);
                break;
            case SceneButton::SHOW_CAD:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::showCad);
                break;
            case SceneButton::SHOW_PATH:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::showPath);
                break;
            case SceneButton::RECORD:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::startRecord);
                break;
            case SceneButton::EXIT:
                m_buttons[index]->AddObserver(
                    vtkCommand::StateChangedEvent, this, &SceneButtons::exit);
                break;
            default:
                LOG_WARNING("Undefined event");
        }


        m_buttons[index]->SetRepresentation(buttonRepresentation);
        m_buttons[index]->SetCurrentRenderer(m_renderer);

        int* dim = m_buttonBackgroundImageOn[index]->GetDimensions();
        double bds[6];
        bds[0] = dim[0] * i;
        bds[1] = dim[0] * (i + 1);
        bds[2] = 0.0;
        bds[3] = dim[1];
        bds[4] = bds[5] = 0.0;

        buttonRepresentation->SetPlaceFactor(1);
        buttonRepresentation->PlaceWidget(bds);

        m_buttons[index]->On();
    }

    return NO_ERR;
}


Errors SceneButtons::createButtonBackgroundImage()
{
    int* dim = m_renderer->GetSize();
    int w = dim[0];
    int h = dim[1];

    for (int i = 0; i < m_gui->getConfigParser()->getWin()->buttons_scene().buttons_size(); i++) {
        SIM::Button button =
            m_gui->getConfigParser()->getWin()->buttons_scene().buttons(i);

        if (SceneButtonNames.end() == SceneButtonNames.find(button.name())) {
            continue;
        }

        SceneButton index = SceneButtonNames.at(button.name());

        vtkSmartPointer<vtkPNGReader> render_0 =
                vtkSmartPointer<vtkPNGReader>::New();

        std::string path = "";
        std::string dummyString = "";
        if (!FileSystem::splitFilename(
            FileSystem::getexepath(), path, dummyString)) {
        }

        std::string file_0 = path + "/icons/";
        std::string file_1 = path + "/icons/";

        if (button.on_icon_name().size() > 0) {
            file_0 += button.on_icon_name();
        }

        if (button.off_icon_name().size() > 0) {
            file_1 += button.off_icon_name();
        }

        switch (index) {
            case SceneButton::SHOW_FRAMES:
                if (m_gui->getConfigParser()->getWin()->default_hide_frames()) {
                    file_0 = path + "/icons/" + std::string(button.off_icon_name());
                    file_1 = path + "/icons/" + std::string(button.on_icon_name());
                }
                break;
            case SceneButton::SHOW_PLANES:
                if (m_gui->getConfigParser()->getWin()->default_hide_planes()) {
                    file_0 = path + "/icons/" + std::string(button.off_icon_name());
                    file_1 = path + "/icons/" + std::string(button.on_icon_name());
                }
                break;
            case SceneButton::SHOW_LINES:
                if (m_gui->getConfigParser()->getWin()->default_hide_lines()) {
                    file_0 = path + "/icons/" + std::string(button.off_icon_name());
                    file_1 = path + "/icons/" + std::string(button.on_icon_name());
                }
                break;
            case SceneButton::SHOW_POINTS:
                if (m_gui->getConfigParser()->getWin()->default_hide_points()) {
                    file_0 = path + "/icons/" + std::string(button.off_icon_name());
                    file_1 = path + "/icons/" + std::string(button.on_icon_name());
                }
                break;
            case SceneButton::SHOW_CAD:
                if (m_gui->getConfigParser()->getWin()->default_hide_cad()) {
                    file_0 = path + "/icons/" + std::string(button.off_icon_name());
                    file_1 = path + "/icons/" + std::string(button.on_icon_name());
                }
                break;
            case SceneButton::SHOW_PATH:
                if (m_gui->getConfigParser()->getWin()->default_hide_path()) {
                    file_0 = path + "/icons/" + std::string(button.off_icon_name());
                    file_1 = path + "/icons/" + std::string(button.on_icon_name());
                }
                break;
            default:
                {}
        }
        
        render_0->SetFileName(file_0.c_str());
        render_0->Update();

        m_buttonBackgroundImageOn[index] = render_0->GetOutput();

        vtkSmartPointer<vtkPNGReader> render_1 =
                vtkSmartPointer<vtkPNGReader>::New();

        render_1->SetFileName(file_1.c_str());
        render_1->Update();

        m_buttonBackgroundImageOff[index] = render_1->GetOutput();
    }
    return NO_ERR;
}




void SceneButtons::resetView(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    if (NO_ERR != m_gui->updateView()) {
        LOG_FAILURE("Failed to update view");
    }
}

void SceneButtons::showFrames(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_gui->setFramesVisibility(!m_gui->getFramesVisibility());
}

void SceneButtons::showPlanes(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_gui->setPlanesVisibility(!m_gui->getPlanesVisibility());
}

void SceneButtons::showLines(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_gui->setLinesVisibility(!m_gui->getLinesVisibility());
}

void SceneButtons::showPoints(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_gui->setPointsVisibility(!m_gui->getPointsVisibility());
}

void SceneButtons::showCad(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_gui->setCadVisibility(!m_gui->getCadVisibility());
}

void SceneButtons::showPath(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_gui->setPathVisibility(!m_gui->getPathVisibility());
}

void SceneButtons::startRecord(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_gui->setRecordRobotScene(!m_gui->getRecordRobotScene());
}


void SceneButtons::exit(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_gui->destroy();
}
} // end of namespace tarsim
