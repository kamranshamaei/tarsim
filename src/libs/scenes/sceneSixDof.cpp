/**
 * @file: sceneSixDof.cpp
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
#include "sceneSixDof.h"
#include "gui.h"
#include "logClient.h"
#include "vtkCoordinate.h"
#include <cmath>
#include "vtkPNGReader.h"
#include "vtkImageResize.h"
#include "fileSystem.h"
#include <vtkImageMapper.h>
#include <vtkProperty2D.h>



namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
SceneSixDof::SceneSixDof(Gui* gui):
    SceneBase(gui, gui->getWin()->six_dof_scene())
{
    if (gui == nullptr) {
        throw std::invalid_argument("No gui was provided");
    }

    if (NO_ERR != createBackgroundImage()) {
        throw std::invalid_argument("Failed to create background image");
    }

    if (createButtons() != NO_ERR) {
        throw std::invalid_argument("Failed to create buttons representation");
    }

}

Errors SceneSixDof::update(bool dimsChanged)
{
    if (dimsChanged) {
        int* dim = m_renderer->GetSize();
        double w = (double)dim[0];
        double h = (double)dim[1];

        vtkSmartPointer<vtkImageResize> resize =
                vtkSmartPointer<vtkImageResize>::New();

        resize->SetResizeMethodToOutputDimensions();
        resize->SetInputData(m_backgroundImage);
        resize->SetOutputDimensions(dim[0], dim[1], 1);
        resize->Update();


        vtkSmartPointer<vtkAlgorithm> algorithm =
          m_backgroundImageActor->GetMapper();
        vtkSmartPointer<vtkImageMapper> mapper =
          vtkImageMapper::SafeDownCast(algorithm);

        mapper->SetInputData(resize->GetOutput());
        mapper->SetColorWindow(255);
        mapper->SetColorLevel(127.5);
        mapper->Update();
        m_backgroundImageActor->SetPosition(0.0, 0.0);
        m_backgroundImageActor->SetPosition2(w, h);


        for (int i = 0; i < m_gui->getConfigParser()->getWin()->six_dof_scene().buttons_size(); i++) {
            SIM::Button button =
                m_gui->getConfigParser()->getWin()->six_dof_scene().buttons(i);

            if (SixDofButtonsNames.end() == SixDofButtonsNames.find(button.name())) {
                continue;
            }
            SixDofButtons index = SixDofButtonsNames.at(button.name());

            vtkSmartPointer<vtkTexturedButtonRepresentation2D> br =
                    vtkTexturedButtonRepresentation2D::SafeDownCast(
                        m_buttons[index]->GetRepresentation());

            double x_min = button.view_port().x_min();
            double x_max = button.view_port().x_max();
            double y_min = button.view_port().y_min();
            double y_max = button.view_port().y_max();
            //Update button widgets position
            double bds[6];
            bds[0] = x_min * w;
            bds[1] = x_max * w;
            bds[2] = y_min * h;
            bds[3] = y_max * h;
            bds[4] = bds[5] = 0.0;


            vtkSmartPointer<vtkImageResize> bresize =
                    vtkSmartPointer<vtkImageResize>::New();

            bresize->SetResizeMethodToOutputDimensions();
            bresize->SetInputData(m_buttonImages[index]);
            bresize->SetOutputDimensions((int)((x_max - x_min) * w), (int)((y_max - y_min) * h), 1);
            bresize->Update();
            br->SetButtonTexture(0, bresize->GetOutput());
            br->Modified();

            br->PlaceWidget(bds);
        }
    }

    return NO_ERR;
}

Errors SceneSixDof::createButtons()
{
    for (int i = 0 ; i < m_gui->getConfigParser()->getWin()->six_dof_scene().buttons_size(); i++) {
        SIM::Button button =
            m_gui->getConfigParser()->getWin()->six_dof_scene().buttons(i);

        if (SixDofButtonsNames.end() == SixDofButtonsNames.find(button.name())) {
            continue;
        }

        SixDofButtons index = SixDofButtonsNames.at(button.name());
        vtkSmartPointer<vtkPNGReader> reader =
                        vtkSmartPointer<vtkPNGReader>::New();

        std::string path = "";
        std::string dummyString = "";
        if (!FileSystem::splitFilename(
            FileSystem::getexepath(), path, dummyString)) {
        }

        std::string file = path + "icons/" + "dofButton.png";
        if (button.on_icon_name().size() > 0) {
            file = path + "icons/" + button.on_icon_name();
        }

        reader->SetFileName(file.c_str());
        reader->Update();

        m_buttonImages[index] = reader->GetOutput();


        vtkSmartPointer<vtkTexturedButtonRepresentation2D> buttonRepresentation =
                vtkSmartPointer<vtkTexturedButtonRepresentation2D>::New();

        buttonRepresentation->SetNumberOfStates(1);
        buttonRepresentation->GetHoveringProperty()->SetOpacity(button.opacity());
        buttonRepresentation->GetSelectingProperty()->SetOpacity(button.opacity());
        buttonRepresentation->GetProperty()->SetOpacity(button.opacity());
        buttonRepresentation->SetButtonTexture(0, m_buttonImages[index]);


        m_buttons[index] = vtkSmartPointer<IncCmdButton>::New();
        m_buttons[index]->SetInteractor(m_gui->getRenderWindowInteractor());

        // Change bindings.
        switch (index) {
            case SixDofButtons::X_POSITIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::xPositive);
                break;
            case SixDofButtons::X_NEGATIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::xNegative);
                break;
            case SixDofButtons::Y_POSITIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::yPositive);
                break;
            case SixDofButtons::Y_NEGATIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::yNegative);
                break;
            case SixDofButtons::Z_POSITIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::zPositive);
                break;
            case SixDofButtons::Z_NEGATIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::zNegative);
                break;

            case SixDofButtons::ROLL_POSITIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::rollPositive);
                break;
            case SixDofButtons::ROLL_NEGATIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::rollNegative);
                break;
            case SixDofButtons::PITCH_POSITIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::pitchPositive);
                break;
            case SixDofButtons::PITCH_NEGATIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::pitchNegative);
                break;
            case SixDofButtons::YAW_POSITIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::yawPositive);
                break;
            case SixDofButtons::YAW_NEGATIVE:
                m_buttons[index]->AddObserver(
                        vtkCommand::StateChangedEvent, this, &SceneSixDof::yawNegative);
                break;
            default:
                LOG_WARNING("Undefined event");
        }

        m_buttons[index]->SetRepresentation(buttonRepresentation);
        m_buttons[index]->SetCurrentRenderer(m_renderer);

        buttonRepresentation->SetPlaceFactor(1);

        double bds[6];
        bds[0] = 0.0;
        bds[1] = 50.0;
        bds[2] = 0.0;
        bds[3] = 50.0;
        bds[4] = bds[5] = 0.0;
        buttonRepresentation->PlaceWidget(bds);
        m_buttons[index]->On();
    }

    return NO_ERR;
}


Errors SceneSixDof::createBackgroundImage()
{
    int* dim = m_renderer->GetSize();
    int w = dim[0];
    int h = dim[1];

    vtkSmartPointer<vtkPNGReader> reader =
                    vtkSmartPointer<vtkPNGReader>::New();

    std::string path = "";
    std::string dummyString = "";
    if (!FileSystem::splitFilename(
        FileSystem::getexepath(), path, dummyString)) {
    }

    std::string file =
        path + "icons/" + m_backgroundImageFileName;

    reader->SetFileName(file.c_str());
    reader->Update();

    m_backgroundImage = reader->GetOutput();

    vtkSmartPointer<vtkImageResize> resize =
            vtkSmartPointer<vtkImageResize>::New();

    resize->SetResizeMethodToOutputDimensions();
    resize->SetInputData(m_backgroundImage);
    resize->SetOutputDimensions(w, h, 1);
    resize->Update();

    vtkSmartPointer<vtkImageMapper> mapper =
            vtkSmartPointer<vtkImageMapper>::New();
    mapper->SetInputData(resize->GetOutput());
    mapper->Update();

    m_backgroundImageActor->SetMapper(mapper);
    m_backgroundImageActor->SetPosition(0.0, 0.0);
    m_renderer->AddActor(m_backgroundImageActor);
    return NO_ERR;
}

void SceneSixDof::xPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::X_POSITIVE);
}

void SceneSixDof::xNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::X_NEGATIVE);
}

void SceneSixDof::yPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::Y_POSITIVE);
}

void SceneSixDof::yNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::Y_NEGATIVE);
}

void SceneSixDof::zPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::Z_POSITIVE);
}

void SceneSixDof::zNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::Z_NEGATIVE);
}

void SceneSixDof::rollPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::ROLL_POSITIVE);
}

void SceneSixDof::rollNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::ROLL_NEGATIVE);
}

void SceneSixDof::pitchPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::PITCH_POSITIVE);
}

void SceneSixDof::pitchNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::PITCH_NEGATIVE);
}

void SceneSixDof::yawPositive(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::YAW_POSITIVE);
}

void SceneSixDof::yawNegative(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    toggleIncCmd(SixDofButtons::YAW_NEGATIVE);
}

void SceneSixDof::toggleIncCmd(SixDofButtons dofButton)
{
    int32_t dof = (int32_t)dofButton;
    if (dof == m_incCmd) {
        m_incCmd = -1;
    } else {
        m_incCmd = dof;
    }
    m_gui->getEitOsMsgServerReceiver()->sendIncrementalCommand(m_incCmd);
}

} // end of namespace tarsim
