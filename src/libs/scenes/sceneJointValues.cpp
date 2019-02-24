/**
 * @file: sceneJointValues.cpp
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
#include "sceneJointValues.h"
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
SceneJointValues::SceneJointValues(Gui* gui, Node* root):
    SceneBase(gui, gui->getWin()->joint_values_scene())
{
    if (root == nullptr) {
        throw std::invalid_argument("No tree root was provided");
    }

    m_root = root;

    if (NO_ERR != mapTreeNodes()) {
        throw std::invalid_argument("Failed to map tree nodes");
    }

    if (createCamera() != NO_ERR) {
        throw std::invalid_argument("Failed to create camera");
    }

    if (addActorsToScene() != NO_ERR) {
        throw std::invalid_argument("Failed to create actors");
    }
}

Errors SceneJointValues::update(bool dimsChanged)
{
    if (NO_ERR != updateSliderActors(dimsChanged)) {
        LOG_FAILURE("Failed to update position of joint slider actors");
        return ERR_INVALID;
    }

    return NO_ERR;
}

Errors SceneJointValues::addActorsToScene()
{
    if (NO_ERR != addActorsSlidersToScene()) {
        LOG_FAILURE("Failed to add joint slider actors to the scene");
        return ERR_INVALID;
    }
    return NO_ERR;
}


Errors SceneJointValues::addActorsSlidersToScene()
{
    m_renderer->WorldToView();

    // Load button images
    for (int i = 0 ; i < m_gui->getConfigParser()->getWin()->joint_values_scene().buttons_size(); i++) {
        SIM::Button button =
            m_gui->getConfigParser()->getWin()->joint_values_scene().buttons(i);

        if (JointIncCmdButtonsNames.end() == JointIncCmdButtonsNames.find(button.name())) {
            continue;
        }

        JointIncCmdButtons index = JointIncCmdButtonsNames.at(button.name());
        vtkSmartPointer<vtkPNGReader> reader =
                vtkSmartPointer<vtkPNGReader>::New();

        std::string path = "";
        std::string dummyString = "";
        if (!FileSystem::splitFilename(
            FileSystem::getexepath(), path, dummyString)) {
        }

        std::string file = path + "icons/" + button.on_icon_name();

        reader->SetFileName(file.c_str());
        reader->Update();

        m_buttonImages[index] = reader->GetOutput();
    }

    for (std::map<int, Node*>::iterator it = m_mapNodes.begin();
            it != m_mapNodes.end(); ++it) {

        // CREATE SLIDERS
        vtkSmartPointer<vtkSliderRepresentation2D> sliderRep =
                vtkSmartPointer<vtkSliderRepresentation2D>::New();

        sliderRep->SetTitleText(it->second->getMateToParent()->name().c_str());
        sliderRep->ShowSliderLabelOn();

        // Set min and max of sliders
        if (it->second->getMateToParent()->is_limited()) {
            sliderRep->SetMinimumValue(it->second->getMateToParent()->min());
            sliderRep->SetMaximumValue(it->second->getMateToParent()->max());
        } else {
            if (it->second->getJointType() == Joint_JointType_REVOLUTE) {
                sliderRep->SetMinimumValue(-360.0);
                sliderRep->SetMaximumValue(360.0);
            } else {
                sliderRep->SetMinimumValue(-1000.0);
                sliderRep->SetMaximumValue(1000.0);
            }
        }

        if (m_gui->getWin()->joint_values_scene().has_slider_color()) {
            sliderRep->GetSliderProperty()->SetColor(
                    m_gui->getWin()->joint_values_scene().slider_color().r(),
                    m_gui->getWin()->joint_values_scene().slider_color().g(),
                    m_gui->getWin()->joint_values_scene().slider_color().b());
            sliderRep->GetLabelProperty()->SetColor(
                    m_gui->getWin()->joint_values_scene().slider_color().r(),
                    m_gui->getWin()->joint_values_scene().slider_color().g(),
                    m_gui->getWin()->joint_values_scene().slider_color().b());
        } else {
            sliderRep->GetSliderProperty()->SetColor(0.9, 0.5, 0.5);
            sliderRep->GetLabelProperty()->SetColor(0.9, 0.5, 0.5);
        }

        if (m_gui->getWin()->joint_values_scene().has_tube_color()) {
            sliderRep->GetTubeProperty()->SetColor(
                    m_gui->getWin()->joint_values_scene().tube_color().r(),
                    m_gui->getWin()->joint_values_scene().tube_color().g(),
                    m_gui->getWin()->joint_values_scene().tube_color().b());

            sliderRep->GetTitleProperty()->SetColor(
                    m_gui->getWin()->joint_values_scene().tube_color().r(),
                    m_gui->getWin()->joint_values_scene().tube_color().g(),
                    m_gui->getWin()->joint_values_scene().tube_color().b());

            sliderRep->GetCapProperty()->SetColor(
                    m_gui->getWin()->joint_values_scene().tube_color().r(),
                    m_gui->getWin()->joint_values_scene().tube_color().g(),
                    m_gui->getWin()->joint_values_scene().tube_color().b());
        } else {
            sliderRep->GetTubeProperty()->SetColor(1.0, 1.0, 1.0);
            sliderRep->GetTitleProperty()->SetColor(0.9, 0.9, 0.9);
            sliderRep->GetCapProperty()->SetColor(1.0, 1.0, 1.0);
        }

        sliderRep->SetValue(it->second->getCurrentJointValue());

        sliderRep->GetPoint1Coordinate()->SetCoordinateSystemToDisplay();
        sliderRep->GetPoint1Coordinate()->SetValue(0, 1);
        sliderRep->GetPoint2Coordinate()->SetCoordinateSystemToDisplay();
        sliderRep->GetPoint2Coordinate()->SetValue(0, 1);


        sliderRep->SetSliderWidth(0.01);
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
        sliderRep->SetTitleHeight(0.03);
        sliderRep->SetLabelHeight(0.03);

        sliderWidget->AddObserver(vtkCommand::InteractionEvent, this,
                &SceneJointValues::setJointValues);

        m_jointSliders[it->second->getRigidBody().index()] = sliderWidget;

        // CREATE BUTTONS
        for (int i = 0 ; i < m_gui->getConfigParser()->getWin()->joint_values_scene().buttons_size(); i++) {
            SIM::Button button =
                m_gui->getConfigParser()->getWin()->joint_values_scene().buttons(i);

            if (JointIncCmdButtonsNames.end() == JointIncCmdButtonsNames.find(button.name())) {
                continue;
            }

            JointIncCmdButtons index = JointIncCmdButtonsNames.at(button.name());

            vtkSmartPointer<vtkTexturedButtonRepresentation2D> buttonRepresentation =
                    vtkSmartPointer<vtkTexturedButtonRepresentation2D>::New();

            buttonRepresentation->SetNumberOfStates(1);
            buttonRepresentation->GetHoveringProperty()->SetOpacity(button.opacity());
            buttonRepresentation->GetSelectingProperty()->SetOpacity(button.opacity());
            buttonRepresentation->GetProperty()->SetOpacity(button.opacity());
            buttonRepresentation->SetButtonTexture(0, m_buttonImages[index]);
            buttonRepresentation->SetPlaceFactor(1);

            double bds[6];
            bds[0] = 0.0;
            bds[1] = 50.0;
            bds[2] = 0.0;
            bds[3] = 50.0;
            bds[4] = bds[5] = 0.0;
            buttonRepresentation->PlaceWidget(bds);


            if (JointIncCmdButtons::INC == index) {
                m_incButtons[it->second->getRigidBody().index()] = vtkSmartPointer<IncCmdButton>::New();
                m_incButtons[it->second->getRigidBody().index()]->SetInteractor(m_gui->getRenderWindowInteractor());
                m_incButtons[it->second->getRigidBody().index()]->SetRepresentation(buttonRepresentation);
                m_incButtons[it->second->getRigidBody().index()]->SetCurrentRenderer(m_renderer);
                m_incButtons[it->second->getRigidBody().index()]->On();
                m_incButtons[it->second->getRigidBody().index()]->setIndex(it->second->getRigidBody().index());
            } else if (JointIncCmdButtons::DEC == index) {
                m_decButtons[it->second->getRigidBody().index()] = vtkSmartPointer<IncCmdButton>::New();
                m_decButtons[it->second->getRigidBody().index()]->SetInteractor(m_gui->getRenderWindowInteractor());
                m_decButtons[it->second->getRigidBody().index()]->SetRepresentation(buttonRepresentation);
                m_decButtons[it->second->getRigidBody().index()]->SetCurrentRenderer(m_renderer);
                m_decButtons[it->second->getRigidBody().index()]->On();
                m_decButtons[it->second->getRigidBody().index()]->setIndex(it->second->getRigidBody().index());
            }

            // Change bindings.
            switch (index) {
                case JointIncCmdButtons::INC:
                    m_incButtons[it->second->getRigidBody().index()]->AddObserver(
                            vtkCommand::StateChangedEvent, this, &SceneJointValues::incJoint);
                    break;
                case JointIncCmdButtons::DEC:
                    m_decButtons[it->second->getRigidBody().index()]->AddObserver(
                            vtkCommand::StateChangedEvent, this, &SceneJointValues::decJoint);
                    break;
                default: {}
            }
        }
    }

    return NO_ERR;
}

Errors SceneJointValues::updateSliderActors(bool dimsChanged)
{
    for (std::map<int, Node*>::iterator it = m_mapNodes.begin();
            it != m_mapNodes.end(); ++it) {
        // Update slider dimensions
        vtkSmartPointer<vtkSliderRepresentation2D> sliderRep =
                vtkSliderRepresentation2D::SafeDownCast(
                        m_jointSliders[it->second->getRigidBody().index()]->
                        GetRepresentation());

        double jntValue = it->second->getCurrentJointValue();
        if (jntValue < sliderRep->GetMinimumValue()) {
            sliderRep->SetMinimumValue(jntValue);
        }

        if (jntValue > sliderRep->GetMaximumValue()) {
            sliderRep->SetMaximumValue(jntValue);
        }

        sliderRep->SetValue(jntValue);
        sliderRep->Modified();
    }

    if (!dimsChanged) {
        return NO_ERR;
    }

    int* dim = m_renderer->GetSize();
    double w = (double)dim[0];
    double h = (double)dim[1];
    double h_border = 0.125 * h;

    // Line space
    double ls = (h - h_border) / (double)m_mapNodes.size();
    // Button normal width
    double bw = 0.08;
    // Distance between button and sliders, normalized
    double dbs = 0.01;

    double n = 0.0;
    for (std::map<int, Node*>::iterator it = m_mapNodes.begin();
            it != m_mapNodes.end(); ++it) {
        // Update slider dimensions
        vtkSmartPointer<vtkSliderRepresentation2D> sliderRep =
                vtkSliderRepresentation2D::SafeDownCast(
                        m_jointSliders[it->second->getRigidBody().index()]->
                        GetRepresentation());

        sliderRep->GetPoint1Coordinate()->SetValue(
                (int)((bw + dbs) * w), (int)(h - n * ls - h_border/2.0));
        sliderRep->GetPoint2Coordinate()->SetValue(
                (int)((1.0 - bw - dbs) * w), (int)(h - n * ls - h_border/2.0));

        sliderRep->SetSliderWidth(0.06);
        sliderRep->SetSliderLength(0.05);
        sliderRep->SetEndCapLength(0.02);
        sliderRep->SetEndCapWidth(0.06);
        sliderRep->SetTubeWidth(0.02);


        sliderRep->Modified();

        // Update buttons' dimensions
        for (int i = 0; i < m_gui->getConfigParser()->getWin()->joint_values_scene().buttons_size(); i++) {
            SIM::Button button =
                m_gui->getConfigParser()->getWin()->joint_values_scene().buttons(i);

            if (JointIncCmdButtonsNames.end() == JointIncCmdButtonsNames.find(button.name())) {
                continue;
            }
            JointIncCmdButtons index = JointIncCmdButtonsNames.at(button.name());

            vtkSmartPointer<vtkTexturedButtonRepresentation2D> br;
            if (JointIncCmdButtons::INC == index) {
                br = vtkTexturedButtonRepresentation2D::SafeDownCast(
                        m_incButtons[it->second->getRigidBody().index()]->GetRepresentation());
            } else {
                br = vtkTexturedButtonRepresentation2D::SafeDownCast(
                        m_decButtons[it->second->getRigidBody().index()]->GetRepresentation());
            }

            // Resize on button icon
            vtkSmartPointer<vtkImageResize> resize =
                    vtkSmartPointer<vtkImageResize>::New();

            resize->SetResizeMethodToOutputDimensions();
            resize->SetInputData(m_buttonImages[index]);
            resize->SetOutputDimensions((int)(bw * w), (int)(bw * w), 1);
            resize->Update();
            br->SetButtonTexture(0, resize->GetOutput());
            br->Modified();

            //Update button widgets position
            double bds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            if (JointIncCmdButtons::INC == index) {
                bds[0] = (1.0 - bw) * w;
                bds[1] = w;
                bds[2] = h - n * ls - h_border/2.0 - (bw * w)/2.0;
                bds[3] = h - n * ls - h_border/2.0 + (bw * w)/2.0;
                br->PlaceWidget(bds);
            } else {
                bds[0] = 0.0;
                bds[1] = bw * w;
                bds[2] = h - n * ls - h_border/2.0 - (bw * w)/2.0;
                bds[3] = h - n * ls - h_border/2.0 + (bw * w)/2.0;
                br->PlaceWidget(bds);

            }
        }


        n += 1.0;
    }

    return NO_ERR;
}

void SceneJointValues::setJointValues(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    for (std::map<int, Node*>::iterator it = m_mapNodes.begin();
            it!=m_mapNodes.end(); ++it) {

        vtkSmartPointer<vtkSliderRepresentation2D> sliderRep =
                vtkSliderRepresentation2D::SafeDownCast(
                        m_jointSliders[it->second->getRigidBody().index()]->
                        GetRepresentation());

        it->second->setTargetJointValue(sliderRep->GetValue());
    }

    GuiStatusMessage_t msg;
    std::map<int32_t, Collision> collisions;
    if (NO_ERR != m_gui->getKinematics()->executeForwardKinematics(
            msg, collisions)) {
        LOG_FAILURE("Failed to execute forward kinematics");
    }
// keep it for debugging collisions with the sliders
//    for (auto pair: collisions) {
//        printf("Rigid body %d is in collisions with ", (int)pair.second.robotLink);
//        for (int32_t i = 0; i < pair.second.numCollisions; i++) {
//            printf("%d, ", (int)pair.second.rigidBody[i]);
//        }
//        printf("\n");
//    }
}

Errors SceneJointValues::mapTreeNodes()
{
    if (NO_ERR != mapNode(m_root)) {
        LOG_FAILURE("Failed to add joint values actor to the scene");
        return ERR_INVALID;
    }

    return NO_ERR;
}

Errors SceneJointValues::mapNode(Node* node)
{
    if (node == nullptr) {
        LOG_FAILURE("Empty node was received");
        return ERR_INVALID;
    }

    if (node != m_root) {
        m_mapNodes.insert(std::pair<int, Node*>(
                node->getMateToParent()->index(), node));
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != mapNode(node->getChildren().at(i))) {
            LOG_FAILURE("Failed to map children mates");
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

void SceneJointValues::incJoint(
        vtkObject* object,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    IncCmdButton *button = static_cast<IncCmdButton*>(object);
    toggleIncCmd(button->getIndex(), JointIncCmdButtons::INC);
}

void SceneJointValues::decJoint(
        vtkObject* object,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    IncCmdButton *button = static_cast<IncCmdButton*>(object);
    toggleIncCmd(button->getIndex(), JointIncCmdButtons::DEC);
}

void SceneJointValues::toggleIncCmd(int32_t jntIndex, JointIncCmdButtons dofButton)
{
    int32_t dof = (int32_t)dofButton;
    if (dof == m_incCmd) {
        m_incCmd = -1;
    } else {
        m_incCmd = dof;
    }

    m_gui->getEitOsMsgServerReceiver()->sendJointIncrementalCommand(jntIndex, m_incCmd);
}

} // end of namespace tarsim
