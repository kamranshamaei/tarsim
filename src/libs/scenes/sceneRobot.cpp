/**
 * @file: sceneRobot.cpp
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
#include "sceneRobot.h"
#include "gui.h"
#include "logClient.h"
#include "vtkTransform.h"
#include "vtkProperty.h"
#include "vtkCaptionActor2D.h"
#include "vtkTextProperty.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"
#include "vtkPlaneSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyData.h"
#include "vtkPolyLine.h"
#include "fileSystem.h"
#include "vtkSTLReader.h"
#include "vtkSTLReader.h"

#include <vtkIntersectionPolyDataFilter.h>
#include <vtkAlgorithmOutput.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
SceneRobot::SceneRobot(Gui* gui, Node* root):
    SceneBase(gui, gui->getWin()->robot_scene())
{
    if (root == nullptr) {
        throw std::invalid_argument("No tree root was provided");
    }

    m_root = root;

    if (createCamera(gui->getRbs()->camera()) != NO_ERR) {
        throw std::invalid_argument("Failed to create camera");
    }

    if (addNodeActorsToScene(root) != NO_ERR) {
        throw std::invalid_argument("Failed to create actors");
    }

    if (addObjectActorsToScene() != NO_ERR) {
        throw std::invalid_argument("Failed to create actors");
    }

    if (addWorldFrameActorToScene() != NO_ERR) {
        throw std::invalid_argument("Failed to create world frame actor");
    }

    if (addPlaneActorsToScene() != NO_ERR) {
        throw std::invalid_argument("Failed to create plane actors");
    }

    if (addActorCompanyNameToScene() != NO_ERR) {
        throw std::invalid_argument("Failed to create company name actor");
    }

    if (addActorEndEffectorPositionToScene() != NO_ERR) {
        throw std::invalid_argument(
                "Failed to add end effector position actor to the scene");
    }
}

Errors SceneRobot::update(bool dimsChanged)
{
    // Update frames visibility
    bool frameVisibility = m_gui->getFramesVisibility();
    if (m_frameVisibility != frameVisibility) {
        m_frameVisibility = frameVisibility;
        for (unsigned int i = 0; i < m_axes.size(); i++) {
            m_axes[i]->SetVisibility(m_frameVisibility);
        }

        if (NO_ERR != updateFrameVisibility(m_root, m_frameVisibility)) {
            LOG_FAILURE("Failed to update tree frame visibility");
            return ERR_INVALID;
        }

        std::map<int, Object*> objects = m_gui->getKinematics()->getObjects();

        for (auto pair: objects) {
            if (NO_ERR != updateFrameVisibility(pair.second, m_frameVisibility)) {
                LOG_FAILURE("Failed to update object frame visibility");
                return ERR_INVALID;
            }
        }

        if (m_tool) {
          if (NO_ERR != updateFrameVisibility(m_tool, m_frameVisibility)) {
              LOG_FAILURE("Failed to update object frame visibility");
              return ERR_INVALID;
          }
        }
    }

    // Update planes visibility
    bool planeVisibility = m_gui->getPlanesVisibility();
    if (m_planeVisibility != planeVisibility) {
        m_planeVisibility = planeVisibility;

        if (NO_ERR != updatePlaneVisibility(m_root, m_planeVisibility)) {
            LOG_FAILURE("Failed to update tree plane visibility");
            return ERR_INVALID;
        }

        std::map<int, Object*> objects = m_gui->getKinematics()->getObjects();

        for (auto pair: objects) {
            if (NO_ERR != updatePlaneVisibility(pair.second, m_planeVisibility)) {
                LOG_FAILURE("Failed to update object plane visibility");
                return ERR_INVALID;
            }
        }

        if (m_tool) {
          if (NO_ERR != updatePlaneVisibility(m_tool, m_planeVisibility)) {
              LOG_FAILURE("Failed to update object plane visibility");
              return ERR_INVALID;
          }
        }
    }

    // Update lines visibility
    bool linesVisibility = m_gui->getLinesVisibility();
    if (m_linesVisibility != linesVisibility) {
        m_linesVisibility = linesVisibility;
        if (NO_ERR != updateLinesVisibility(m_root, m_linesVisibility)) {
            LOG_FAILURE("Failed to update tree lines visibility");
            return ERR_INVALID;
        }

        std::map<int, Object*> objects = m_gui->getKinematics()->getObjects();

        for (auto pair: objects) {
            if (NO_ERR != updateLinesVisibility(pair.second, m_linesVisibility)) {
                LOG_FAILURE("Failed to update object line visibility");
                return ERR_INVALID;
            }
        }

        if (m_tool) {
          if (NO_ERR != updateLinesVisibility(m_tool, m_linesVisibility)) {
              LOG_FAILURE("Failed to update object line visibility");
              return ERR_INVALID;
          }
        }
    }

    // Update points visibility
    bool pointsVisibility = m_gui->getPointsVisibility();
    if (m_pointsVisibility != pointsVisibility) {
        m_pointsVisibility = pointsVisibility;
        if (NO_ERR != updatePointsVisibility(m_root, m_pointsVisibility)) {
            LOG_FAILURE("Failed to update tree points visibility");
            return ERR_INVALID;
        }

        std::map<int, Object*> objects = m_gui->getKinematics()->getObjects();

        for (auto pair: objects) {
            if (NO_ERR != updatePointsVisibility(pair.second, m_pointsVisibility)) {
                LOG_FAILURE("Failed to update object point visibility");
                return ERR_INVALID;
            }
        }

        if (m_tool) {
          if (NO_ERR != updatePointsVisibility(m_tool, m_pointsVisibility)) {
              LOG_FAILURE("Failed to update object point visibility");
              return ERR_INVALID;
          }
        }
    }

    // Update cad visibility
    bool cadVisibility = m_gui->getCadVisibility();
    if (m_cadVisibility != cadVisibility) {
        m_cadVisibility = cadVisibility;
        if (NO_ERR != updateCadVisibility(m_root, m_cadVisibility)) {
            LOG_FAILURE("Failed to update tree cad visibility");
            return ERR_INVALID;
        }

        std::map<int, Object*> objects = m_gui->getKinematics()->getObjects();

        for (auto pair: objects) {
            if (NO_ERR != updateCadVisibility(pair.second, m_cadVisibility)) {
                LOG_FAILURE("Failed to update object CAD visibility");
                return ERR_INVALID;
            }
        }

        if (m_tool) {
          if (NO_ERR != updateCadVisibility(m_tool, m_cadVisibility)) {
              LOG_FAILURE("Failed to update object cad visibility");
              return ERR_INVALID;
          }
        }
    }

    if (dimsChanged) {
        if (NO_ERR != updateActorCompanyName()) {
            LOG_FAILURE("Failed to update company name actors");
            return ERR_INVALID;
        }
    }

    unsigned int kinCounter = m_gui->getKinematics()->getCounter();
    if (m_kinCounter != kinCounter) {
        if (NO_ERR != updateTreeActors(m_root)) {
            LOG_FAILURE("Failed to update tree actors");
            return ERR_INVALID;
        }

        if (m_tool) {
            if (NO_ERR != m_tool->getActorsRigidBody()->updateNodeActors()) {
                LOG_FAILURE("Failed to update actors for tool");
                return ERR_INVALID;
            }
        }

        if (NO_ERR != updateObjectsActors()) {
            LOG_FAILURE("Failed to update object actors");
            return ERR_INVALID;
        }

        // Update path visibility
        if (NO_ERR != updateActorEndEffectorPosition()) {
            LOG_FAILURE("Failed to update end effector position actor");
            return ERR_INVALID;
        }
    }


    m_kinCounter = kinCounter;
    return NO_ERR;
}

Errors SceneRobot::updateTreeActors(Node* node)
{
    if (NO_ERR != node->getActorsRigidBody()->updateNodeActors()) {
        LOG_FAILURE("Failed to update actors for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != updateTreeActors(node->getChildren().at(i))) {
            LOG_FAILURE("Failed to update actors");
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

Errors SceneRobot::updateFrameVisibility(Node* node, bool frameVisibility)
{
    if (NO_ERR != node->getActorsRigidBody()->updateFrameVisibility(frameVisibility)) {
        LOG_FAILURE("Failed to update frame visibility for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != updateFrameVisibility(node->getChildren().at(i), frameVisibility)) {
            LOG_FAILURE("Failed to update frame visibility");
        }
    }
    return NO_ERR;
}

Errors SceneRobot::updatePlaneVisibility(Node* node, bool planeVisibility)
{
    if (NO_ERR != node->getActorsRigidBody()->updatePlaneVisibility(planeVisibility)) {
        LOG_FAILURE("Failed to update frame visibility for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != updatePlaneVisibility(node->getChildren().at(i), planeVisibility)) {
            LOG_FAILURE("Failed to update frame visibility");
        }
    }
    return NO_ERR;
}

Errors SceneRobot::updateLinesVisibility(Node* node, bool linesVisibility)
{
    if (NO_ERR != node->getActorsRigidBody()->updateLinesVisibility(linesVisibility)) {
        LOG_FAILURE("Failed to update lines visibility for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != updateLinesVisibility(node->getChildren().at(i), linesVisibility)) {
            LOG_FAILURE("Failed to update lines visibility");
        }
    }
    return NO_ERR;
}

Errors SceneRobot::updatePointsVisibility(Node* node, bool pointsVisibility)
{
    if (NO_ERR != node->getActorsRigidBody()->updatePointsVisibility(pointsVisibility)) {
        LOG_FAILURE("Failed to update points visibility for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != updatePointsVisibility(node->getChildren().at(i), pointsVisibility)) {
            LOG_FAILURE("Failed to update points visibility");
        }
    }
    return NO_ERR;
}

Errors SceneRobot::updateCadVisibility(Node* node, bool cadVisibility)
{
    if (NO_ERR != node->getActorsRigidBody()->updateCadVisibility(cadVisibility)) {
        LOG_FAILURE("Failed to update points visibility for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != updateCadVisibility(node->getChildren().at(i), cadVisibility)) {
            LOG_FAILURE("Failed to update points visibility");
        }
    }
    return NO_ERR;
}

Errors SceneRobot::addNodeActorsToScene(Node* node)
{
    if (NO_ERR != addActorsPlanesToScene(node)) {
        LOG_FAILURE("Failed to add planes actors to the scene for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    if (NO_ERR != addActorsLinesToScene(node)) {
        LOG_FAILURE("Failed to add lines actors to the scene for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    if (NO_ERR != addActorsPointsToScene(node)) {
        LOG_FAILURE("Failed to add points actors to the scene for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    if (NO_ERR != addActorCadToScene(node)) {
        LOG_FAILURE("Failed to add cad actor to the scene for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    if (NO_ERR != addActorFramesToScene(node)) {
        LOG_FAILURE("Failed to add frame actors to the scene for node %s",
                node->getName().c_str());
        return ERR_INVALID;
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        addNodeActorsToScene(node->getChildren().at(i));
    }

    return NO_ERR;
}

Errors SceneRobot::addObjectActorsToScene()
{
    std::map<int, Object*> objects = m_gui->getKinematics()->getObjects();
    for (auto pair: objects) {
        Object* object = pair.second;

        if (NO_ERR != addActorsPlanesToScene(object)) {
            LOG_FAILURE("Failed to add planes actors to the scene for object %s",
                    object->getName().c_str());
            return ERR_INVALID;
        }

        if (NO_ERR != addActorsLinesToScene(object)) {
            LOG_FAILURE("Failed to add lines actors to the scene for object %s",
                    object->getName().c_str());
            return ERR_INVALID;
        }

        if (NO_ERR != addActorsPointsToScene(object)) {
            LOG_FAILURE("Failed to add points actors to the scene for object %s",
                    object->getName().c_str());
            return ERR_INVALID;
        }

        if (NO_ERR != addActorCadToScene(object)) {
            LOG_FAILURE("Failed to add cad actor to the scene for object %s",
                    object->getName().c_str());
            return ERR_INVALID;
        }

        if (NO_ERR != addActorFramesToScene(object)) {
            LOG_FAILURE("Failed to add frame actors to the scene for object %s",
                    object->getName().c_str());
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

Errors SceneRobot::addActorsPlanesToScene(Node* node)
{
    for (unsigned int i = 0;
            i < node->getActorsRigidBody()->getActorsPlanes().size(); i++) {
        m_renderer->AddActor(node->getActorsRigidBody()->getActorsPlanes().at(i));
    }
    return NO_ERR;
}

Errors SceneRobot::addActorsLinesToScene(Node* node)
{
    for (unsigned int i = 0;
            i < node->getActorsRigidBody()->getActorsLines().size(); i++) {
        m_renderer->AddActor(node->getActorsRigidBody()->getActorsLines().at(i));
    }
    return NO_ERR;
}

Errors SceneRobot::addActorsPointsToScene(Node* node)
{
    for (unsigned int i = 0;
            i < node->getActorsRigidBody()->getActorsPoints().size(); i++) {
        m_renderer->AddActor(node->getActorsRigidBody()->getActorsPoints().at(i));
    }
    return NO_ERR;
}

Errors SceneRobot::addActorCadToScene(Node* node)
{
    for (size_t i = 0; i < node->getActorsRigidBody()->getActorCad().size(); i++) {
        m_renderer->AddActor(node->getActorsRigidBody()->getActorCad()[i]);
    }
    return NO_ERR;
}

Errors SceneRobot::addActorFramesToScene(Node* node)
{
    for (unsigned int i = 0;
            i < node->getActorsRigidBody()->getActorsFrames().size(); i++) {
        m_renderer->AddActor(node->getActorsRigidBody()->getActorsFrames().at(i));
    }
    return NO_ERR;
}

Errors SceneRobot::addWorldFrameActorToScene()
{
    m_axes.resize(m_scene.world_frames_size());
    for (unsigned int i = 0; i < m_scene.world_frames_size(); i++) {
        vtkSmartPointer<vtkAxesActor> axes =
                vtkSmartPointer<vtkAxesActor>::New();
        m_axes[i] = axes;
        if (m_scene.world_frames(i).has_xfm()) {
            vtkSmartPointer<vtkTransform> transform =
                    vtkSmartPointer<vtkTransform>::New();

            Matrix4d m;
            m <<
                m_scene.world_frames(i).xfm().rxx(),
                m_scene.world_frames(i).xfm().rxy(),
                m_scene.world_frames(i).xfm().rxz(),
                m_scene.world_frames(i).xfm().tx(),

                m_scene.world_frames(i).xfm().ryx(),
                m_scene.world_frames(i).xfm().ryy(),
                m_scene.world_frames(i).xfm().ryz(),
                m_scene.world_frames(i).xfm().ty(),

                m_scene.world_frames(i).xfm().rzx(),
                m_scene.world_frames(i).xfm().rzy(),
                m_scene.world_frames(i).xfm().rzz(),
                m_scene.world_frames(i).xfm().tz(),

                0.0, 0.0, 0.0, 1.0;

            const double t[16] = {
                            m(0, 0), m(0, 1), m(0, 2), m(0, 3),
                            m(1, 0), m(1, 1), m(1, 2), m(1, 3),
                            m(2, 0), m(2, 1), m(2, 2), m(2, 3),
                                  0,       0,       0,      1};

            transform->SetMatrix(t);
            axes->SetUserTransform(transform);
        }

        axes->SetShaftTypeToCylinder();
        axes->SetNormalizedTipLength(0.3, 0.3, 0.3);
        axes->SetNormalizedShaftLength(0.7, 0.7, 0.7);



        if (m_scene.world_frames(i).length() >= 0.0) {
            axes->SetTotalLength(
                    m_scene.world_frames(i).length(),
                    m_scene.world_frames(i).length(),
                    m_scene.world_frames(i).length());
        }

        if (m_scene.world_frames(i).length() >= 0.0) {
            axes->SetCylinderRadius(m_scene.world_frames(i).radius());
            axes->SetConeRadius(m_scene.world_frames(i).radius() * 20.0);
        }

        if (m_scene.world_frames(i).font() >= 0) {
            axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
            axes->GetXAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->
                    SetFontSize(m_scene.world_frames(i).font());

            axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
            axes->GetYAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->
                    SetFontSize(m_scene.world_frames(i).font());

            axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
            axes->GetZAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->
                    SetFontSize(m_scene.world_frames(i).font());
        }

        axes->GetXAxisCaptionActor2D()->
                GetCaptionTextProperty()->SetColor(1.0, 0.0, 0.0);
        axes->GetYAxisCaptionActor2D()->
                GetCaptionTextProperty()->SetColor(0.0, 1.0, 0.0);
        axes->GetZAxisCaptionActor2D()->
                GetCaptionTextProperty()->SetColor(0.0, 0.0, 1.0);

        m_renderer->AddActor(axes);
    }

    return NO_ERR;
}


Errors SceneRobot::addPlaneActorsToScene()
{
    for (unsigned int i = 0; i < m_scene.planes_size(); i++) {
        vtkSmartPointer<vtkPlaneSource> planeSource =
                vtkSmartPointer<vtkPlaneSource>::New();

        Vector4d u = Vector4d(
                m_scene.planes(i).u().x(),
                m_scene.planes(i).u().y(),
                m_scene.planes(i).u().z(),
                0.0);

        Vector4d v = Vector4d(
                m_scene.planes(i).v().x(),
                m_scene.planes(i).v().y(),
                m_scene.planes(i).v().z(),
                0.0);

        Vector4d s = u + v;

        Vector4d c = Vector4d(
                m_scene.planes(i).center().x(),
                m_scene.planes(i).center().y(),
                m_scene.planes(i).center().z(),
                1) - s/2.0;

        planeSource->SetPoint1(c.x() + u.x(), c.y() + u.y(), c.z() + u.z());
        planeSource->SetPoint2(c.x() + v.x(), c.y() + v.y(), c.z() + v.z());
        planeSource->SetOrigin(c.x(), c.y(), c.z());
        planeSource->Update();

        vtkSmartPointer<vtkPolyDataMapper> mapper =
                vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(planeSource->GetOutputPort());

        vtkSmartPointer<vtkActor> actor =
                vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);


        if (m_scene.planes(i).has_color()) {
            actor->GetProperty()->SetColor(
                    m_scene.planes(i).color().r(),
                    m_scene.planes(i).color().g(),
                    m_scene.planes(i).color().b());
        }

        if (m_scene.planes(i).opacity() > 0) {
            actor->GetProperty()->SetOpacity(m_scene.planes(i).opacity());
        }

        m_renderer->AddActor(actor);
    }

    return NO_ERR;
}

Errors SceneRobot::createCamera(const Camera &camera)
{
    m_cameraConfig = camera;
    m_camera = vtkSmartPointer<vtkCamera>::New();

    m_camera->SetPosition(
            m_cameraConfig.position().x(),
            m_cameraConfig.position().y(),
            m_cameraConfig.position().z());

    m_camera->SetFocalPoint(
            m_cameraConfig.focal_point().x(),
            m_cameraConfig.focal_point().y(),
            m_cameraConfig.focal_point().z());

    if ((fabs(m_cameraConfig.view_up().x()) < 1e-5) &&
        (fabs(m_cameraConfig.view_up().y()) < 1e-5) &&
        (fabs(m_cameraConfig.view_up().z()) < 1e-5)) {
        m_camera->SetViewUp(0.0, 1.0, 0.0);
    } else {
        m_camera->SetViewUp(
                m_cameraConfig.view_up().x(),
                m_cameraConfig.view_up().y(),
                m_cameraConfig.view_up().z());
    }

    if (m_cameraConfig.has_clipping_range()) {
        m_camera->SetClippingRange(
                m_cameraConfig.clipping_range().min(),
                m_cameraConfig.clipping_range().max());
    } else {
        m_camera->SetClippingRange(0.01, 5000.0);
    }

    m_renderer->SetActiveCamera(m_camera);
    return NO_ERR;
}

Errors SceneRobot::updateCamera()
{
    m_camera->SetPosition(
            m_cameraConfig.position().x(),
            m_cameraConfig.position().y(),
            m_cameraConfig.position().z());

    m_camera->SetFocalPoint(
            m_cameraConfig.focal_point().x(),
            m_cameraConfig.focal_point().y(),
            m_cameraConfig.focal_point().z());

    m_camera->SetViewUp(
            m_cameraConfig.view_up().x(),
            m_cameraConfig.view_up().y(),
            m_cameraConfig.view_up().z());

    if (m_cameraConfig.has_clipping_range()) {
        m_camera->SetClippingRange(
                m_cameraConfig.clipping_range().min(),
                m_cameraConfig.clipping_range().max());
    }
    return NO_ERR;
}

Errors SceneRobot::addActorCompanyNameToScene()
{
    m_actorClientName = vtkSmartPointer<vtkTextActor>::New();
    m_actorClientName->SetInput(m_gui->getWin()->company_name().c_str());
    m_actorClientName->GetTextProperty()->ItalicOn();
    m_actorClientName->GetTextProperty()->BoldOn();
    m_actorClientName->GetTextProperty()->ShadowOn();
    m_actorClientName->GetTextProperty()->SetJustificationToRight();

    if (m_scene.font_size() > 0) {
        m_actorClientName->GetTextProperty()->SetFontSize(m_scene.font_size());
    }

    m_actorClientName->GetTextProperty()->SetColor(
        m_scene.font_color().r(),
        m_scene.font_color().g(),
        m_scene.font_color().b());

    m_renderer->AddActor2D(m_actorClientName);
    return NO_ERR;
}

Errors SceneRobot::updateActorCompanyName()
{
    int* dim = m_renderer->GetSize();
    int w = dim[0];
    int h = dim[1];

    double bbox[4];
    m_actorClientName->GetBoundingBox(m_renderer, bbox);
    m_actorClientName->SetPosition(w - bbox[1] - 0.01 * w, 0.005 * h);

    return NO_ERR;
}

Errors SceneRobot::addActorEndEffectorPositionToScene()
{
    if (m_gui->getConfigParser()->getWin()->default_number_of_path_points() > 0) {
        m_numPointsOnPath =
            m_gui->getConfigParser()->getWin()->default_number_of_path_points();
    } else {
        m_numPointsOnPath = 100;
    }

    m_endEffectorNode =
            m_gui->getConfigParser()->getEndEffectorNode();
    m_endEffectorFrameNumber =
            m_gui->getConfigParser()->getEndEffectorFrameNumber();


    Matrix4d m = Matrix4d::Identity();
    if (m_endEffectorNode) {
        if (NO_ERR != m_endEffectorNode->getFrame(m_endEffectorFrameNumber, m)) {
            LOG_FAILURE("Failed to get end effector frame");
            return ERR_INVALID;
        }
    }

    // Create the geometry of a point (the coordinate)
    m_pointsPath = vtkSmartPointer<vtkPoints>::New();

    // Create a cell array to store the lines in and add the lines to it
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    for (unsigned int i = 0; i < m_numPointsOnPath; i++) {
        vtkIdType pid[1];
        pid[0] = m_pointsPath->InsertNextPoint(m(0, 3), m(1, 3), m(2, 3));
        vertices->InsertNextCell(1, pid);
    }

    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

    // Add the points to the dataset
    polyData->SetPoints(m_pointsPath);

    // Add the lines to the dataset
    polyData->SetVerts(vertices);

    // Setup actor and mapper
    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();

    mapper->SetInputData(polyData);

    m_actorPath = vtkSmartPointer<vtkActor>::New();
    m_actorPath->SetMapper(mapper);

    if (m_gui->getConfigParser()->getWin()->path_point_size() > 0.0) {
        m_actorPath->GetProperty()->SetPointSize(
                m_gui->getConfigParser()->getWin()->path_point_size());
    } else {
        m_actorPath->GetProperty()->SetPointSize(1);
    }

    if (m_gui->getConfigParser()->getWin()->has_path_color()) {
        m_actorPath->GetProperty()->SetColor(
                m_gui->getConfigParser()->getWin()->path_color().r(),
                m_gui->getConfigParser()->getWin()->path_color().g(),
                m_gui->getConfigParser()->getWin()->path_color().b());
    } else {
        m_actorPath->GetProperty()->SetColor(1.0, 1.0, 1.0);
    }

    m_renderer->AddActor(m_actorPath);
    return NO_ERR;
}


Errors SceneRobot::updateActorEndEffectorPosition()
{

    if (m_gui->getPathVisibility()) {
        m_actorPath->VisibilityOn();
        Matrix4d m = Matrix4d::Identity();
        if (nullptr != m_endEffectorNode) {
          if (NO_ERR != m_endEffectorNode->getFrame(m_endEffectorFrameNumber, m)) {
              LOG_FAILURE("Failed to get end effector frame");
              return ERR_INVALID;
          }
        }

        unsigned int kinCounter = m_gui->getKinematics()->getCounter();
        if (m_kinCounter != kinCounter) {
            m_currentPointOnPath++;
            if (m_numPointsOnPath == m_currentPointOnPath) {
                m_currentPointOnPath = 0;
            }
            m_pointsPath->SetPoint(
                    m_currentPointOnPath, m(0, 3), m(1, 3), m(2, 3));
            m_pointsPath->Modified();
        }
    } else {
        m_currentPointOnPath = 0;
        m_actorPath->VisibilityOff();
    }
    return NO_ERR;
}


Errors SceneRobot::updateObjectsActors()
{
    std::map<int, Object*> objects = m_gui->getKinematics()->getObjects();
    for (auto pair: objects) {
        Object* object = pair.second;

        if (NO_ERR != object->getActorsRigidBody()->updateNodeActors()) {
            LOG_FAILURE("Failed to update actors for object %s",
                    object->getName().c_str());
            return ERR_INVALID;
        }
    }
    return NO_ERR;
}

Errors SceneRobot::installTool(Object* object)
{
    if (NO_ERR != addActorsPlanesToScene(object)) {
        LOG_FAILURE("Failed to add planes actors to the scene for object %s",
                object->getName().c_str());
        return ERR_INVALID;
    }

    if (NO_ERR != addActorsLinesToScene(object)) {
        LOG_FAILURE("Failed to add lines actors to the scene for object %s",
                object->getName().c_str());
        return ERR_INVALID;
    }

    if (NO_ERR != addActorsPointsToScene(object)) {
        LOG_FAILURE("Failed to add points actors to the scene for object %s",
                object->getName().c_str());
        return ERR_INVALID;
    }

    if (NO_ERR != addActorCadToScene(object)) {
        LOG_FAILURE("Failed to add cad actor to the scene for object %s",
                object->getName().c_str());
        return ERR_INVALID;
    }

    if (NO_ERR != addActorFramesToScene(object)) {
        LOG_FAILURE("Failed to add frame actors to the scene for object %s",
                object->getName().c_str());
        return ERR_INVALID;
    }

    m_tool = object;

    return NO_ERR;
}

Errors SceneRobot::removeTool()
{
    if (m_tool) {
      for (size_t i = 0; i < m_tool->getActorsRigidBody()->getActorCad().size(); i++) {
        m_tool->getActorsRigidBody()->getActorCad()[i]->VisibilityOff();
      }
      m_tool->getActorsRigidBody()->getActorCad().clear();

      for (size_t i = 0; i < m_tool->getActorsRigidBody()->getActorsFrames().size(); i++) {
        m_tool->getActorsRigidBody()->getActorsFrames()[i]->VisibilityOff();
      }
      m_tool->getActorsRigidBody()->getActorsFrames().clear();

      for (size_t i = 0; i < m_tool->getActorsRigidBody()->getActorsLines().size(); i++) {
        m_tool->getActorsRigidBody()->getActorsLines()[i]->VisibilityOff();
      }
      m_tool->getActorsRigidBody()->getActorsLines().clear();

      for (size_t i = 0; i < m_tool->getActorsRigidBody()->getActorsPlanes().size(); i++) {
        m_tool->getActorsRigidBody()->getActorsPlanes()[i]->VisibilityOff();
      }
      m_tool->getActorsRigidBody()->getActorsPlanes().clear();

      for (size_t i = 0; i < m_tool->getActorsRigidBody()->getActorsPoints().size(); i++) {
        m_tool->getActorsRigidBody()->getActorsPoints()[i]->VisibilityOff();
      }
      m_tool->getActorsRigidBody()->getActorsPoints().clear();
    }

    m_tool = nullptr;

    return NO_ERR;
}

} // end of namespace tarsim

