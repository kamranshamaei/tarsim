/**
 * @file: gui.cpp
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
#include "gui.h"
#include "node.h"
#include "object.h"
#include "configParser.h"
#include "kinematics.h"
#include "sceneBase.h"
#include "threadQueue.h"
#include "vtkCallbackCommand.h"
#include "sceneRobot.h"
#include "sceneJointValues.h"
#include "sceneEndEffectorFrame.h"
#include "sceneButtons.h"
#include "sceneFaults.h"
#include "sceneSixDof.h"
#include "sceneSpeed.h"
#include "logClient.h"
#include "vtkTextActor.h"
#include <algorithm>
#include "vtkButtonWidget.h"
#include "vtkTexturedButtonRepresentation2D.h"
#include "vtkImageData.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkWindowToImageFilter.h"
#include "vtkPNGWriter.h"
#include "fileSystem.h"
#include <chrono>
#include <ctime>
#include <time.h>
#include "win.pb.h"
#include "eitOsMsgServerReceiver.h"
#include "cmake_defs.h"


namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
/**
 * Product name
 */
const std::string k_productName = "Tarsim";

/**
 * Company name
 */
const std::string k_companyName = "Kamran Shamaei";
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
Gui::Gui(ConfigParser* cp, Kinematics* kin)
{
    if (cp == nullptr) {
        throw std::invalid_argument("No config parser was received");
    }
    m_cp = cp;

    if (kin == nullptr) {
        throw std::invalid_argument("No kinematics was received");
    }
    m_kin = kin;

    if (cp->getRoot() == nullptr) {
        throw std::invalid_argument("No tree root was provided");
    }
    m_root = cp->getRoot();

    if (cp->getWin() == nullptr) {
        throw std::invalid_argument("No window config was provided");
    }
    m_win = cp->getWin();

    if (cp->getRbs() == nullptr) {
        throw std::invalid_argument("No rbs config was provided");
    }
    m_rbs = cp->getRbs();

    m_cameraQueue = kin->getCameraDataQueue();

    if (NO_ERR != setDefaultVisibilityValues()) {
        throw std::invalid_argument("Failed to set default visibility");
    }

    m_renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();

    if (NO_ERR != createRenderWindow()) {
        throw std::invalid_argument("Failed to create render window");
    }

    if (NO_ERR != createTreeActors()) {
        throw std::invalid_argument("Failed to create tree actors");
    }

    if (NO_ERR != createObjectActors()) {
        throw std::invalid_argument("Failed to create object actors");
    }

    if (NO_ERR != populateRenderWindow()) {
        throw std::invalid_argument("Failed to populate render window");
    }

    std::string lic = getTarsimLicenseType();
    std::string ver = getTarsimVersion();


    std::string licenseStatement =
        "Welcome to " + k_productName + " " + lic + " " + ver + ". ";

    if ("Academic" == lic) {
        licenseStatement += "Any commercial use of this package is strictly prohibited";
    } else {
        licenseStatement += "This license is strictly owned by " +
                std::string(getTarsimLicenseOwnerName());
    }

    GuiStatusMessage_t msg;
    msg.faultLevel = FAULT_LEVEL_CRITICAL;
    msg.faultType = FAULT_TYPE_LICENSE;
    strncpy(msg.statusMessage, licenseStatement.c_str(), sizeof(msg.statusMessage));
    msg.statusMessage[MAX_STATUS_TEXT_SIZE - 1] = '\0';

    setStatusMessage(msg);
}

Gui::~Gui()
{
    for (unsigned int i = 0; i < m_scenes.size(); i++) {
        delete m_scenes.at(i);
        m_scenes.at(i) = nullptr;
    }
    m_scenes.clear();
}


void Gui::stop(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    destroy();
}

void Gui::destroy()
{
    m_renderWindowInteractor->RemoveObserver(
        vtkCommand::TimerEvent);
    m_updateLock->Lock();
    m_isDestroying = true;
    m_updateLock->Unlock();

    m_renderWindowInteractor->TerminateApp();
}

void Gui::update()
{
    int* size = m_renderWindow->GetSize();
    bool dimsChanged = false;
    if ((m_windowSize[0] != size[0]) || (m_windowSize[1] != size[1])) {
        m_windowSize[0] = size[0];
        m_windowSize[1] = size[1];
        dimsChanged = true;
    }

    for (unsigned int i = 0; i < m_scenes.size(); i++) {
        if (NO_ERR != m_scenes.at(i)->update(dimsChanged)) {
            LOG_FAILURE("Failed to update the scene %d", i);
            // TODO: Show as a widget
        }
    }

    if (m_cameraQueue != nullptr) {
        if (m_cameraQueue->size() > 0) {
            Camera_t c = m_cameraQueue->pop();
            m_scenes[ROBOT]->updateCamera(
                    c.position, c.focalPoint, c.viewUp, c.clippingRange);
        }
    }

    if (getRecordRobotScene()) {
        if (NO_ERR != record()) {
            LOG_FAILURE("Failed to record");
        }
    } else {
        m_isRecordingSetup = false;
    }

    m_frameNumber++;
}

void Gui::vtkUpdate(
        vtkObject*,
        long unsigned int vtkNotUsed(eventId),
        void* vtkNotUsed(callData))
{
    m_updateLock->Lock();
    if (m_isDestroying) {
        m_updateLock->Unlock();
        return;
    }

    update();
    m_renderWindow->Render();
    m_updateLock->Unlock();
}

Errors Gui::setupRecording()
{
    time_t rawtime;
    struct tm * timeinfo;
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    char buffer[256];
    strftime(buffer, sizeof(buffer), "%a %b %d %H-%M-%S %Y", timeinfo);
    std::string str(buffer);

    m_dataFolder = FileSystem::homeDirectory() + "/record" + k_productName + "/"
            + m_cp->getRbs()->name() + "/" + str + "/";

    if (!FileSystem::pathExists(m_dataFolder)) {
        FileSystem::recursiveMkDir(m_dataFolder.c_str());
    }
    return NO_ERR;
}

Errors Gui::record()
{
    if (!m_isRecordingSetup) {
        if (NO_ERR != setupRecording()) {
            LOG_FAILURE("Failed to set up recording");
            return ERR_INVALID;
        }
        m_isRecordingSetup = true;
    }

    if (m_isRecordingSetup) {
        unsigned int kinCounter = m_kin->getCounter();
        if (m_kinCounter != kinCounter) {
            m_kinCounter = kinCounter;

            if (FileSystem::pathExists(m_dataFolder)) {
                // Screenshot
                vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
                        vtkSmartPointer<vtkWindowToImageFilter>::New();
                windowToImageFilter->SetInput(
                    m_scenes[ROBOT]->getRenderer()->GetRenderWindow());
                windowToImageFilter->SetViewport(
                        m_scenes[ROBOT]->getRenderer()->GetViewport());
                windowToImageFilter->SetInputBufferTypeToRGB();
                windowToImageFilter->ReadFrontBufferOff();
                windowToImageFilter->Update();

                vtkSmartPointer<vtkPNGWriter> writer =
                        vtkSmartPointer<vtkPNGWriter>::New();

                std::string file =
                    m_dataFolder + "frame_" + std::to_string(m_frameNumber) + ".png";

                writer->SetFileName(file.c_str());
                writer->SetInputConnection(windowToImageFilter->GetOutputPort());
                writer->Write();
            }
        }
    }
    return NO_ERR;
}

Errors Gui::createRenderWindow()
{
    m_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    m_renderWindow->SetNumberOfLayers(2);

    int* screenSize = m_renderWindow->GetScreenSize();

    if (!screenSize) {
        LOG_FAILURE("No window was created");
        return ERR_INVALID;
    }

    int w = *screenSize;
    int h = *(screenSize + 1);
    if (m_win->is_full_screen()) {
        m_renderWindow->FullScreenOn();
    } else if (m_win->is_maximized()) {
        int* screenSize = m_renderWindow->GetScreenSize();
        m_renderWindow->SetSize(w, h);
    } else {
        m_renderWindow->SetSize(
            m_win->window_size().width(),
            m_win->window_size().height());

        m_renderWindow->SetPosition(
            m_win->lower_left_corner_position().x(),
            m_win->lower_left_corner_position().y());
    }

    std::string name = k_productName;
    if (m_rbs->name().size() > 0) {
        name = m_rbs->name() + " - " + name + " by " + k_companyName;
    }

    m_renderWindow->SetWindowName(name.c_str());
    return NO_ERR;
}

Errors Gui::populateRenderWindow()
{
    SceneRobot* sceneRobot = nullptr;
    SceneJointValues* sceneJointValues = nullptr;
    SceneEndEffectorFrame* sceneEndEffector = nullptr;
    SceneButtons* sceneButtons = nullptr;
    SceneFaults* sceneFaults = nullptr;
    SceneSixDof* sceneSixDof = nullptr;
    SceneSpeed* sceneSpeed = nullptr;

    try {
        sceneJointValues = new SceneJointValues(this, m_root);
        sceneEndEffector = new SceneEndEffectorFrame(this, m_root);
        sceneButtons = new SceneButtons(this);
        sceneFaults = new SceneFaults(this);
        sceneRobot = new SceneRobot(this, m_root);
        sceneSixDof = new SceneSixDof(this);
        sceneSpeed = new SceneSpeed(this);
    } catch (const std::invalid_argument& e) {
        LOG_FAILURE("Error: %s\n", e.what());
        return ERR_INVALID;
    }

    // Follow SCENES enum
    m_scenes.push_back(sceneRobot);
    m_scenes.push_back(sceneJointValues);
    m_scenes.push_back(sceneEndEffector);
    m_scenes.push_back(sceneButtons);
    m_scenes.push_back(sceneFaults);
    m_scenes.push_back(sceneSixDof);
    m_scenes.push_back(sceneSpeed);

    // Add an emty endrer to layer 0 to avoid reflections
    vtkSmartPointer<vtkRenderer> emptyRender = vtkSmartPointer<vtkRenderer>::New();
    emptyRender->SetBackground(1.0, 1.0, 1.0);
    m_renderWindow->AddRenderer(emptyRender);
    for (unsigned int i = 0; i < m_scenes.size(); i++) {
        m_renderWindow->AddRenderer(m_scenes.at(i)->getRenderer());
    }

    return NO_ERR;
}

void Gui::startRenderWindowInteractor()
{
    m_renderWindowInteractor->SetRenderWindow(m_renderWindow);
    m_renderWindowInteractor->Initialize();

    m_renderWindowInteractor->AddObserver(
            vtkCommand::TimerEvent, this, &Gui::vtkUpdate);

    if (m_win->graphics_cycle_ms() > 0) {
        m_renderWindowInteractor->CreateRepeatingTimer(m_win->graphics_cycle_ms());
    } else {
        m_renderWindowInteractor->CreateRepeatingTimer(40);
    }

    m_renderWindowInteractor->AddObserver(
            vtkCommand::ExitEvent, this, &Gui::stop);

    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();

    m_renderWindowInteractor->SetInteractorStyle(style);

    m_renderWindowInteractor->Start();
}

Errors Gui::createTreeActors()
{
    return createNodeActors(m_root);
}

Errors Gui::createNodeActors(Node* node)
{
    if (node->setActorsRigidBody(new ActorsRigidBody(node))) {
        LOG_FAILURE("Failed to create actors for node %",
                node->getName().c_str());
        return ERR_INVALID;
    }
    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != createNodeActors(node->getChildren().at(i))) {
            LOG_FAILURE("Failed to create actors for child %",
                    node->getChildren().at(i)->getName().c_str());
            return ERR_INVALID;
        }
    }
    return NO_ERR;
}

Errors Gui::createObjectActors()
{
    for (auto pair: m_kin->getObjects()) {
        Object* object = pair.second;
        if (object->setActorsRigidBody(new ActorsRigidBody(object))) {
            LOG_FAILURE("Failed to create actors for node %",
                    object->getName().c_str());
            return ERR_INVALID;
        }
    }
    return NO_ERR;
}

vtkSmartPointer<vtkRenderWindowInteractor> Gui::getRenderWindowInteractor()
{
    return m_renderWindowInteractor;
}

vtkSmartPointer<vtkRenderWindow> Gui::getRenderWindow()
{
    return m_renderWindow;
}

Window* Gui::getWin()
{
    return m_win;
}

RigidBodySystem* Gui::getRbs()
{
    return m_rbs;
}

Errors Gui::updateView()
{
    if (NO_ERR != m_scenes[ROBOT]->updateCamera()) {
        LOG_FAILURE("Failed to update camera");
        return ERR_INVALID;
    }
    return NO_ERR;
}

void Gui::setFramesVisibility(bool framesVisibility)
{
    m_framesVisibilityLock->Lock();
    m_framesVisibility = framesVisibility;
    m_framesVisibilityLock->Unlock();
}

bool Gui::getFramesVisibility()
{
    m_framesVisibilityLock->Lock();
    bool framesVisibility = m_framesVisibility;
    m_framesVisibilityLock->Unlock();
    return framesVisibility;
}

void Gui::setPlanesVisibility(bool planesVisibility)
{
    m_planesVisibilityLock->Lock();
    m_planesVisibility = planesVisibility;
    m_planesVisibilityLock->Unlock();
}

bool Gui::getPlanesVisibility()
{
    m_planesVisibilityLock->Lock();
    bool planesVisibility = m_planesVisibility;
    m_planesVisibilityLock->Unlock();
    return planesVisibility;
}

void Gui::setLinesVisibility(bool framesVisibility)
{
    m_linesVisibilityLock->Lock();
    m_linesVisibility = framesVisibility;
    m_linesVisibilityLock->Unlock();
}

bool Gui::getLinesVisibility()
{
    m_linesVisibilityLock->Lock();
    bool linesVisibility = m_linesVisibility;
    m_linesVisibilityLock->Unlock();
    return linesVisibility;
}

void Gui::setPointsVisibility(bool framesVisibility)
{
    m_pointsVisibilityLock->Lock();
    m_pointsVisibility = framesVisibility;
    m_pointsVisibilityLock->Unlock();
}

bool Gui::getPointsVisibility()
{
    m_pointsVisibilityLock->Lock();
    bool pointsVisibility = m_pointsVisibility;
    m_pointsVisibilityLock->Unlock();
    return pointsVisibility;
}

void Gui::setCadVisibility(bool framesVisibility)
{
    m_cadVisibilityLock->Lock();
    m_cadVisibility = framesVisibility;
    m_cadVisibilityLock->Unlock();
}

bool Gui::getCadVisibility()
{
    m_cadVisibilityLock->Lock();
    bool cadVisibility = m_cadVisibility;
    m_cadVisibilityLock->Unlock();
    return cadVisibility;
}

void Gui::setPathVisibility(bool visibility)
{
    m_pathVisibilityLock->Lock();
    m_pathVisibility = visibility;
    m_pathVisibilityLock->Unlock();
}

bool Gui::getPathVisibility()
{
    m_pathVisibilityLock->Lock();
    bool visibility = m_pathVisibility;
    m_pathVisibilityLock->Unlock();
    return visibility;
}

void Gui::setRecordRobotScene(bool recordRobotScene)
{
    m_recordRobotSceneLock->Lock();
    m_recordRobotScene = recordRobotScene;
    m_recordRobotSceneLock->Unlock();
}

bool Gui::getRecordRobotScene()
{
    m_recordRobotSceneLock->Lock();
    bool recordRobotScene = m_recordRobotScene;
    m_recordRobotSceneLock->Unlock();
    return recordRobotScene;
}

ConfigParser* Gui::getConfigParser()
{
    return m_cp;
}

Kinematics* Gui::getKinematics()
{
    return m_kin;
}

void Gui::setStatusMessage(const GuiStatusMessage_t &m)
{
    std::unique_lock<std::mutex> lock(m_mutexStatusMessage);
    if (m_mapStatusMessages.find(m.faultType) == m_mapStatusMessages.end()) {
        m_mapStatusMessages[m.faultType] = m;
    }
}

GuiStatusMessage_t Gui::getHighestPriorityStatusMessage()
{
    std::unique_lock<std::mutex> lock(m_mutexStatusMessage);

    GuiStatusMessage_t msg;
    msg.faultLevel = FAULT_LEVEL_NOFAULT;
    msg.faultType = FAULT_TYPE_NOFAULT;

    strncpy(msg.statusMessage, c_idleText, sizeof(msg.statusMessage));
    msg.statusMessage[MAX_STATUS_TEXT_SIZE - 1] = '\0';


    for (int i = (int)FAULT_LEVEL_CRITICAL; i >= (int)FAULT_LEVEL_NOFAULT; i--) {
        for (std::map<FaultTypes, GuiStatusMessage_t>::iterator
                it  = m_mapStatusMessages.begin();
                it != m_mapStatusMessages.end(); ++it) {
            if (it->second.faultLevel == (FaultLevels)i) {
                memcpy(&msg, &it->second, sizeof(it->second));
                m_mapStatusMessages.erase(it->first);
                return msg;
            }
        }
    }

    return msg;
}

vtkSmartPointer<vtkRenderer> Gui::getSceneRenderer(SCENES index)
{
  return m_scenes.at((size_t)index)->getRenderer();
}

Errors Gui::setDefaultVisibilityValues()
{
    setFramesVisibility(!m_cp->getWin()->default_hide_frames());
    setPlanesVisibility(!m_cp->getWin()->default_hide_planes());
    setLinesVisibility(!m_cp->getWin()->default_hide_lines());
    setPointsVisibility(!m_cp->getWin()->default_hide_points());
    setCadVisibility(!m_cp->getWin()->default_hide_cad());
    setPathVisibility(!m_cp->getWin()->default_hide_path());
    return NO_ERR;
}

EitOsMsgServerReceiver* Gui::getEitOsMsgServerReceiver()
{
    return m_eitOsMsgServerReceiver;
}

void Gui::setEitOsMsgServerReceiver(EitOsMsgServerReceiver* eitOsMsgServerReceiver)
{
    m_eitOsMsgServerReceiver = eitOsMsgServerReceiver;
}

Errors Gui::installTool(Object* tool)
{
    m_updateLock->Lock();
    if (tool->setActorsRigidBody(new ActorsRigidBody(tool))) {
        LOG_FAILURE("Failed to create actors for tool");
        return ERR_INVALID;
    }

    if (NO_ERR != m_scenes[ROBOT]->installTool(tool)) {
        LOG_FAILURE("Failed to install tool in robot scene");
        return ERR_INVALID;
    }

    m_updateLock->Unlock();

    return NO_ERR;
}

Errors Gui::removeTool()
{
    m_updateLock->Lock();
    if (NO_ERR != m_scenes[ROBOT]->removeTool()) {
        LOG_FAILURE("Failed to remove tool in robot scene");
        return ERR_INVALID;
    }

    m_updateLock->Unlock();
    return NO_ERR;
}

} // end of namespace tarsim

