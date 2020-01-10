/**
 *
 * @file: gui.h
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
#ifndef GUI_H
#define GUI_H

//INCLUDES
#include "simulatorMessages.h"
#include "eitErrors.h"

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkMutexLock.h>
#include <vector>
#include <map>
#include <mutex>

namespace SIM {
class Window;
class RigidBodySystem;
}

namespace tarsim {
// FORWARD DECLARATIONS
class Node;
class ConfigParser;
class SceneBase;
class Kinematics;
template<typename Type> class ThreadQueue;
class EitOsMsgServerReceiver;
class Object;

// TYPEDEFS AND DEFINES

// ENUMS
enum SCENES {
    ROBOT,
    JOINT_VALUES,
    END_EFFECTOR,
    BUTTONS,
    FAULTS,
    SIX_DOF,
    SPEED
};

// NAMESPACES AND STRUCTS

// CLASS DEFINITION
class Gui
{
public:
    // FUNCTIONS
    Gui(ConfigParser* cp, Kinematics* kin);
    virtual ~Gui();

    void startRenderWindowInteractor();

    void update();

    void vtkUpdate(
            vtkObject*,
            long unsigned int vtkNotUsed(eventId),
            void* vtkNotUsed(callData));

    void stop(
            vtkObject*,
            long unsigned int vtkNotUsed(eventId),
            void* vtkNotUsed(callData));

    void destroy();

    SIM::Window* getWin();
    SIM::RigidBodySystem* getRbs();

    vtkSmartPointer<vtkRenderWindowInteractor> getRenderWindowInteractor();
    vtkSmartPointer<vtkRenderWindow> getRenderWindow();
    Errors updateView();

    void setFramesVisibility(bool framesVisibility);
    bool getFramesVisibility();

    void setPlanesVisibility(bool planesVisibility);
    bool getPlanesVisibility();

    void setLinesVisibility(bool linesVisibility);
    bool getLinesVisibility();

    void setPointsVisibility(bool pointsVisibility);
    bool getPointsVisibility();

    void setCadVisibility(bool cadVisibility);
    bool getCadVisibility();

    void setPathVisibility(bool cadVisibility);
    bool getPathVisibility();

    void setRecordRobotScene(bool recordRobotScene);
    bool getRecordRobotScene();

    ConfigParser* getConfigParser();
    Kinematics* getKinematics();

    vtkSmartPointer<vtkRenderer> getSceneRenderer(SCENES index = ROBOT);

    void setStatusMessage(const GuiStatusMessage_t &m);
    GuiStatusMessage_t getHighestPriorityStatusMessage();
    unsigned int getFrameNumber() {return m_frameNumber;}

    EitOsMsgServerReceiver* getEitOsMsgServerReceiver();
    void setEitOsMsgServerReceiver(EitOsMsgServerReceiver* eitOsMsgServerReceiver);

    Errors installTool(Object* tool);
    Errors removeTool();
    // MEMBERS

private:
    // FUNCTIONS
    Errors record();
    Errors createRenderWindow();
    Errors setupRecording();
    Errors populateRenderWindow();
    Errors createTreeActors();
    Errors createObjectActors();
    Errors createNodeActors(Node* node);
    Errors addWidgets();
    Errors addExitButton();
    Errors setDefaultVisibilityValues();
    // MEMBERS
    ConfigParser* m_cp = nullptr;
    Kinematics* m_kin = nullptr;
    Node* m_root = nullptr;
    SIM::Window* m_win = nullptr;
    SIM::RigidBodySystem* m_rbs = nullptr;

    // The render window is the actual GUI window
    // that appears on the computer screen
    vtkSmartPointer<vtkRenderWindow> m_renderWindow;

    // The render window interactor captures mouse events
    // and will perform appropriate camera or actor manipulation
    // depending on the nature of the events.
    vtkSmartPointer<vtkRenderWindowInteractor> m_renderWindowInteractor;

    std::vector<SceneBase*> m_scenes {};

    vtkSmartPointer<vtkMutexLock> m_framesVisibilityLock =
            vtkSmartPointer<vtkMutexLock>::New();
    bool m_framesVisibility = true;

    vtkSmartPointer<vtkMutexLock> m_planesVisibilityLock =
            vtkSmartPointer<vtkMutexLock>::New();
    bool m_planesVisibility = true;

    vtkSmartPointer<vtkMutexLock> m_linesVisibilityLock =
            vtkSmartPointer<vtkMutexLock>::New();
    bool m_linesVisibility = true;

    vtkSmartPointer<vtkMutexLock> m_pointsVisibilityLock =
            vtkSmartPointer<vtkMutexLock>::New();
    bool m_pointsVisibility = true;

    vtkSmartPointer<vtkMutexLock> m_cadVisibilityLock =
            vtkSmartPointer<vtkMutexLock>::New();
    bool m_cadVisibility = true;

    vtkSmartPointer<vtkMutexLock> m_pathVisibilityLock =
            vtkSmartPointer<vtkMutexLock>::New();
    bool m_pathVisibility = true;

    vtkSmartPointer<vtkMutexLock> m_recordRobotSceneLock =
            vtkSmartPointer<vtkMutexLock>::New();
    bool m_recordRobotScene = false;

    ThreadQueue<Camera_t>* m_cameraQueue = nullptr;

    std::map<FaultTypes, GuiStatusMessage_t> m_mapStatusMessages;
    mutable std::mutex m_mutexStatusMessage;

    unsigned int m_frameNumber = 0;

    std::string m_dataFolder = "";
    bool m_isRecordingSetup = false;

    unsigned int m_kinCounter = 0;

    int m_windowSize[2] = {0, 0};

    vtkSmartPointer<vtkMutexLock> m_updateLock =
        vtkSmartPointer<vtkMutexLock>::New();

    EitOsMsgServerReceiver* m_eitOsMsgServerReceiver = nullptr;

    const char* c_idleText = "";

    bool m_isDestroying = false;

};
} // end of namespace tarsim
// ENDIF
#endif /* GUI_H */
