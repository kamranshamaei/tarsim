/**
* @file: tarsimClient.h
*
* @Created on: March 31, 2018
* @Author: Kamran Shamaei
*
*
* @brief - This is the main class of the simulator. It includes five main member
* objects:
* CofigParser: Processes the configuration files rbs.txt and win.txt that the
*   user creates to define the robot
* Kinematics: The realtime kinematics engine. It processes all kinematics data
*   including the joint values, rigid body poses, objects poses, etc.
* Gui: Based on the output of the kinematics class it generates and displays
*   rigid bodies to the user. It relies on VTK library.
* EitServer: This class is the server that tarsimClient communicates with to
*   send joint values and other requests and receive poses and other data.
* LogServer: It logs all the data that come from the simulator through a
*   realtime-safe logging mechanism. The data are saved in file /tmp/eitLog.txt.
*   Every time class Tarsim is instantiated, the previous log file is deleted
*   and a new file is created.
*
* @copyright Copyright [2017-2018] Kamran Shamaei .
* All Rights Reserved.
*
* This file is subject to the terms and conditions defined in
* file 'LICENSE', which is part of this source code package.
*/


#ifndef TARSIM_H
#define TARSIM_H

#include <string>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include "ipcMessages.h"

namespace tarsim {
class Gui;
class ConfigParser;
class Kinematics;
class EitServer;
class LogServer;

class Tarsim
{
public:
    /**
     * Constructor
     * @param configFolderName path to where the config files are store
     * @param policy The realtime thread policy (if fails, it just used default
     * non-realtime value of 0)
     * @param priority The realtime thread priority (if fails, it just used
     * non-realtime value of 0)
     */
    Tarsim(const std::string &configFolderName,
            int policy = DEFAULT_RT_THREAD_POLICY,
            int priority = DEFAULT_RT_THREAD_PRIORITY,
            unsigned int msgPriority = DEFAULT_MSG_PRIORITY);

    /**
     * Destructor
     */
    virtual ~Tarsim();

    /**
     * Starts the simulator GUI, it is a blocking call.
     */
    void start();

    /**
     * Updates the gui scenes. If you instantiate Tarsim inside a Qt widget,
     * in order to update the scenes, you can call this function.
     */
    void update();

    /**
     * Destroys all member objects
     */
    void destroy();

    /**
     * Renders the render window content
     */
    void render();

    /**
     * Get the gui's vtk RendererWindow. You can use this to access all
     * underlying mechanisms as far as vtk allows or use in your Qt applications
     */
    vtkSmartPointer<vtkRenderWindow> getRenderWindow();

    /**
     * Get the renderer of the scene that display the robot. You can use this
     * to get the renderer and display it in a Qt widget
     * (e.g. of type QVTKWidget).
     */
    vtkSmartPointer<vtkRenderer> getRobotSceneRenderer();

    /**
     * Get the renderer of the scene that display the joint values. You can use
     * this to get the renderer and display it in a Qt widget
     * (e.g. of type QVTKWidget).
     */
    vtkSmartPointer<vtkRenderer> getJointValuesSceneRenderer();

    /**
     * Get the renderer of the scene that display the buttons. You can use this
     * to get the renderer and display it in a Qt widget
     * (e.g. of type QVTKWidget).
     */
    vtkSmartPointer<vtkRenderer> getButtonsSceneRenderer();

    /**
     * Get the renderer of the scene that display the faults. You can use this
     * to get the renderer and display it in a Qt widget
     * (e.g. of type QVTKWidget).
     */
    vtkSmartPointer<vtkRenderer> getFaultsSceneRenderer();

    /**
     * Get the renderer of the scene that display the end-effector frame.
     * You can use this to get the renderer and display it in a Qt widget
     * (e.g. of type QVTKWidget).
     */
    vtkSmartPointer<vtkRenderer> getEndEffectorSceneRenderer();

    /**
     * Get the renderer of the scene that display the six-dof incremental commands.
     * You can use this to get the renderer and display it in a Qt widget
     * (e.g. of type QVTKWidget).
     */
    vtkSmartPointer<vtkRenderer> getSixDofSceneRenderer();

private:
    /**
     * Member object that processes configuration files like rbx.txt and win.txt
     */
    ConfigParser* m_cp = nullptr;

    /**
     * Kinematics engine that process all kinematic data such as rigid body
     * poses, object poses, etc.
     */
    Kinematics* m_kin = nullptr;

    /*
     * What the user sees. It is based on the graphics library vtk. (Many thanks
     * to VTK!)
     */
    Gui* m_gui = nullptr;

    /*
     * The server that relies on inter-process communication using POSIX
     * message queue.
     */
    EitServer* m_srv = nullptr;

    /**
     * Logger server, all the data are logged here through a realtime-safe
     * logging mechanism
     */
    LogServer* m_logServer = nullptr;

};
} // end of namespace tarsim
#endif /* TARSIM_H */
