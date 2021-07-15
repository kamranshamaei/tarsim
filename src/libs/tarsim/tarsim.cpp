/**
 * @file: tarsim.cpp
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
#include "tarsim.h"

#include "configParser.h"
#include "kinematics.h"
#include "gui.h"
#include "eitServer.h"
#include "logServer.h"
#include "logClient.h"
#include "fileSystem.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
Tarsim::Tarsim(const std::string &configFolderName,
        int policy, int priority, unsigned int msgPriority)
{
    int ret = 0;
    ret = std::system("rm -f /dev/mqueue/Tarsim*");
    mq_unlink(("/" + tarsim::RobotJointsReceiverThreadName).c_str());
    mq_unlink(("/" + tarsim::LogServerThreadName).c_str());

    try {
      m_logServer = new tarsim::LogServer();
      m_logServer->start();
    } catch (...) {
      throw std::invalid_argument("Failed to instantiate tarsim logger");
    }

    int counter = 0;
    while (!FileSystem::pathExists("/dev/mqueue/" + LogServerThreadName)) {
        usleep(10);
        if (counter > 100000) {
            break;
        }
    }
    usleep(1000);
    LOG_INFO("Tarsim logger started");

    try {
      // 1. Instantiate config parser
      m_cp = new ConfigParser(configFolderName);

      // 2. Instantiate kinematics engine
      m_kin = new Kinematics(m_cp);

      // 3. Instantiate the gui
      m_gui = new Gui(m_cp, m_kin);

      // 4. Instantiate the server
      m_srv = new EitServer(m_cp, m_kin, m_gui, policy, priority, msgPriority);

      // Provide the gui with the message server
      m_gui->setEitOsMsgServerReceiver(m_srv->getEitOsMsgServerReceiver());
    } catch (...) {
      throw std::invalid_argument("Failed to construct tarsim");
    }
}

Tarsim::~Tarsim()
{
  delete m_logServer;
  m_logServer = nullptr;

  delete m_srv;
  m_srv = nullptr;

  delete m_cp;
  m_cp = nullptr;

  delete m_kin;
  m_kin = nullptr;

  m_gui->destroy();
  delete m_gui;
  m_gui = nullptr;
}

void Tarsim::start()
{
  m_gui->startRenderWindowInteractor();
}

void Tarsim::update()
{
  m_gui->update();
}

void Tarsim::destroy()
{
  m_gui->destroy();
}

void Tarsim::render()
{
  m_gui->getRenderWindow()->Render();
}

vtkSmartPointer<vtkRenderWindow> Tarsim::getRenderWindow()
{
    return m_gui->getRenderWindow();
}

vtkSmartPointer<vtkRenderer> Tarsim::getRobotSceneRenderer()
{
    return m_gui->getSceneRenderer(ROBOT);
}

vtkSmartPointer<vtkRenderer> Tarsim::getJointValuesSceneRenderer()
{
    return m_gui->getSceneRenderer(JOINT_VALUES);
}

vtkSmartPointer<vtkRenderer> Tarsim::getButtonsSceneRenderer()
{
    return m_gui->getSceneRenderer(BUTTONS);
}

vtkSmartPointer<vtkRenderer> Tarsim::getFaultsSceneRenderer()
{
    return m_gui->getSceneRenderer(FAULTS);
}

vtkSmartPointer<vtkRenderer> Tarsim::getEndEffectorSceneRenderer()
{
    return m_gui->getSceneRenderer(END_EFFECTOR);
}

vtkSmartPointer<vtkRenderer> Tarsim::getSixDofSceneRenderer()
{
    return m_gui->getSceneRenderer(SIX_DOF);
}

} // end of namespace tarsim

