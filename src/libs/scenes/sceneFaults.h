/**
 *
 * @file: sceneFaults.h
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
#ifndef SCENE_FAULTS_H
#define SCENE_FAULTS_H

//INCLUDES
#include "sceneBase.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"
#include "vtkActor2D.h"
#include "simulatorMessages.h"
#include <chrono>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class SceneFaults: public SceneBase
{
public:
    // FUNCTIONS
    SceneFaults(Gui* gui);
    virtual ~SceneFaults() = default;

    Errors update(bool dimsChanged = false) override;
    // MEMBERS

private:
    // FUNCTIONS
    Errors addActorsToScene();
    Errors addActorCircleToScene();
    Errors updateActorCircle(const GuiStatusMessage_t &fault);
    Errors addActorFaultToScene();
    Errors updateActorFault(const GuiStatusMessage_t &fault);
    // MEMBERS
    vtkSmartPointer<vtkTextActor> m_actorFault;
    vtkSmartPointer<vtkActor2D> m_actorCircle;
    unsigned int m_faultCounter = 0;
    std::chrono::high_resolution_clock::time_point m_prevUpdateTime =
            std::chrono::high_resolution_clock::now();
    double m_updateDuration = 5.0;

    GuiStatusMessage_t m_statusMessage {};

    const char* c_initText = "System initialization...";
};
} // end of namespace tarsim
// ENDIF
#endif /* SCENE_FAULTS_H */
