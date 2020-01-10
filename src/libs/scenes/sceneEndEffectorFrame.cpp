/**
 * @file: sceneEndEffectorFrame.cpp
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
#include "sceneEndEffectorFrame.h"
#include "gui.h"
#include "logClient.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
SceneEndEffectorFrame::SceneEndEffectorFrame(Gui* gui, Node* root):
    SceneBase(gui, gui->getWin()->end_effector_frame_scene())
{
    if (root == nullptr) {
        throw std::invalid_argument("No tree root was provided");
    }

    m_root = root;
    m_endEffectorNode =
            m_gui->getConfigParser()->getEndEffectorNode();
    m_endEffectorFrameNumber =
            m_gui->getConfigParser()->getEndEffectorFrameNumber();

    if (createCamera() != NO_ERR) {
        throw std::invalid_argument("Failed to create camera");
    }

    m_actorsEndEffector.resize(16);
    if (addActorsToScene() != NO_ERR) {
        throw std::invalid_argument("Failed to create actors");
    }
}

Errors SceneEndEffectorFrame::update(bool dimsChanged)
{
    if (m_isVisible != m_gui->getFramesVisibility()) {
        m_isVisible = m_gui->getFramesVisibility();

        m_actorFrameName->SetVisibility(m_isVisible);
        for (size_t i = 0; i < m_actorsEndEffector.size(); i++) {
            m_actorsEndEffector[i]->SetVisibility(m_isVisible);
        }

        if (!m_isVisible) {
            return NO_ERR;
        }

        if (NO_ERR != positionEndEffectorActors()) {
            LOG_FAILURE("Failed to update position of end-effector frame actors");
            return ERR_INVALID;
        }
    }

    if (!m_isVisible) {
        return NO_ERR;
    }

    if (NO_ERR != updateActorsEndEffector()) {
        LOG_FAILURE("Failed to update end-effector frame actor");
        return ERR_INVALID;
    }

    if (dimsChanged) {
        if (NO_ERR != positionEndEffectorActors()) {
            LOG_FAILURE("Failed to update position of end-effector frame actors");
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

Errors SceneEndEffectorFrame::addActorsToScene()
{
    if (NO_ERR != addActorFrameNameToScene()) {
        LOG_FAILURE("Failed to add frame name actor to the scene");
        return ERR_INVALID;
    }
    if (NO_ERR != addActorsEndEffectorToScene()) {
        LOG_FAILURE("Failed to add joint values actor to the scene");
        return ERR_INVALID;
    }
    return NO_ERR;
}

Errors SceneEndEffectorFrame::addActorFrameNameToScene()
{
    m_actorFrameName = vtkSmartPointer<vtkTextActor>::New();
    m_actorFrameName->SetInput("End-Effector Frame:");
    m_actorFrameName->GetTextProperty()->ItalicOn();
    m_actorFrameName->GetTextProperty()->BoldOn();

    if (m_scene.has_font_color()) {
        m_actorFrameName->GetTextProperty()->SetColor(
                m_scene.font_color().r(),
                m_scene.font_color().g(),
                m_scene.font_color().b());
    }

    m_actorFrameName->GetTextProperty()->SetJustificationToLeft();
    m_renderer->AddActor2D(m_actorFrameName);
    return NO_ERR;
}

Errors SceneEndEffectorFrame::addActorsEndEffectorToScene()
{
    if (nullptr != m_endEffectorNode) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                int index = 4 * i + j;
                m_actorsEndEffector[index] = vtkSmartPointer<vtkTextActor>::New();

                int p = 2;
                if (m_scene.data_precision() > 0) {
                    p = m_scene.data_precision();
                }

                Matrix4d t;
                if (NO_ERR != m_endEffectorNode->getFrame(m_endEffectorFrameNumber, t)) {
                    LOG_FAILURE("Failed to get end effector frame");
                    return ERR_INVALID;
                }

                std::ostringstream mij;
                mij << std::fixed << std::setprecision(p) << t(i, j);

                m_actorsEndEffector[index]->SetInput(mij.str().c_str());

                if (m_scene.has_font_color()) {
                    m_actorsEndEffector[index]->GetTextProperty()->SetColor(
                            m_scene.font_color().r(),
                            m_scene.font_color().g(),
                            m_scene.font_color().b());
                }
                m_actorsEndEffector[index]->GetTextProperty()->SetJustificationToRight();
                m_renderer->AddActor2D(m_actorsEndEffector[index]);
            }
        }
    }

    m_isVisible = !m_gui->getFramesVisibility();

    return NO_ERR;
}

Errors SceneEndEffectorFrame::updateActorsEndEffector()
{
  Matrix4d t = m_gui->getKinematics()->getXfmEndEffector();

  for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
          int index = 4 * i + j;
          int p_l = 2;
          int p_a = 3;
          if (m_scene.data_precision() > 0) {
              p_l = m_scene.data_precision();
              p_a = m_scene.data_precision();
          }

          std::ostringstream mij;
          if ((i < 3) and (j < 3)) {
              mij << std::fixed << std::setprecision(p_a) << t(i, j);
          } else if ((i < 3) && (j == 3)) {
              mij << std::fixed << std::setprecision(p_l) << t(i, j);
          } else {
              mij << std::fixed << std::setprecision(0) << t(i, j);
          }


          m_actorsEndEffector[index]->SetInput(mij.str().c_str());
      }
  }

    return NO_ERR;
}

std::vector<vtkSmartPointer<vtkTextActor>>
    SceneEndEffectorFrame::getActorsEndEffector()
{
    return m_actorsEndEffector;
}

Errors SceneEndEffectorFrame::positionEndEffectorActors()
{
    int* dim = m_renderer->GetSize();
    int w = dim[0];
    int h = dim[1];
    int bw = 0.05 * w;
    int bh = 0.05 * h;

    w *= 0.9;
    h *= 0.9;


    m_actorFrameName->SetPosition(bw, 4 * h / 5 + bh);
    m_actorFrameName->SetConstrainedFontSize(m_renderer, 0.9 * w, 0.9 * h / 5);

    if (nullptr != m_endEffectorNode) {
        int minFontSize = 1000;
        for (unsigned int i = 0; i < m_actorsEndEffector.size(); i++) {
            m_actorsEndEffector[i]->SetConstrainedFontSize(
                    m_renderer, 0.9 * w/4, 0.9 * h/5);
            int fs = m_actorsEndEffector[i]->GetTextProperty()->GetFontSize();
            if (minFontSize > fs) {
                minFontSize = fs;
            }
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                int index = 4 * i + j;
                m_actorsEndEffector[index]->SetPosition(
                        (j + 1) * w / 4 + bw,
                        (3 - i) * h / 5 + bh);

                m_actorsEndEffector[index]->GetTextProperty()->
                        SetFontSize(minFontSize);
            }
        }
    }
    return NO_ERR;
}
} // end of namespace tarsim
