/**
 * @file: incCmdButton.cpp
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
#include "incCmdButton.h"
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkButtonRepresentation.h>
#include <vtkRenderWindow.h>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
IncCmdButton::IncCmdButton()
{
    this->CallbackMapper->SetCallbackMethod(
            vtkCommand::LeftButtonPressEvent,
            vtkWidgetEvent::Select,
            this,
            IncCmdButton::SelectAction);

    this->CallbackMapper->SetCallbackMethod(
            vtkCommand::LeftButtonReleaseEvent,
            vtkWidgetEvent::EndSelect,
            this,
            IncCmdButton::EndSelectAction);
}

bool IncCmdButton::getIsPressed()
{
    m_isPressedLock->Lock();
    bool isPressed = m_isPressed;
    m_isPressedLock->Unlock();
    return isPressed;
}

void IncCmdButton::SelectAction(vtkAbstractWidget* w)
{
    IncCmdButton *self = reinterpret_cast<IncCmdButton*>(w);
    self->setIsPressed(true);

    int X = self->Interactor->GetEventPosition()[0];
    int Y = self->Interactor->GetEventPosition()[1];

    // The state must be hovering for anything to happen. MoveAction sets the
    // state.
    if ( self->WidgetState != vtkButtonWidget::Hovering )
    {
      return;
    }

    // Okay, make sure that the selection is in the current renderer
    if (!self->CurrentRenderer || !self->CurrentRenderer->IsInViewport(X,Y))
    {
      self->WidgetState = vtkButtonWidget::Start;
      return;
    }

    // We are definitely selected, Highlight as necessary.
    self->WidgetState = vtkButtonWidget::Selecting;
    self->EventCallbackCommand->SetAbortFlag(1);

    reinterpret_cast<vtkButtonRepresentation*>(self->WidgetRep)->NextState();
    self->InvokeEvent(vtkCommand::StateChangedEvent,nullptr);
    self->Render();
}
void IncCmdButton::EndSelectAction(vtkAbstractWidget* w)
{
      IncCmdButton *self = reinterpret_cast<IncCmdButton*>(w);
      self->setIsPressed(false);

      if ( self->WidgetState != vtkButtonWidget::Selecting )
      {
        return;
      }

      int X = self->Interactor->GetEventPosition()[0];
      int Y = self->Interactor->GetEventPosition()[1];

      int state = self->WidgetRep->ComputeInteractionState(X, Y);
      if ( state == vtkButtonRepresentation::Outside )
      {
        if ( self->ManagesCursor )
        {
          self->RequestCursorShape(VTK_CURSOR_DEFAULT);
        }
        self->WidgetRep->Highlight(vtkButtonRepresentation::HighlightNormal);
        self->WidgetState = vtkButtonWidget::Start;
      }
      else //state == vtkButtonRepresentation::Inside
      {
        if ( self->ManagesCursor )
        {
          self->RequestCursorShape(VTK_CURSOR_HAND);
        }
        self->WidgetRep->Highlight(vtkButtonRepresentation::HighlightHovering);
        self->WidgetState = vtkButtonWidget::Hovering;
      }

      // Complete interaction
      self->EventCallbackCommand->SetAbortFlag(1);

      reinterpret_cast<vtkButtonRepresentation*>(self->WidgetRep)->NextState();
      self->InvokeEvent(vtkCommand::StateChangedEvent,nullptr);
      self->Render();
}

void IncCmdButton::setIsPressed(bool isPressed)
{
    m_isPressedLock->Lock();
    m_isPressed = isPressed;
    m_isPressedLock->Unlock();
}

void IncCmdButton::setIndex(int32_t index)
{
    m_index = index;
}

int32_t IncCmdButton::getIndex()
{
    return m_index;
}

vtkStandardNewMacro(IncCmdButton);

} // end of namespace tarsim
