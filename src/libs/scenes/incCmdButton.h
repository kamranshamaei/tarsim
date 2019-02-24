/**
 *
 * @file: incCmdButton.h
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
#ifndef INC_CMD_BUTTON
#define INC_CMD_BUTTON

//INCLUDES
#include <vtkCommand.h>
#include <vtkWidgetEventTranslator.h>
#include <vtkObjectFactory.h>
#include <vtkCallbackCommand.h>
#include <vtkButtonWidget.h>
#include <vtkWidgetCallbackMapper.h>
#include <vtkMutexLock.h>
#include <vtkWidgetEvent.h>
#include <vtkMutexLock.h>
#include <vtkSmartPointer.h>

namespace tarsim {
// FORWARD DECLARATIONS

// TYPEDEFS AND DEFINES

// ENUMS

// NAMESPACES AND STRUCTS

// CLASS DEFINITION
class IncCmdButton: public vtkButtonWidget
{
public:
    static IncCmdButton* New();

    bool getIsPressed();

    void setIndex(int index);

    int getIndex();


protected:
    IncCmdButton();
    ~IncCmdButton() {}
private:
    static void SelectAction(vtkAbstractWidget* w);
    static void EndSelectAction(vtkAbstractWidget* w);
    void setIsPressed(bool isPressed);

    bool m_isPressed = false;
    int32_t m_index = -1;
    vtkSmartPointer<vtkMutexLock> m_isPressedLock =
            vtkSmartPointer<vtkMutexLock>::New();
};
} // end of namespace tarsim
// ENDIF
#endif /* INC_CMD_BUTTON */
