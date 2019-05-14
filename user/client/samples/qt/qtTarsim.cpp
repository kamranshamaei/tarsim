/**
 * @file:  loggerMain.cpp
 *
 * @Created on: November 12, 2018
 * @Author: Kamran Shamaei
 *
 *
 * @brief - A sample program showing how you can embed Tarsim or any of its
 * scenes in a Qt application.
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright Kamran Shamaei
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 */


//INCLUDES
#include <QApplication>
#include <QVTKWidget.h>
#include <QSurfaceFormat>
#include <QTimer>
#include "tarsim.h"

tarsim::Tarsim sim("/home/kamran/dev/simApp/src/samples/fanuc_LRM200iD");
QVTKWidget* widget = nullptr;

void update() {
  sim.update();
  widget->GetRenderWindow()->Render();
}

/**
 * @brief creates the log server and remains blocked for ever.
 * @param argc - number of arguments
 * @param argv - list of arguments
 * @return EXIT_SUCCESS
 */
int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  widget = new QVTKWidget();
  widget->resize(1000, 1000);
  widget->GetRenderWindow()->SetNumberOfLayers(2);
  widget->GetRenderWindow()->AddRenderer(sim.getJointValuesSceneRenderer());
  widget->GetRenderWindow()->AddRenderer(sim.getRobotSceneRenderer());

  widget->GetRenderWindow()->AddRenderer(sim.getButtonsSceneRenderer());
  widget->GetRenderWindow()->AddRenderer(sim.getFaultsSceneRenderer());
  widget->GetRenderWindow()->AddRenderer(sim.getEndEffectorSceneRenderer());

  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, update);
  timer.start(30);

  widget->show();
  app.exec();

  delete widget;
  widget = nullptr;

  return EXIT_SUCCESS;
}
