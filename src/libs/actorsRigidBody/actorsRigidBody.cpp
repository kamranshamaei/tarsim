
/**
 * @file: actorsRigidBody.cpp
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
#include "node.h"
#include "actorsRigidBody.h"
#include "logClient.h"
#include "fileSystem.h"

#include "vtkCellData.h"
#include "vtkLine.h"
#include "vtkPolyDataMapper.h"
#include "vtkMapper.h"
#include "vtkSphereSource.h"
#include "vtkTransform.h"

#include "vtkPolyData.h"
#include "vtkSTLReader.h"
#include "vtkAlgorithmOutput.h"


#include "vtkVersion.h"
#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkLineSource.h"
#include "vtkPlaneSource.h"
#include "vtkPoints.h"
#include "vtkLine.h"
#include "vtkPolyData.h"
#include "vtkProperty.h"

#include "vtkTransform.h"
#include "vtkProperty.h"
#include "vtkCaptionActor2D.h"
#include "vtkTextProperty.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
ActorsRigidBody::ActorsRigidBody(Node* node)
{
    if (node == nullptr) {
        throw std::invalid_argument(
                "No tree was provided to actors rigid body");
    }

    m_node = node;

    if (createNodeActors() != NO_ERR) {
        throw std::invalid_argument("Failed to create actors");
    }
}

Errors ActorsRigidBody::createNodeActors()
{
    Matrix4d xfm = m_node->getXfm();

    if (createPlaneActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to create plane actors");
        return ERR_INVALID;
    }

    if (createLineActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to create line actors");
        return ERR_INVALID;
    }

    if (createPointActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to create point actors");
        return ERR_INVALID;
    }

    if (createCadActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to create cad actors");
        return ERR_INVALID;
    }

    if (createFrameActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to create frame actors");
        return ERR_INVALID;
    }

    return NO_ERR;
}

Errors ActorsRigidBody::updateNodeActors()
{
    Matrix4d xfm = m_node->getXfm();

    if (updatePlaneActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to update plane actors");
        return ERR_INVALID;
    }

    if (updateLineActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to update line actors");
        return ERR_INVALID;
    }

    if (updatePointActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to update point actors");
        return ERR_INVALID;
    }

    if (updateCadActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to update cad actors");
        return ERR_INVALID;
    }

    if (updateFrameActors(xfm) != NO_ERR) {
        LOG_FAILURE("Failed to update frame actors");
        return ERR_INVALID;
    }

    return NO_ERR;
}

Errors ActorsRigidBody::createPlaneActors(const Matrix4d &xfm)
{
    m_actorsPlanes.resize(m_node->getRigidBodyAppearance().planes_size());
    for (size_t i = 0; i < m_node->getRigidBodyAppearance().planes_size(); i++) {
        // Create two points for each line
        Vector4d u = xfm * Vector4d(
                m_node->getRigidBodyAppearance().planes(i).u().x(),
                m_node->getRigidBodyAppearance().planes(i).u().y(),
                m_node->getRigidBodyAppearance().planes(i).u().z(),
                0.0);

        Vector4d v = xfm * Vector4d(
                m_node->getRigidBodyAppearance().planes(i).v().x(),
                m_node->getRigidBodyAppearance().planes(i).v().y(),
                m_node->getRigidBodyAppearance().planes(i).v().z(),
                0.0);

        Vector4d s = u + v;

        Vector4d c = xfm * Vector4d(
                m_node->getRigidBodyAppearance().planes(i).center().x(),
                m_node->getRigidBodyAppearance().planes(i).center().y(),
                m_node->getRigidBodyAppearance().planes(i).center().z(),
                1) - s/2.0;

        // Create plane source
        vtkSmartPointer<vtkPlaneSource> planeSource =
                vtkSmartPointer<vtkPlaneSource>::New();
        planeSource->SetPoint1(c.x() + u.x(), c.y() + u.y(), c.z() + u.z());
        planeSource->SetPoint2(c.x() + v.x(), c.y() + v.y(), c.z() + v.z());
        planeSource->SetOrigin(c.x(), c.y(), c.z());
        planeSource->Update();

        // Visualize
        vtkSmartPointer<vtkPolyDataMapper> mapper =
                vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(planeSource->GetOutputPort());
        vtkSmartPointer<vtkActor> actor =
                vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetLineWidth(10);

        actor->GetProperty()->SetColor(
                m_node->getRigidBodyAppearance().planes(i).color().r(),
                m_node->getRigidBodyAppearance().planes(i).color().g(),
                m_node->getRigidBodyAppearance().planes(i).color().b());

        m_actorsPlanes[i] = actor;
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updatePlaneActors(const Matrix4d &xfm)
{
    for (unsigned int i = 0; i < m_actorsPlanes.size(); i++) {
        Vector4d u = xfm * Vector4d(
                m_node->getRigidBodyAppearance().planes(i).u().x(),
                m_node->getRigidBodyAppearance().planes(i).u().y(),
                m_node->getRigidBodyAppearance().planes(i).u().z(),
                0.0);

        Vector4d v = xfm * Vector4d(
                m_node->getRigidBodyAppearance().planes(i).v().x(),
                m_node->getRigidBodyAppearance().planes(i).v().y(),
                m_node->getRigidBodyAppearance().planes(i).v().z(),
                0.0);

        Vector4d s = u + v;

        Vector4d c = xfm * Vector4d(
                m_node->getRigidBodyAppearance().planes(i).center().x(),
                m_node->getRigidBodyAppearance().planes(i).center().y(),
                m_node->getRigidBodyAppearance().planes(i).center().z(),
                1) - s/2.0;

        vtkSmartPointer<vtkAlgorithm> algorithm = m_actorsPlanes.at(i)->
                GetMapper()->GetInputConnection(0, 0)->GetProducer();
        vtkSmartPointer<vtkPlaneSource> planeSource =
                vtkPlaneSource::SafeDownCast(algorithm);

        planeSource->SetPoint1(c.x() + u.x(), c.y() + u.y(), c.z() + u.z());
        planeSource->SetPoint2(c.x() + v.x(), c.y() + v.y(), c.z() + v.z());
        planeSource->SetOrigin(c.x(), c.y(), c.z());
        planeSource->Update();
        planeSource->Modified();
    }
    return NO_ERR;
}

Errors ActorsRigidBody::createLineActors(const Matrix4d &xfm)
{
    if (m_node->getRigidBodyAppearance().lines_size() > 0) {
        m_actorsLines.resize(m_node->getRigidBodyAppearance().lines_size());
        for (unsigned int i = 0;
                i < m_node->getRigidBodyAppearance().lines_size(); i++) {
            // Create two points for each line
            Vector4d from = xfm * Vector4d(
                    m_node->getRigidBodyAppearance().lines(i).from().x(),
                    m_node->getRigidBodyAppearance().lines(i).from().y(),
                    m_node->getRigidBodyAppearance().lines(i).from().z(),
                    0.0);

            Vector4d to = xfm * Vector4d(
                    m_node->getRigidBodyAppearance().lines(i).to().x(),
                    m_node->getRigidBodyAppearance().lines(i).to().y(),
                    m_node->getRigidBodyAppearance().lines(i).to().z(),
                    0.0);

            double p1[3] = {from.x(), from.y(), from.z()};
            double p2[3] = {to.x(), to.y(), to.z()};

            // Create line source
            vtkSmartPointer<vtkLineSource> lineSource =
                    vtkSmartPointer<vtkLineSource>::New();
            lineSource->SetPoint1(p1);
            lineSource->SetPoint2(p2);
            lineSource->Update();

            // Visualize
            vtkSmartPointer<vtkPolyDataMapper> mapper =
                    vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(lineSource->GetOutputPort());
            vtkSmartPointer<vtkActor> actor =
                    vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetLineWidth(
                    m_node->getRigidBodyAppearance().lines(i).width());

            actor->GetProperty()->SetColor(
                    m_node->getRigidBodyAppearance().lines(i).color().r(),
                    m_node->getRigidBodyAppearance().lines(i).color().g(),
                    m_node->getRigidBodyAppearance().lines(i).color().b());

            m_actorsLines[i] = actor;
        }
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updateLineActors(const Matrix4d &xfm)
{
    for (unsigned int i = 0; i < m_actorsLines.size(); i++) {
        Vector4d from = xfm * Vector4d(
                m_node->getRigidBodyAppearance().lines(i).from().x(),
                m_node->getRigidBodyAppearance().lines(i).from().y(),
                m_node->getRigidBodyAppearance().lines(i).from().z(),
                1);

        Vector4d to = xfm * Vector4d(
                m_node->getRigidBodyAppearance().lines(i).to().x(),
                m_node->getRigidBodyAppearance().lines(i).to().y(),
                m_node->getRigidBodyAppearance().lines(i).to().z(),
                1);

        double p1[3] = {from.x(), from.y(), from.z()};
        double p2[3] = {to.x(), to.y(), to.z()};

        vtkSmartPointer<vtkAlgorithm> algorithm = m_actorsLines.at(i)->
                GetMapper()->GetInputConnection(0, 0)->GetProducer();
        vtkSmartPointer<vtkLineSource> lineSource =
                vtkLineSource::SafeDownCast(algorithm);

        lineSource->SetPoint1(p1);
        lineSource->SetPoint2(p2);
        lineSource->Update();
        lineSource->Modified();
    }
    return NO_ERR;
}

Errors ActorsRigidBody::createPointActors(const Matrix4d &xfm)
{
    if (m_node->getRigidBodyAppearance().points_size() > 0) {
        m_actorsPoints.resize(m_node->getRigidBodyAppearance().points_size());
        for (unsigned int i = 0;
                i < m_node->getRigidBodyAppearance().points_size(); i++) {
            // Create a sphere
            vtkSmartPointer<vtkSphereSource> sphereSource =
                    vtkSmartPointer<vtkSphereSource>::New();

            Vector4d v = xfm * Vector4d(
                    m_node->getRigidBodyAppearance().points(i).location().x(),
                    m_node->getRigidBodyAppearance().points(i).location().y(),
                    m_node->getRigidBodyAppearance().points(i).location().z(),
                    1);
            double p[3] = {v.x(), v.y(), v.z()};

            sphereSource->SetCenter(p[0], p[1], p[2]);
            sphereSource->SetRadius(
                    m_node->getRigidBodyAppearance().points(i).radius());
            sphereSource->Update();

            // Setup the visualization pipeline
            vtkSmartPointer<vtkPolyDataMapper> mapper =
                    vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(sphereSource->GetOutputPort());

            vtkSmartPointer<vtkActor> actor =
                    vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);

            actor->GetProperty()->SetColor(
                    m_node->getRigidBodyAppearance().points(i).color().r(),
                    m_node->getRigidBodyAppearance().points(i).color().g(),
                    m_node->getRigidBodyAppearance().points(i).color().b());

            m_actorsPoints[i] = actor;
        }
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updatePointActors(const Matrix4d &xfm)
{
    for (unsigned int i = 0; i < m_actorsPoints.size(); i++) {
        Vector4d center = xfm * Vector4d(
                m_node->getRigidBodyAppearance().points(i).location().x(),
                m_node->getRigidBodyAppearance().points(i).location().y(),
                m_node->getRigidBodyAppearance().points(i).location().z(),
                1);

        double p[3] = {center.x(), center.y(), center.z()};

        vtkSmartPointer<vtkAlgorithm> algorithm =
                m_actorsPoints.at(i)->GetMapper()->GetInputConnection(0, 0)->GetProducer();
        vtkSmartPointer<vtkSphereSource> sphereSource =
                vtkSphereSource::SafeDownCast(algorithm);

        sphereSource->SetCenter(p[0], p[1], p[2]);
    }
    return NO_ERR;
}

Errors ActorsRigidBody::createCadActors(const Matrix4d &xfmWldRb)
{
    for (size_t i = 0; i < m_node->getRigidBodyAppearance().cad_size(); i++) {
      if (m_node->getRigidBodyAppearance().cad(i).path().size() > 0) {
          std::string dir = m_node->getConfigFolderName();
          std::string file = dir + "/" + m_node->getRigidBodyAppearance().cad(i).path();

          if (FileSystem::fileExists(file)) {
              vtkSmartPointer<vtkSTLReader> stlReader =
                      vtkSmartPointer<vtkSTLReader>::New();

              stlReader->SetFileName(file.c_str());

              // Visualize
              vtkSmartPointer<vtkPolyDataMapper> mapper =
                      vtkSmartPointer<vtkPolyDataMapper>::New();

              mapper->SetInputConnection(stlReader->GetOutputPort());

              vtkSmartPointer<vtkActor> actorsCad =
                  vtkSmartPointer<vtkActor>::New();
              actorsCad->SetMapper(mapper);

              actorsCad->GetProperty()->SetColor(
                      m_node->getRigidBodyAppearance().cad(i).color().r(),
                      m_node->getRigidBodyAppearance().cad(i).color().g(),
                      m_node->getRigidBodyAppearance().cad(i).color().b());

              actorsCad->GetProperty()->SetOpacity(
                      m_node->getRigidBodyAppearance().cad(i).opacity());

              actorsCad->GetProperty()->SetInterpolationToGouraud();

              Matrix4d xfmRbCad;
              xfmRbCad <<
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rxx(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rxy(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rxz(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().tx(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().ryx(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().ryy(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().ryz(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().ty(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rzx(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rzy(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rzz(),
                  m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().tz(),
                  0, 0, 0, 1;

              Matrix4d xfmWldCad = xfmWldRb * xfmRbCad;

              const double m[16] = {
                      xfmWldCad(0, 0), xfmWldCad(0, 1), xfmWldCad(0, 2), xfmWldCad(0, 3),
                      xfmWldCad(1, 0), xfmWldCad(1, 1), xfmWldCad(1, 2), xfmWldCad(1, 3),
                      xfmWldCad(2, 0), xfmWldCad(2, 1), xfmWldCad(2, 2), xfmWldCad(2, 3),
                                    0,               0,               0,              1};
              vtkSmartPointer<vtkTransform> t =
                      vtkSmartPointer<vtkTransform>::New();
              t->SetMatrix(m);
              actorsCad->SetUserTransform(t);
              m_actorsCad.push_back(actorsCad);
          }
      }
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updateCadActors(const Matrix4d &xfmWldRb)
{
    for (unsigned int i = 0; i < m_actorsCad.size(); i++) {
        Matrix4d xfmRbCad;
        xfmRbCad <<
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rxx(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rxy(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rxz(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().tx(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().ryx(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().ryy(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().ryz(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().ty(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rzx(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rzy(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().rzz(),
            m_node->getRigidBodyAppearance().cad(i).xfm_cad_to_rigid_body().tz(),
            0, 0, 0, 1;

        Matrix4d m = xfmWldRb * xfmRbCad;

        const double t[16] = {
                        m(0, 0), m(0, 1), m(0, 2), m(0, 3),
                        m(1, 0), m(1, 1), m(1, 2), m(1, 3),
                        m(2, 0), m(2, 1), m(2, 2), m(2, 3),
                              0,       0,       0,      1};

        vtkSmartPointer<vtkTransform> xfm =
                vtkSmartPointer<vtkTransform>::New();
        xfm->SetMatrix(t);
        m_actorsCad[i]->SetUserTransform(xfm);
    }
    return NO_ERR;
}

Errors ActorsRigidBody::createFrameActors(const Matrix4d &xfmWldRb)
{
    if (m_node->getCoordinateFrames()->size() > 0) {
        m_actorsFrames.resize(m_node->getCoordinateFrames()->size());

        for (unsigned int i = 0; i < m_node->getCoordinateFrames()->size(); i++) {
            vtkSmartPointer<vtkTransform> transform =
                    vtkSmartPointer<vtkTransform>::New();

            Matrix4d xfmRbFrame;
            xfmRbFrame <<
                    m_node->getCoordinateFrames()->at(i).xfm().rxx(),
                    m_node->getCoordinateFrames()->at(i).xfm().rxy(),
                    m_node->getCoordinateFrames()->at(i).xfm().rxz(),
                    m_node->getCoordinateFrames()->at(i).xfm().tx(),

                    m_node->getCoordinateFrames()->at(i).xfm().ryx(),
                    m_node->getCoordinateFrames()->at(i).xfm().ryy(),
                    m_node->getCoordinateFrames()->at(i).xfm().ryz(),
                    m_node->getCoordinateFrames()->at(i).xfm().ty(),

                    m_node->getCoordinateFrames()->at(i).xfm().rzx(),
                    m_node->getCoordinateFrames()->at(i).xfm().rzy(),
                    m_node->getCoordinateFrames()->at(i).xfm().rzz(),
                    m_node->getCoordinateFrames()->at(i).xfm().tz(),

                    0.0, 0.0, 0.0, 1.0;

            Matrix4d m = xfmWldRb * xfmRbFrame;

            const double t[16] = {
                            m(0, 0), m(0, 1), m(0, 2), m(0, 3),
                            m(1, 0), m(1, 1), m(1, 2), m(1, 3),
                            m(2, 0), m(2, 1), m(2, 2), m(2, 3),
                                  0,       0,       0,      1};

            transform->SetMatrix(t);

            vtkSmartPointer<vtkAxesActor> axes =
                    vtkSmartPointer<vtkAxesActor>::New();

            axes->SetUserTransform(transform);

            axes->SetShaftTypeToCylinder();
            axes->SetNormalizedTipLength(0.3, 0.3, 0.3);
            axes->SetNormalizedShaftLength(0.7, 0.7, 0.7);

            if (m_node->getCoordinateFrames()->at(i).length() >= 0.0) {
                axes->SetTotalLength(
                        m_node->getCoordinateFrames()->at(i).length(),
                        m_node->getCoordinateFrames()->at(i).length(),
                        m_node->getCoordinateFrames()->at(i).length());
            }

            if (m_node->getCoordinateFrames()->at(i).length() >= 0.0) {
                axes->SetCylinderRadius(m_node->getCoordinateFrames()->at(i).radius());
                axes->SetConeRadius(m_node->getCoordinateFrames()->at(i).radius() * 20.0);
            }

            if (m_node->getCoordinateFrames()->at(i).font() >= 0) {
                axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
                axes->GetXAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->
                        SetFontSize(m_node->getCoordinateFrames()->at(i).font());

                axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
                axes->GetYAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->
                        SetFontSize(m_node->getCoordinateFrames()->at(i).font());

                axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
                axes->GetZAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->
                        SetFontSize(m_node->getCoordinateFrames()->at(i).font());
            }

            axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1.0, 0.0, 0.0);
            axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 1.0, 0.0);
            axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 1.0);

            m_actorsFrames[i] = axes;

        }
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updateFrameActors(const Matrix4d &xfmWldRb)
{
    for (unsigned int i = 0; i < m_actorsFrames.size(); i++) {
        Matrix4d xfmRbFrame;
        xfmRbFrame <<
                m_node->getCoordinateFrames()->at(i).xfm().rxx(),
                m_node->getCoordinateFrames()->at(i).xfm().rxy(),
                m_node->getCoordinateFrames()->at(i).xfm().rxz(),
                m_node->getCoordinateFrames()->at(i).xfm().tx(),

                m_node->getCoordinateFrames()->at(i).xfm().ryx(),
                m_node->getCoordinateFrames()->at(i).xfm().ryy(),
                m_node->getCoordinateFrames()->at(i).xfm().ryz(),
                m_node->getCoordinateFrames()->at(i).xfm().ty(),

                m_node->getCoordinateFrames()->at(i).xfm().rzx(),
                m_node->getCoordinateFrames()->at(i).xfm().rzy(),
                m_node->getCoordinateFrames()->at(i).xfm().rzz(),
                m_node->getCoordinateFrames()->at(i).xfm().tz(),

                0.0, 0.0, 0.0, 1.0;

        Matrix4d m = xfmWldRb * xfmRbFrame;

        const double t[16] = {
                        m(0, 0), m(0, 1), m(0, 2), m(0, 3),
                        m(1, 0), m(1, 1), m(1, 2), m(1, 3),
                        m(2, 0), m(2, 1), m(2, 2), m(2, 3),
                              0,       0,       0,      1};
        vtkSmartPointer<vtkTransform> transform =
                vtkSmartPointer<vtkTransform>::New();
        transform->SetMatrix(t);
        m_actorsFrames.at(i)->SetUserTransform(transform);
    }
    return NO_ERR;
}

std::vector<vtkSmartPointer<vtkActor>> ActorsRigidBody::getActorsPlanes()
{
    return m_actorsPlanes;
}

std::vector<vtkSmartPointer<vtkActor>> ActorsRigidBody::getActorsLines()
{
    return m_actorsLines;
}

std::vector<vtkSmartPointer<vtkActor>> ActorsRigidBody::getActorsPoints()
{
    return m_actorsPoints;
}

std::vector<vtkSmartPointer<vtkActor>> ActorsRigidBody::getActorCad()
{
    return m_actorsCad;
}

std::vector<vtkSmartPointer<vtkAxesActor>> ActorsRigidBody::getActorsFrames()
{
    return m_actorsFrames;
}

Errors ActorsRigidBody::updateFrameVisibility(bool framesVisibility)
{
    for (unsigned int i = 0; i < m_actorsFrames.size(); i++) {
        m_actorsFrames[i]->SetVisibility(framesVisibility);
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updatePlaneVisibility(bool planesVisibility)
{
    for (unsigned int i = 0; i < m_actorsPlanes.size(); i++) {
        m_actorsPlanes[i]->SetVisibility(planesVisibility);
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updateLinesVisibility(bool linesVisibility)
{
    for (unsigned int i = 0; i < m_actorsLines.size(); i++) {
        m_actorsLines[i]->SetVisibility(linesVisibility);
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updatePointsVisibility(bool pointsVisibility)
{
    for (unsigned int i = 0; i < m_actorsPoints.size(); i++) {
        m_actorsPoints[i]->SetVisibility(pointsVisibility);
    }
    return NO_ERR;
}

Errors ActorsRigidBody::updateCadVisibility(bool cadVisibility)
{
    for (unsigned int i = 0; i < m_actorsCad.size(); i++) {
        m_actorsCad[i]->SetVisibility(cadVisibility);
    }
    return NO_ERR;
}
}
