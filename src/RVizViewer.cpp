/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Matthew Klingensmith <mklingen@cs.cmu.edu>
         Michael Koval <mkoval@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#include <qapplication.h>
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QMenuBar>
#include <QString>
#include <QTimer>
#include <boost/format.hpp>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include "util/ros_conversions.h"
#include "RVizViewer.h"

using boost::format;
using boost::str;

static double const kRefreshRate = 30;
static std::string const kOffscreenCameraName = "OffscreenCamera";

namespace or_interactivemarker {

/*
 * Public
 */
RVizViewer::RVizViewer(OpenRAVE::EnvironmentBasePtr env,
                       std::string const &topic_name,
                       bool anonymize)
    : InteractiveMarkerViewer(env, GenerateTopicName(topic_name, anonymize))
    , timer_(NULL)
{
    initialize();

    rviz_manager_ = getManager();
    rviz_main_panel_ = rviz_manager_->getRenderPanel();
    rviz_scene_manager_ = rviz_manager_->getSceneManager();

    environment_display_ = dynamic_cast<rviz::EnvironmentDisplay *>(
        rviz_manager_->createDisplay("or_interactivemarker::rviz::EnvironmentDisplay",
                                     "OpenRAVE Environment", true)
    );
    environment_display_->set_environment(env);
    environment_frame_handle_ = environment_display_->RegisterFrameChangeCallback(
        boost::bind(&RVizViewer::set_parent_frame, this, _1));

    // Create an extra camera to use for off-screen rendering.
    offscreen_camera_ = rviz_scene_manager_->createCamera(kOffscreenCameraName);

    InitializeLighting();
    InitializeInteractiveMarkers();
    InitializeMenus();

    installEventFilter(this);
}

int RVizViewer::main(bool bShow)
{
    qApp->setActiveWindow(this);

    timer_ = new QTimer(this);
    timer_->setInterval(33);
    timer_->setSingleShot(false);
    timer_->start();

    connect(timer_, SIGNAL(timeout()), this, SLOT(EnvironmentSyncSlot()));

    running_ = true;
    show();
    return qApp->exec();
}

void RVizViewer::quitmainloop()
{
    // TODO: Disconnect the timer.
    running_ = false;
    qApp->quit();
}

void RVizViewer::SetSize(int w, int h)
{
    resize(w, h);
}

void RVizViewer::Move(int x, int y)
{
    move(x, y);
}

std::string const &RVizViewer::GetName() const
{
    // We need to store the string in member variable to avoid returning a
    // temporary reference. This isn't great, but it's the best we can do.
    window_title_ = windowTitle().toStdString();
    return window_title_;
}

void RVizViewer::SetName(std::string const &name)
{
    setWindowTitle(QString::fromStdString(name));
}

void RVizViewer::SetCamera(OpenRAVE::RaveTransform<float> &trans,
                           float focalDistance)
{
    SetCamera(rviz_main_panel_->getCamera(), trans, focalDistance);
}

OpenRAVE::RaveTransform<float> RVizViewer::GetCameraTransform() const
{
    Ogre::Camera *const camera = rviz_main_panel_->getCamera();

    OpenRAVE::RaveTransform<float> pose;
    pose.trans = util::ToRaveVector(camera->getPosition());
    pose.rot = util::ToRaveQuaternion(camera->getOrientation());

    return pose;
}

OpenRAVE::geometry::RaveCameraIntrinsics<float> RVizViewer::GetCameraIntrinsics() const
{
    Ogre::Camera* camera = rviz_main_panel_->getCamera();
    Ogre::Matrix4 projection_matrix = camera->getProjectionMatrix();

    OpenRAVE::geometry::RaveCameraIntrinsics<float> intrinsics;
    intrinsics.focal_length = camera->getFocalLength();
    intrinsics.fx = projection_matrix[0][0];
    intrinsics.fy = projection_matrix[1][1];
    intrinsics.cx = projection_matrix[0][2];
    intrinsics.cy = projection_matrix[1][2];
    intrinsics.distortion_model = "";
    return intrinsics;
}


/*
 * Slots
 */
void RVizViewer::LoadEnvironmentSlot()
{
    QString file = QFileDialog::getOpenFileName(this, "Load", ".");
    if (file.count() > 0) {
        if (!GetEnv()->Load(file.toStdString())) {
            QMessageBox::warning(this, "Load", "Failed to load environment.");
        }
    }
}

void RVizViewer::EnvironmentSyncSlot()
{
    if (running_) {
        if(do_sync_) {
            EnvironmentSync();
        }
        viewer_callbacks_();
    }
}


/*
 * Private
 */
void RVizViewer::InitializeLighting()
{
    rviz_scene_manager_->setShadowTechnique(Ogre::SHADOWTYPE_NONE);

    Ogre::Light *light = rviz_scene_manager_->createLight("FillLight");
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setDiffuseColour(0.6, 0.55, 0.5);
    light->setSpecularColour(1, 1, 1);
    light->setDirection(0.05, 0.01, -1);
    light->setCastShadows(true);

    Ogre::Light *light2 = rviz_scene_manager_->createLight("Backlight");
    light2->setType(Ogre::Light::LT_DIRECTIONAL);
    light2->setDiffuseColour(0.2, 0.25, 0.3);
    light2->setSpecularColour(1, 1, 1);
    light2->setDirection(-0.1, -0.1, 0.05);
    light2->setCastShadows(false);

    Ogre::Light *light3 = rviz_scene_manager_->createLight("Keylight");
    light3->setType(Ogre::Light::LT_DIRECTIONAL);
    light3->setDiffuseColour(0.4, 0.4, 0.4);
    light3->setSpecularColour(1, 1, 1);
    light3->setDirection(0.1, 0.1, -0.05);
    light3->setCastShadows(false);

    rviz_scene_manager_->setAmbientLight(Ogre::ColourValue(0.3, 0.3, 0.3));
    rviz_scene_manager_->setShadowColour(Ogre::ColourValue(0.3, 0.3, 0.3, 1.0));
}

void RVizViewer::InitializeMenus()
{
    menu_openrave_ = new QMenu("OpenRAVE", this);
    menu_openrave_->addAction(LoadEnvironmentAction());
    menu_environments_ = menu_openrave_->addMenu("Environments");
    menuBar()->addMenu(menu_openrave_);
}

void RVizViewer::InitializeInteractiveMarkers()
{
    markers_display_ = dynamic_cast<::rviz::InteractiveMarkerDisplay *>(
        rviz_manager_->createDisplay("rviz/InteractiveMarkers",
                                     "OpenRAVE Markers", true)
    );

    std::string const update_topic = str(format("%s/update") % topic_name_);
    markers_display_->setTopic(QString::fromStdString(update_topic), "");
}

QAction *RVizViewer::LoadEnvironmentAction()
{
    QAction* toReturn = new QAction("Load", this);
    connect(toReturn, SIGNAL(triggered(bool)), this, SLOT(LoadEnvironmentSlot()));
    return toReturn;
}

std::string RVizViewer::GenerateTopicName(std::string const &base_name,
                                          bool anonymize) const
{
    if (anonymize) {
        return str(format("%s_%p") % base_name % this);
    } else {
        return base_name;
    }
}

void RVizViewer::SetCamera(Ogre::Camera *camera,
                           OpenRAVE::RaveTransform<float> &trans,
                           float focalDistance) const
{
    camera->setPosition(util::ToOgreVector(trans.trans));
    camera->setOrientation(util::ToOgreQuaternion(trans.rot));
    camera->setFocalLength(std::max(focalDistance, 0.01f));
}

}
