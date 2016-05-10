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
#include <OgreRenderWindow.h>
#include <OgreHardwarePixelBuffer.h>
#include <boost/format.hpp>
#include <rviz/display_group.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include "util/ogre_conversions.h"
#include "util/ros_conversions.h"
#include "util/ScopedConnection.h"
#include "RVizViewer.h"

using boost::format;
using boost::str;

static double const kRefreshRate = 30;
static std::string const kOffscreenCameraName = "OffscreenCamera";
static std::string const kInteractiveMarkersDisplayName = "OpenRAVE Markers";
static std::string const kEnvironmentDisplayName = "OpenRAVE Environment";

namespace or_rviz {

/*
 * Helpers
 */
namespace detail {

OffscreenRenderRequest::OffscreenRenderRequest()
    : done(false)
    , width(0)
    , height(0)
    , depth(0)
    , memory(NULL)
{
}

template <typename T>
T *getOrCreateDisplay(::rviz::VisualizationManager *manager,
                      std::string const &class_name,
                      std::string const &name,
                      bool enabled)
{
    ::rviz::DisplayGroup *display_group = manager->getRootDisplayGroup();
    BOOST_ASSERT(display_group);

    // Search for an existing display with the same name.
    ::rviz::Display *this_display = NULL;
    for (size_t i = 0; i < display_group->numDisplays(); ++i) {
        ::rviz::Display *display = display_group->getDisplayAt(i);
        BOOST_ASSERT(display);

        if (display->getNameStd() == name) {
            this_display = display;
            break;
        }
    }

    // Update the existing display.
    if (this_display) {
        std::string const matched_class = this_display->getClassId().toStdString();

        if (matched_class != class_name) {
            throw OpenRAVE::openrave_exception(
                str(format(
                    "Unable to create RViz display '%s' of type '%s': There is"
                    " already a display of type '%s' with this name. Try deleting"
                    " '$HOME/.rviz/config'.\n")
                        % name % class_name % matched_class),
                OpenRAVE::ORE_InvalidState
            );
        }

        this_display->setEnabled(enabled);
        RAVELOG_DEBUG("Re-using existing RViz diplay '%s' of type '%s'.\n",
            name.c_str(), class_name.c_str()
        );
    } else {
        this_display = manager->createDisplay(
            QString::fromStdString(class_name),
            QString::fromStdString(name),
            true
        );
        RAVELOG_DEBUG("Creating new RViz diplay '%s' of type '%s'.\n",
            name.c_str(), class_name.c_str()
        );
    }

    // Cast to the display subclass. Ideally, we would use a dynamic_cast here
    // to check that the Display is of the correct type. Unfortunately, that is
    // not possible because recent builds of RViz do not include RTTI. And,
    // even if they did, RTTI does not work reliably between shared objects.
    return static_cast<T *>(this_display);
}

}

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

    markers_display_ = InitializeInteractiveMarkers();
    environment_display_ = InitializeEnvironmentDisplay(env);
    InitializeOffscreenRendering();
    InitializeLighting();
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

void RVizViewer::EnvironmentSync()
{
    InteractiveMarkerViewer::EnvironmentSync();
    environment_display_->EnvironmentSync();
}

void RVizViewer::SetBkgndColor(OpenRAVE::RaveVector<float> const &color)
{
    render_panel_->setBackgroundColor(
        Ogre::ColourValue(color.x, color.y, color.z));
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

OpenRAVE::UserDataPtr RVizViewer::RegisterViewerImageCallback(
        OpenRAVE::ViewerBase::ViewerImageCallbackFn const &cb)
{
    boost::signals2::connection const con = viewer_image_callbacks_.connect(cb);
    return boost::make_shared<util::ScopedConnection>(con);
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
    pose.trans = util::toORVector<float>(camera->getPosition());
    pose.rot = util::toORQuaternion<float>(camera->getOrientation());
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

bool RVizViewer::GetCameraImage(
        std::vector<uint8_t> &memory, int width, int height,
        OpenRAVE::RaveTransform<float> const &t,
        OpenRAVE::SensorBase::CameraIntrinsics const &intrinsics)
{
    static int const depth = 24;

    BOOST_ASSERT(width > 0);
    BOOST_ASSERT(height > 0);
    BOOST_ASSERT(depth >= 8 && depth % 8 == 0);

    memory.resize(width * height * (depth / 8), 0x00);

    detail::OffscreenRenderRequest request;
    request.done = false;
    request.width = width;
    request.height = height;
    request.depth = depth;
    request.memory = &memory.front();
    request.extrinsics = t;
    request.intrinsics = intrinsics;

    RAVELOG_DEBUG("Submitting OffscreenRenderRequest(%p).\n", &request);
    {
        // Make the request.
        boost::mutex::scoped_lock lock(offscreen_mutex_);
        offscreen_requests_.push_back(&request);

        // Wait for the request to finish. At this point, the output buffer has
        // been populated by the render thread.
        while (!request.done) {
            offscreen_condition_.wait(lock);
        }
    }
    RAVELOG_DEBUG("Completed OffscreenRenderRequest(%p).\n", &request);

    return true;
}

void RVizViewer::ProcessOffscreenRenderRequests()
{
    while (!offscreen_requests_.empty()) {
        detail::OffscreenRenderRequest *request;
        {
            boost::mutex::scoped_lock lock(offscreen_mutex_);
            request = offscreen_requests_.front();
            offscreen_requests_.pop_front();
        }

        RAVELOG_DEBUG(
            "Processing OffscreenRenderRequest(%p): size = [%d x %d],"
            " depth = %d, focal_length = %f.\n",
            request, request->width, request->height,
            request->depth, request->intrinsics.focal_length
        );

        // Setup the camera.
        //float const &focal_length = request->intrinsics.focal_length;
        float const focal_length = 0.785;
        SetCamera(offscreen_camera_, request->extrinsics, focal_length);

#if 0
        offscreen_camera_->setNearClipDistance(focal_length);
        offscreen_camera_->setFarClipDistance(focal_length * 10000);
        offscreen_camera_->setAspectRatio(
              (request->intrinsics.fy / static_cast<float>(request->height))
            / (request->intrinsics.fx / static_cast<float>(request->width))
        );
        offscreen_camera_->setFOVy(
            Ogre::Radian(2.0f * std::atan(
                0.5f * request->height / request->intrinsics.fy))
        );
#endif

        // Render the texture into a texture.
        std::string const name = str(format("offscreen[%p]") % request);
        Ogre::PixelFormat const pixel_format = GetPixelFormat(request->depth);

        Ogre::TexturePtr const texture
            = Ogre::TextureManager::getSingleton().createManual(
                name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                Ogre::TEX_TYPE_2D, request->width, request->height, 0,
                pixel_format, Ogre::TU_RENDERTARGET
        );
        BOOST_ASSERT(!texture.isNull());

        // Render into the texture.
        Ogre::RenderTexture *render_texture
            = texture->getBuffer()->getRenderTarget();
        BOOST_ASSERT(render_texture);
        render_texture->addViewport(offscreen_camera_);

        // Copy the texture into the output buffer.
        Ogre::Box const extents(0, 0,  request->width, request->height);
        Ogre::PixelBox const pb(extents, pixel_format, request->memory);
        render_texture->update();
        render_texture->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);

        // Delete the texture.
        Ogre::TextureManager::getSingleton().unload(name);
        Ogre::TextureManager::getSingleton().remove(name);

        {
            boost::mutex::scoped_lock lock(offscreen_mutex_);
            request->done = true;
            offscreen_condition_.notify_all();
        }
    }
}

bool RVizViewer::eventFilter(QObject *o, QEvent *e)
{
    bool result = ::rviz::VisualizationFrame::eventFilter(o, e);

    if (e->type() == QEvent::Paint) {
        if (!viewer_image_callbacks_.empty()) {
            int width, height, bytes_per_pixel;
            unsigned char *data = WriteCurrentView(&width, &height,
                                                   &bytes_per_pixel);
            viewer_image_callbacks_(data, width, height, bytes_per_pixel);
        }
    }

    return result;
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

        ProcessOffscreenRenderRequests();

        viewer_callbacks_();
    }
}


/*
 * Private
 */
void RVizViewer::InitializeLighting()
{
    rviz_scene_manager_->setShadowTechnique(Ogre::SHADOWTYPE_NONE);

    Ogre::Light* mainDirectional = rviz_scene_manager_->getLight( "MainDirectional" );
    mainDirectional->setCastShadows(false);

    Ogre::Light *light = rviz_scene_manager_->createLight("FillLight");
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setDiffuseColour(0.6, 0.55, 0.5);
    light->setSpecularColour(1, 1, 1);
    light->setDirection(0.05, 0.01, -1);
    light->setCastShadows(true);

    Ogre::Light *light2 = rviz_scene_manager_->createLight("BackLight");
    light2->setType(Ogre::Light::LT_DIRECTIONAL);
    light2->setDiffuseColour(0.2, 0.25, 0.3);
    light2->setSpecularColour(1, 1, 1);
    light2->setDirection(-0.1, -0.1, 0.05);
    light2->setCastShadows(false);

    Ogre::Light *light3 = rviz_scene_manager_->createLight("KeyLight");
    light3->setType(Ogre::Light::LT_DIRECTIONAL);
    light3->setDiffuseColour(0.4, 0.4, 0.4);
    light3->setSpecularColour(1, 1, 1);
    light3->setDirection(0.1, 0.1, -0.05);
    light3->setCastShadows(false);

    rviz_scene_manager_->setAmbientLight(Ogre::ColourValue(0.3, 0.3, 0.3));
    rviz_scene_manager_->setShadowColour(Ogre::ColourValue(0.3, 0.3, 0.3, 1.0));
}

void RVizViewer::InitializeOffscreenRendering()
{
    offscreen_panel_ = new ::rviz::RenderPanel(this);
    offscreen_panel_->setVisible(false);
    offscreen_main_panel_ = offscreen_panel_->getRenderWindow();
    offscreen_main_panel_->setVisible(false);

    offscreen_camera_ = rviz_scene_manager_->createCamera(kOffscreenCameraName);
}

void RVizViewer::InitializeMenus()
{
    menu_openrave_ = new QMenu("OpenRAVE", this);
    menu_openrave_->addAction(LoadEnvironmentAction());
    menu_environments_ = menu_openrave_->addMenu("Environments");
    menuBar()->addMenu(menu_openrave_);
}

::rviz::InteractiveMarkerDisplay *RVizViewer::InitializeInteractiveMarkers()
{
    auto *const display =
        detail::getOrCreateDisplay< ::rviz::InteractiveMarkerDisplay>(
            rviz_manager_, "rviz/InteractiveMarkers",
            kInteractiveMarkersDisplayName, true);

    std::string const update_topic = str(format("%s/update") % topic_name_);
    display->setTopic(QString::fromStdString(update_topic), "");

    return display;
}

rviz::EnvironmentDisplay *RVizViewer::InitializeEnvironmentDisplay(
    OpenRAVE::EnvironmentBasePtr const &env)
{
    auto *const display =
        detail::getOrCreateDisplay<rviz::EnvironmentDisplay>(
            rviz_manager_, "or_rviz::rviz::EnvironmentDisplay",
            kEnvironmentDisplayName, true);

    //display->set_environment(env);
    set_parent_frame(rviz_manager_->getFixedFrame().toStdString());
    environment_change_handle_
        = display->RegisterEnvironmentChangeCallback(
            boost::bind(&RVizViewer::set_environment, this, _1));

    return display;
}

QAction *RVizViewer::LoadEnvironmentAction()
{
    QAction* toReturn = new QAction("Load", this);
    connect(toReturn, SIGNAL(triggered(bool)), this, SLOT(LoadEnvironmentSlot()));
    return toReturn;
}

unsigned char *RVizViewer::WriteCurrentView(int *width, int *height, int *depth)
{
    BOOST_ASSERT(width && height && depth);

    int left, top;
    render_panel_->getViewport()->getActualDimensions(left, top, *width, *height);

    Ogre::PixelFormat format = Ogre::PF_BYTE_RGBA;
    int outWidth = *width;
    int outHeight = *height;
    *depth = Ogre::PixelUtil::getNumElemBytes(format);

    unsigned char *data = new unsigned char[outWidth * outHeight * *depth];
    Ogre::Box extents(left, top, left + *width, top + *height);
    Ogre::PixelBox pb(extents, format, data);

    render_panel_->getRenderWindow()->copyContentsToMemory(
        pb, Ogre::RenderTarget::FB_AUTO);

    return data;
}

Ogre::PixelFormat RVizViewer::GetPixelFormat(int depth) const
{
    switch (depth) {
    case 8:
        return Ogre::PF_L8;
    case 16:
        return Ogre::PF_FLOAT16_GR;
    case 24:
        return Ogre::PF_R8G8B8;
    case 32:
        return Ogre::PF_R8G8B8A8;
    default:
        RAVELOG_ERROR(
            "Error: Unsupported depth %d. Supported depths: 8 (gray byte),"
            " 16 (float16 gray), 24 (RGB bytes), 32 (RGBA bytes)\n",
            depth
        );
        return Ogre::PF_R8G8B8;
    }
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
                           OpenRAVE::RaveTransform<float> const &trans,
                           float focalDistance) const
{
    camera->setPosition(util::toOgreVector(trans.trans));
    camera->setOrientation(util::toOgreQuaternion(trans.rot));
    camera->setFocalLength(std::max(focalDistance, 0.01f));
}

}
