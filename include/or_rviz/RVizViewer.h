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
#ifndef ORRVIZ_H_
#define ORRVIZ_H_
#include <QAction>
#include <QMenu>
#include <QTimer>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/visualization_frame.h>
#include "rviz/EnvironmentDisplay.h"
#include "InteractiveMarkerViewer.h"

namespace or_rviz {

namespace detail {

struct OffscreenRenderRequest {
    OffscreenRenderRequest();

    bool done;
    int width;
    int height;
    int depth;
    uint8_t *memory;
    OpenRAVE::RaveTransform<float> extrinsics;
    OpenRAVE::SensorBase::CameraIntrinsics intrinsics;
};

}

class RVizViewer : public ::rviz::VisualizationFrame,
                   public InteractiveMarkerViewer {
    Q_OBJECT

public:
    typedef void ViewerImageCallbackFn(uint8_t const *, int, int, int);

    RVizViewer(OpenRAVE::EnvironmentBasePtr env,
               std::string const &topic_name, bool anonymize);

    int main(bool bShow);
    void quitmainloop();

    virtual bool eventFilter(QObject *o, QEvent *e);

    virtual void EnvironmentSync();

    virtual void SetBkgndColor(OpenRAVE::RaveVector<float> const &color);
    virtual void SetSize(int w, int h);
    virtual void Move(int x, int y);

    virtual std::string const &GetName() const;
    virtual void SetName(std::string const &name);
    
    virtual OpenRAVE::UserDataPtr RegisterViewerImageCallback(
        OpenRAVE::ViewerBase::ViewerImageCallbackFn const &cb);

    virtual OpenRAVE::RaveTransform<float> GetCameraTransform() const;
    virtual OpenRAVE::geometry::RaveCameraIntrinsics<float> GetCameraIntrinsics() const;
    virtual void SetCamera(OpenRAVE::RaveTransform<float> &trans, float focalDistance = 0);
    bool GetCameraImage(std::vector<uint8_t> &memory, int width, int height,
                        OpenRAVE::RaveTransform<float> const &t,
                        OpenRAVE::SensorBase::CameraIntrinsics const &intrinsics);

public Q_SLOTS:
    void LoadEnvironmentSlot();
    void EnvironmentSyncSlot();

private:
    ::rviz::VisualizationManager *rviz_manager_;
    ::rviz::RenderPanel *rviz_main_panel_;
    Ogre::SceneManager *rviz_scene_manager_;
    ::rviz::InteractiveMarkerDisplay *markers_display_;

    rviz::EnvironmentDisplay *environment_display_;
    boost::signals2::connection environment_change_handle_;
    boost::signals2::connection environment_frame_handle_;

    boost::mutex offscreen_mutex_;
    boost::condition_variable offscreen_condition_;
    std::list<detail::OffscreenRenderRequest *> offscreen_requests_;

    ::rviz::RenderPanel *offscreen_panel_;
    Ogre::RenderWindow *offscreen_main_panel_;
    Ogre::Camera *offscreen_camera_;
    boost::signals2::signal<ViewerImageCallbackFn> viewer_image_callbacks_;

    QTimer *timer_;
    QMenu *menu_openrave_;
    QMenu *menu_environments_;

    mutable std::string window_title_;

    void InitializeMenus();
    void InitializeLighting();
    void InitializeOffscreenRendering();
    ::rviz::InteractiveMarkerDisplay *InitializeInteractiveMarkers();
    rviz::EnvironmentDisplay *InitializeEnvironmentDisplay(
        OpenRAVE::EnvironmentBasePtr const &env);

    QAction *LoadEnvironmentAction();
    
    void ProcessOffscreenRenderRequests();
    unsigned char *WriteCurrentView(int *width, int *height, int *depth);

    Ogre::PixelFormat GetPixelFormat(int depth) const;
    std::string GenerateTopicName(std::string const &base_name, bool anonymize) const;
    virtual void SetCamera(
        Ogre::Camera *camera,
        OpenRAVE::RaveTransform<float> const &trans,
        float focalDistance) const;

};

}

#endif
