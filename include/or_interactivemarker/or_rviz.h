#ifndef ORRVIZ_H_
#define ORRVIZ_H_
#include <QAction>
#include <QMenu>
#include <QTimer>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/visualization_frame.h>
#include "or_interactivemarker.h"

namespace or_interactivemarker {

class RVizViewer : public rviz::VisualizationFrame,
                   public InteractiveMarkerViewer {
    Q_OBJECT

public:
    RVizViewer(OpenRAVE::EnvironmentBasePtr env,
               std::string const &topic_name, bool anonymize);

    int main(bool bShow);
    void quitmainloop();

    virtual void SetSize(int w, int h);
    virtual void Move(int x, int y);

    virtual std::string const &GetName() const;
    virtual void SetName(std::string const &name);

    virtual OpenRAVE::RaveTransform<float> GetCameraTransform() const;
    virtual OpenRAVE::geometry::RaveCameraIntrinsics<float> GetCameraIntrinsics() const;
    virtual void SetCamera(OpenRAVE::RaveTransform<float> &trans, float focalDistance = 0);

public Q_SLOTS:
    void LoadEnvironmentSlot();
    void EnvironmentSyncSlot();

private:
    rviz::VisualizationManager *rviz_manager_;
    rviz::RenderPanel *rviz_main_panel_;
    Ogre::SceneManager *rviz_scene_manager_;
    rviz::InteractiveMarkerDisplay *markers_display_;

    Ogre::Camera *offscreen_camera_;

    QTimer *timer_;
    QMenu *menu_openrave_;
    QMenu *menu_environments_;

    mutable std::string window_title_;

    void InitializeMenus();
    void InitializeLighting();
    void InitializeInteractiveMarkers();

    QAction *LoadEnvironmentAction();

    std::string GenerateTopicName(std::string const &base_name, bool anonymize) const;
    virtual void SetCamera(Ogre::Camera *camera, OpenRAVE::RaveTransform<float> &trans,
                           float focalDistance) const;

};

}

#endif
