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
    RVizViewer(OpenRAVE::EnvironmentBasePtr env);

    int main(bool bShow);
    void quitmainloop();

public Q_SLOTS:
    void LoadEnvironmentSlot();
    void EnvironmentSyncSlot();

private:
    rviz::VisualizationManager *rviz_manager_;
    rviz::RenderPanel *rviz_main_panel_;
    rviz::InteractiveMarkerDisplay *markers_display_;

    QTimer *timer_;
    QMenu *menu_openrave_;
    QMenu *menu_environments_;

    void InitializeMenus();
    void InitializeLighting();
    void InitializeInteractiveMarkers();

    QAction *LoadEnvironmentAction();
};

}

#endif
