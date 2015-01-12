#ifndef ORRVIZ_H_
#define ORRVIZ_H_
#include <rviz/visualization_frame.h>
#include "or_interactivemarker.h"

namespace or_interactivemarker {

class RVizViewer : public rviz::VisualizationFrame,
                   public InteractiveMarkerViewer {
    Q_OBJECT

public:
    RVizViewer(OpenRAVE::EnvironmentBasePtr env);

public Q_SLOTS:
    void syncUpdate();
    void loadEnvironment();
    void setEnvironment(bool checked);

private:
};

}

#endif
