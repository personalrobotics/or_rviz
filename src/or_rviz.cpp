#include <qapplication.h>
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QMenuBar>
#include <QString>
#include <QTimer>
#include <rviz/visualization_manager.h>
#include "or_rviz.h"

static double const kRefreshRate = 30;

namespace or_interactivemarker {

RVizViewer::RVizViewer(OpenRAVE::EnvironmentBasePtr env)
    : InteractiveMarkerViewer(env)
    , timer_(NULL)
{
    initialize();

    rviz_manager_ = getManager();
    rviz_main_panel_ = rviz_manager_->getRenderPanel();

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
    Ogre::SceneManager *scene_manager = rviz_manager_->getSceneManager();

    scene_manager->setShadowTechnique(Ogre::SHADOWTYPE_NONE);

    Ogre::Light *light = scene_manager->createLight("FillLight");
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setDiffuseColour(0.6, 0.55, 0.5);
    light->setSpecularColour(1, 1, 1);
    light->setDirection(0.05, 0.01, -1);
    light->setCastShadows(true);

    Ogre::Light *light2 = scene_manager->createLight("Backlight");
    light2->setType(Ogre::Light::LT_DIRECTIONAL);
    light2->setDiffuseColour(0.2, 0.25, 0.3);
    light2->setSpecularColour(1, 1, 1);
    light2->setDirection(-0.1, -0.1, 0.05);
    light2->setCastShadows(false);

    Ogre::Light *light3 = scene_manager->createLight("Keylight");
    light3->setType(Ogre::Light::LT_DIRECTIONAL);
    light3->setDiffuseColour(0.4, 0.4, 0.4);
    light3->setSpecularColour(1, 1, 1);
    light3->setDirection(0.1, 0.1, -0.05);
    light3->setCastShadows(false);

    scene_manager->setAmbientLight(Ogre::ColourValue(0.3, 0.3, 0.3));
    scene_manager->setShadowColour(Ogre::ColourValue(0.3, 0.3, 0.3, 1.0));
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
    markers_display_ = dynamic_cast<rviz::InteractiveMarkerDisplay *>(
        rviz_manager_->createDisplay("rviz/InteractiveMarkers",
                                     "OpenRAVE Markers", true)
    );
    // TODO: Set this to an auto-generated string.
    markers_display_->setTopic("/openrave/update", "");
}

QAction *RVizViewer::LoadEnvironmentAction()
{
    QAction* toReturn = new QAction("Load", this);
    connect(toReturn, SIGNAL(triggered(bool)), this, SLOT(LoadEnvironmentSlot()));
    return toReturn;
}


}
