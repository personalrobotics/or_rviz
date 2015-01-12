#include <QMenu>
#include <QMenuBar>
#include <QTimer>
#include "or_rviz.h"

namespace or_interactivemarker {

RVizViewer::RVizViewer(OpenRAVE::EnvironmentBasePtr env)
    : InteractiveMarkerViewer(env)
{
    initialize();

#if 0
    QMenu* openRaveMenu = new QMenu("OpenRAVE", this);
    openRaveMenu->addAction(LoadEnvironmentAction());
    m_environmentsMenu = openRaveMenu->addMenu("Environments");

    menuBar()->addMenu(openRaveMenu);

    m_rvizManager = getManager();

    m_mainRenderPanel = this->getManager()->getRenderPanel();

    m_rvizManager->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_NONE);

    Ogre::Light* light = m_rvizManager->getSceneManager()->createLight("FillLight");
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setDiffuseColour(0.6, 0.55, 0.5);
    light->setSpecularColour(1, 1, 1);
    light->setDirection(0.05, 0.01, -1);
    light->setCastShadows(true);

    Ogre::Light* light2 = m_rvizManager->getSceneManager()->createLight("Backlight");
    light2->setType(Ogre::Light::LT_DIRECTIONAL);
    light2->setDiffuseColour(0.2, 0.25, 0.3);
    light2->setSpecularColour(1, 1, 1);
    light2->setDirection(-0.1, -0.1, 0.05);
    light2->setCastShadows(false);

    Ogre::Light* light3 = m_rvizManager->getSceneManager()->createLight("Keylight");
    light3->setType(Ogre::Light::LT_DIRECTIONAL);
    light3->setDiffuseColour(0.4, 0.4, 0.4);
    light3->setSpecularColour(1, 1, 1);
    light3->setDirection(0.1, 0.1, -0.05);
    light3->setCastShadows(false);

    m_rvizManager->getSceneManager()->setAmbientLight(Ogre::ColourValue(0.3, 0.3, 0.3));
    m_rvizManager->getSceneManager()->setShadowColour(Ogre::ColourValue(0.3, 0.3, 0.3, 1.0));
    installEventFilter(this);

    rviz::RenderPanel* offScreenPanel = new rviz::RenderPanel(this);

    offScreenPanel->setVisible(false);

    m_offscreenRenderer = (offScreenPanel)->getRenderWindow();
    m_offscreenRenderer->setVisible(false);
    //m_offscreenRenderer->setHidden(true);

    m_offscreenCamera = m_rvizManager->getSceneManager()->createCamera("OfscreenCamera");

    rviz::InteractiveMarkerDisplay* markerDisplay =  dynamic_cast< rviz::InteractiveMarkerDisplay*>(m_rvizManager->createDisplay("rviz/InteractiveMarkers", "OpenRAVE Markers", true));
    markerDisplay->setTopic("/openrave/update", "");
#endif
}

}
