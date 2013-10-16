#include "OpenRaveRviz.h"
//#include "RenderWindow.h"
#include <qapplication.h>
#include <openrave/config.h>
#include <QDockWidget>
#include <OgreCamera.h>
#include "Converters.h"
#include <rviz/display.h>
#include <rviz/display_wrapper.h>
#include <rviz/default_plugin/marker_display.h>
#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/default_plugin/markers/shape_marker.h>
#include <visualization_msgs/Marker.h>
#include <rviz/default_plugin/grid_display.h>
#include <rviz/ogre_helpers/grid.h>
#include <rviz/ogre_helpers/render_system.h>
#include <OgreSceneManager.h>
#include <qtimer.h>
#include <openrave/plugin.h>
#include <rviz/displays_panel.h>
#include <rviz/tool_properties_panel.h>
#include <QFileDialog>
#include <QMenuBar>
#include <QMessageBox>

using namespace OpenRAVE;
using namespace rviz;


namespace or_rviz
{



    OpenRaveRviz::OpenRaveRviz(OpenRAVE::EnvironmentBasePtr env, QWidget * parent, Qt::WindowFlags flags) :
            VisualizationFrame(parent),
            OpenRAVE::ViewerBase(env),
            m_rvizManager(NULL),
            m_mainRenderPanel(NULL),
            m_autoSync(false),
            m_name("or_rviz")
    {
        SetCurrentViewEnv(env);
        setWindowTitle("Openrave Rviz Viewer[*]");
        setUpdatesEnabled(true);

        QTimer* timer = new QTimer(this);
        timer->setInterval(33);
        timer->setSingleShot(false);
        timer->start();

        connect(timer, SIGNAL(timeout()), this, SLOT(syncUpdate()));

        initialize();

        QMenu* openRaveMenu = new QMenu("OpenRAVE", this);
        openRaveMenu->addAction(LoadEnvironmentAction());
        m_environmentsMenu = openRaveMenu->addMenu("Environments");

        this->menuBar()->addMenu(openRaveMenu);

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

    }

    QAction* OpenRaveRviz::LoadEnvironmentAction()
    {
        QAction* toReturn = new QAction("Load", this);
        connect(toReturn, SIGNAL(triggered(bool)), this, SLOT(loadEnvironment()));
        return toReturn;
    }

    void OpenRaveRviz::loadEnvironment()
    {
        QString file = QFileDialog::getOpenFileName(this, "Load", ".");
        if(file.count() > 0)
        {
            if(!GetCurrentViewEnv()->Load(file.toStdString()))
            {
                QMessageBox::warning(this, "Load", "Failed to load objects!");
            }
        }
    }

    OpenRaveRviz::~OpenRaveRviz()
    {
        for(std::map<std::string, KinBodyDisplay*>::iterator it = m_kinBodies.begin(); it != m_kinBodies.end(); it++)
        {
            delete it->second;
        }

        m_kinBodies.clear();
    }

    int OpenRaveRviz::main(bool showWindow)
    {
        qApp->setActiveWindow(this);

        if(showWindow)
        {
            show();
        }

        return qApp->exec();
    }

    void OpenRaveRviz::quitmainloop()
    {
        qApp->quit();
    }

    void OpenRaveRviz::Reset()
    {

    }


    void OpenRaveRviz::SetBkgndColor(const OpenRAVE::RaveVector<float> &color)
    {
        getManager()->getRenderPanel()->setBackgroundColor(Ogre::ColourValue(color.x, color.y, color.z));
    }

    // registers a function with the viewer that gets called everytime mouse button is clicked
    OpenRAVE::UserDataPtr OpenRaveRviz::RegisterItemSelectionCallback(const ItemSelectionCallbackFn &fncallback)
    {
        // TODO: Implement
        return OpenRAVE::UserDataPtr();
    }

    // registers a function with the viewer that gets called for every new image rendered.
    OpenRAVE::UserDataPtr OpenRaveRviz::RegisterViewerImageCallback(const ViewerImageCallbackFn &fncallback)
    {
        //TODO: Implement
        return OpenRAVE::UserDataPtr();
    }


    // registers a function with the viewer that gets called in the viewer's GUI thread for every cycle the viewer refreshes at
    OpenRAVE::UserDataPtr OpenRaveRviz::RegisterViewerThreadCallback(const ViewerThreadCallbackFn &fncallback)
    {
        //TODO: Implement
        return OpenRAVE::UserDataPtr();
    }

    // controls whether the viewer synchronizes with the newest environment automatically
    void OpenRaveRviz::SetEnvironmentSync(bool update)
    {
        SetAutoSync(update);
    }


    std::string OpenRaveRviz::GetEnvironmentHash(OpenRAVE::EnvironmentBasePtr env)
    {
        std::stringstream ss;
        ss << RaveGetEnvironmentId(env);
        return ss.str();
    }

    void OpenRaveRviz::setEnvironment(bool checked)
    {
        if(!checked)
        {
            return;
        }

        QAction* action = dynamic_cast<QAction*>(sender());

        if(action)
        {
            EnvironmentBasePtr environmentBase = RaveGetEnvironment(action->text().toInt());

            if(environmentBase && environmentBase != GetCurrentViewEnv())
            {
                Clear();
                environmentBase->GetMutex().lock();
                SetCurrentViewEnv(environmentBase);
                environmentBase->GetMutex().unlock();


            }
        }

    }

    // forces synchronization with the environment, returns when the environment is fully synchronized.
    void OpenRaveRviz::EnvironmentSync()
    {
        setWindowTitle("Openrave Rviz Viewer[*]");
        GetCurrentViewEnv()->GetMutex().lock();

        std::vector<OpenRAVE::KinBodyPtr> bodies;
        GetCurrentViewEnv()->GetBodies(bodies);

        for(size_t i = 0; i < bodies.size(); i++)
        {
            if(!HasKinBody(bodies[i]->GetName()))
            {
                if(getManager())
                {
                    rviz::DisplayWrapper* displayWrapper = getManager()->getDisplayWrapper(bodies[i]->GetName());

                    if(!displayWrapper)
                    {
                        displayWrapper = getManager()->createDisplay("or_rviz/KinBody", bodies[i]->GetName(), true);
                    }

                    if(!displayWrapper)
                    {
                        continue;
                    }

                    KinBodyDisplay* display = dynamic_cast<KinBodyDisplay*>(displayWrapper->getDisplay());

                    if(!display)
                    {
                        continue;
                    }

                    display->CreateVisual(bodies[i], getManager()->getSceneManager());
                    m_kinBodies[bodies[i]->GetName()] = display;
                }
            }
            else
            {
                m_kinBodies[bodies[i]->GetName()]->UpdateTransforms();
            }
        }

        std::vector<std::string> removals;
        for(std::map<std::string, KinBodyDisplay*>::iterator it = m_kinBodies.begin(); it != m_kinBodies.end(); it++)
        {
            bool containsBody = false;
            for(size_t j = 0; j < bodies.size(); j++)
            {
                if(bodies[j]->GetName() == it->first)
                {
                    containsBody = true;
                    break;
                }
            }

            if(!containsBody)
            {
                removals.push_back(it->first);
            }
        }

        for(size_t i = 0; i < removals.size(); i++)
        {
            RemoveKinBody(removals[i]);
        }

        std::list<OpenRAVE::EnvironmentBasePtr> envs;
        RaveGetEnvironments(envs);


        std::vector<QAction*> actionRemovals;
        for(int j = 0; j < m_environmentsMenu->actions().count(); j++)
        {
            QAction* action = m_environmentsMenu->actions().at(j);



            if(!RaveGetEnvironment(action->text().toInt()))
            {
                actionRemovals.push_back(action);
            }
            else
            {
                if(RaveGetEnvironmentId(GetCurrentViewEnv()) != action->text().toInt())
                {
                    action->setChecked(false);
                }
                else
                {
                    action->setChecked(true);
                }
            }
        }

        for(size_t j = 0; j < actionRemovals.size(); j++)
        {
            m_environmentsMenu->removeAction(actionRemovals.at(j));
        }

        for(std::list<OpenRAVE::EnvironmentBasePtr>::iterator it = envs.begin(); it != envs.end(); it++)
        {
            OpenRAVE::EnvironmentBasePtr& env = *it;
            std::string name = GetEnvironmentHash(env);

            bool hasAction = false;

            for(int j = 0; j < m_environmentsMenu->actions().count(); j++)
            {
                QAction* action = m_environmentsMenu->actions().at(j);

                if(action->text().toStdString() == name)
                {
                    hasAction = true;
                    break;
                }
            }

            if(!hasAction)
            {
                QAction* action = m_environmentsMenu->addAction(QString::fromStdString(name));
                action->setCheckable(true);
                connect(action, SIGNAL(triggered(bool)), this, SLOT(setEnvironment(bool)));
            }
        }

        GetCurrentViewEnv()->GetMutex().unlock();

    }

    // Viewer size and position can be set outside in the
    // OpenRAVE API
    void OpenRaveRviz::SetSize (int w, int h)
    {
        resize(w, h);
    }

    void OpenRaveRviz::Move (int x, int y)
    {
        move(x, y);
    }

    // Name is set by OpenRAVE outside
    void OpenRaveRviz::SetName (const std::string &name)
    {
        m_name = name;
    }

    const std::string & OpenRaveRviz::GetName () const
    {
        return m_name;
    }

    // Keeps camera transform consistent
    void  OpenRaveRviz::UpdateCameraTransform()
    {
        //TODO: Implement
    }

    // Set the camera transformation.
    void OpenRaveRviz::SetCamera (const OpenRAVE::RaveTransform<float> &trans, float focalDistance)
    {
        Ogre::Camera* camera = getManager()->getRenderPanel()->getCamera();
        camera->setPosition(converters::ToOgreVector(trans.trans));
        camera->setOrientation(converters::ToOgreQuaternion(trans.rot));
        camera->setFocalLength(std::max<float>(focalDistance, 0.01f));

    }

    // Return the current camera transform that the viewer is rendering the environment at.
    OpenRAVE::RaveTransform<float>  OpenRaveRviz::GetCameraTransform() const
    {
        OpenRAVE::RaveTransform<float> toReturn;
        if(m_rvizManager)
        {
            Ogre::Camera* camera = m_rvizManager->getRenderPanel()->getCamera();
            toReturn.trans = converters::ToRaveVector(camera->getPosition());
            toReturn.rot = converters::ToRaveQuaternion(camera->getOrientation());
        }
        return toReturn;
    }

    // Return the closest camera intrinsics that the viewer is rendering the environment at.
    OpenRAVE::geometry::RaveCameraIntrinsics<float> OpenRaveRviz::GetCameraIntrinsics()
    {
        OpenRAVE::geometry::RaveCameraIntrinsics<float> toReturn;
        Ogre::Camera* camera = getManager()->getRenderPanel()->getCamera();
        Ogre::Matrix4 projectionMatrix = camera->getProjectionMatrix();
        toReturn.focal_length = camera->getFocalLength();
        toReturn.fx = projectionMatrix[0][0];
        toReturn.fy = projectionMatrix[1][1];
        toReturn.cx = projectionMatrix[0][2];
        toReturn.cy = projectionMatrix[1][2];
        toReturn.distortion_model = "";
        return toReturn;
    }

    // Renders a 24bit RGB image of dimensions width and height from the current scene.
    bool OpenRaveRviz::GetCameraImage(std::vector<uint8_t> &memory, int width, int height, const OpenRAVE::RaveTransform<float> &t, const OpenRAVE::SensorBase::CameraIntrinsics &intrinsics)
    {
        //TODO: Implement
        return false;
    }


    // Overloading OPENRAVE drawing functions....
    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector< float > &color, int drawstyle)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const float *colors, int drawstyle, bool bhasalpha)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinelist (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinelist (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawarrow (const OpenRAVE::RaveVector< float > &p1, const OpenRAVE::RaveVector< float > &p2, float fwidth, const OpenRAVE::RaveVector< float > &color)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawbox (const OpenRAVE::RaveVector< float > &vpos, const OpenRAVE::RaveVector< float > &vextents)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawplane (const OpenRAVE::RaveTransform< float > &tplane, const OpenRAVE::RaveVector< float > &vextents, const boost::multi_array< float, 3 > &vtexture)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const OpenRAVE::RaveVector< float > &color)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array< float, 2 > &colors)
    {
        //TODO: Implement
        return OpenRAVE::GraphHandlePtr();
    }

    void OpenRaveRviz::Clear()
    {
        std::vector<std::string> removals;
        for(std::map<std::string, KinBodyDisplay*>::iterator it = m_kinBodies.begin(); it != m_kinBodies.end(); it++)
         {
            removals.push_back(it->first);
         }

        for(size_t i = 0; i < removals.size(); i++)
        {
            RemoveKinBody(removals.at(i));
        }
    }

    void OpenRaveRviz::RemoveKinBody(const std::string& bodyName)
    {
        m_kinBodies.erase(bodyName);
        getManager()->removeDisplay(bodyName);
    }

    void OpenRaveRviz::RemoveKinBody(OpenRAVE::KinBodyPtr kinBody)
    {
        RemoveKinBody(kinBody->GetName());
    }

    void OpenRaveRviz::syncUpdate()
    {
        EnvironmentSync();
    }
}

static char* argv[1] = {const_cast<char *>("or_rviz")};
static int argc = 1;


OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if (type == OpenRAVE::PT_Viewer && interfacename == "or_rviz")
    {

        if (!ros::isInitialized())
        {
            ros::init(argc, argv, "or_rviz", ros::init_options::AnonymousName);
        }
        else
        {
            RAVELOG_DEBUG("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
        }
        new QApplication(argc, argv);
        return OpenRAVE::InterfaceBasePtr(new or_rviz::OpenRaveRviz(penv));
    }

    return OpenRAVE::InterfaceBasePtr();
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_Viewer].push_back("or_rviz");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}

