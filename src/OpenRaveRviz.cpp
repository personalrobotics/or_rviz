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

using namespace OpenRAVE;
using namespace rviz;


namespace or_rviz
{



    OpenRaveRviz::OpenRaveRviz(OpenRAVE::EnvironmentBasePtr env, QWidget * parent, Qt::WindowFlags flags) :
            QMainWindow(parent, flags),
            OpenRAVE::ViewerBase(env),
            m_rvizManager(NULL),
            m_mainRenderPanel(NULL),
            m_autoSync(false),
            m_name("Superviewer")
    {
        setWindowTitle("SuperViewer!");
        m_mainRenderPanel = new rviz::RenderPanel();

        m_rvizManager = new rviz::VisualizationManager(m_mainRenderPanel);

        setCentralWidget(m_mainRenderPanel);


        rviz::DisplaysPanel* propertyWidget = new rviz::DisplaysPanel(this);
        QDockWidget* dockWidgetProperties = new QDockWidget(this);
        dockWidgetProperties->setWidget(propertyWidget);

        addDockWidget(Qt::LeftDockWidgetArea, dockWidgetProperties);

        m_mainRenderPanel->initialize( m_rvizManager->getSceneManager(), m_rvizManager );
        m_rvizManager->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_NONE);

        m_rvizManager->initialize();
        propertyWidget->initialize(m_rvizManager);
        m_rvizManager->startUpdate();
        setUpdatesEnabled(true);

        QTimer* timer = new QTimer(this);
        timer->setInterval(33);
        timer->setSingleShot(false);
        timer->start();

        connect(timer, SIGNAL(timeout()), this, SLOT(syncUpdate()));


        // Create a Grid display.
        rviz::DisplayWrapper* wrapper = m_rvizManager->createDisplay( "rviz/Grid", "adjustable grid", true );
        ROS_ASSERT( wrapper != NULL );


        // Unwrap it.
         rviz::Display* display = wrapper->getDisplay();
         ROS_ASSERT( display != NULL );

         // Downcast it to the type we think we know it is.
         //
         // (This is one part I would like to improve in the future.  For
         // this to work currently, we need to link against the plugin
         // library containing GridDisplay (libdefault_plugin.so) in addition
         // to linking against librviz.so.  This pretty much negates the
         // benefits of the plugin architecture.)
         rviz::GridDisplay* grid_ = dynamic_cast<rviz::GridDisplay*>( display );
         ROS_ASSERT( grid_ != NULL );

         // Configure the GridDisplay the way we like it.
         grid_->setStyle( rviz::Grid::Lines); // Fat lines.
         grid_->setColor( rviz::Color( 0.1f, 0.1f, 0.1f ));

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

         SetBkgndColor(OpenRAVE::Vector(1, 1, 1));
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
        m_mainRenderPanel->setBackgroundColor(Ogre::ColourValue(color.x, color.y, color.z));
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


    // forces synchronization with the environment, returns when the environment is fully synchronized.
    void OpenRaveRviz::EnvironmentSync()
    {
        GetEnv()->GetMutex().lock();

        std::vector<OpenRAVE::KinBodyPtr> bodies;
        GetEnv()->GetBodies(bodies);

        for(size_t i = 0; i < bodies.size(); i++)
        {
            if(!HasKinBody(bodies[i]->GetName()))
            {
                rviz::DisplayWrapper* displayWrapper = m_rvizManager->createDisplay("superviewer/KinBody", bodies[i]->GetName(), true);
                KinBodyDisplay* display = dynamic_cast<KinBodyDisplay*>(displayWrapper->getDisplay());
                display->CreateVisual(bodies[i], m_rvizManager->getSceneManager());
                        //new KinBodyDisplay(bodies[i], m_rvizManager->getSceneManager());
                display->setFixedFrame("World");

                m_kinBodies[bodies[i]->GetName()] = display;
            }
            else
            {
                m_kinBodies[bodies[i]->GetName()]->UpdateTransforms();
            }
        }




        GetEnv()->GetMutex().unlock();
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
        Ogre::Camera* camera = m_mainRenderPanel->getCamera();
        camera->setPosition(converters::ToOgreVector(trans.trans));
        camera->setOrientation(converters::ToOgreQuaternion(trans.rot));
        camera->setFocalLength(std::max<float>(focalDistance, 0.01f));

        RAVELOG_INFO("Setting camera parameters: %f\n", focalDistance);
    }

    // Return the current camera transform that the viewer is rendering the environment at.
    OpenRAVE::RaveTransform<float>  OpenRaveRviz::GetCameraTransform() const
    {
        OpenRAVE::RaveTransform<float> toReturn;
        Ogre::Camera* camera = m_mainRenderPanel->getCamera();
        toReturn.trans = converters::ToRaveVector(camera->getPosition());
        toReturn.rot = converters::ToRaveQuaternion(camera->getOrientation());
        return toReturn;
    }

    // Return the closest camera intrinsics that the viewer is rendering the environment at.
    OpenRAVE::geometry::RaveCameraIntrinsics<float> OpenRaveRviz::GetCameraIntrinsics()
    {
        OpenRAVE::geometry::RaveCameraIntrinsics<float> toReturn;
        Ogre::Camera* camera = m_mainRenderPanel->getCamera();
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

    void OpenRaveRviz::RemoveKinBody(OpenRAVE::KinBodyPtr kinBody)
    {
        delete m_kinBodies[kinBody->GetName()];
        m_kinBodies.erase(kinBody->GetName());
    }

    void OpenRaveRviz::syncUpdate()
    {
        EnvironmentSync();
    }
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if( type == OpenRAVE::PT_Viewer && interfacename == "superviewer" )
    {
    RAVELOG_INFO("Creating superviewer");
    char** argv = NULL;
    int argc = 0;
    ros::init(argc, argv, "superviewer", ros::init_options::AnonymousName);
    QApplication* app = new QApplication(argc, argv);
        RAVELOG_INFO("success\n");
        return OpenRAVE::InterfaceBasePtr(new or_rviz::OpenRaveRviz(penv));
    }
    RAVELOG_INFO("Failure!\n");
    return OpenRAVE::InterfaceBasePtr();
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
    info.interfacenames[  OpenRAVE::PT_Viewer].push_back("superviewer");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}

