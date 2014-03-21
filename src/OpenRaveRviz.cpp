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
#include <OgrePixelFormat.h>
#include <OgreManualObject.h>
#include <OgreRenderWindow.h>
#include <rviz/default_plugin/pose_display.h>
#include <rviz/default_plugin/interactive_marker_display.h>
#include <rviz/ogre_helpers/arrow.h>

using namespace OpenRAVE;
using namespace rviz;


namespace or_rviz
{

    OpenRaveRviz::OpenRaveRviz(OpenRAVE::EnvironmentBasePtr env, QWidget * parent, Qt::WindowFlags flags) :
            VisualizationFrame(parent),
            OpenRAVE::ViewerBase(env),
            m_rvizManager(NULL),
            m_mainRenderPanel(NULL),
            m_envDisplay(NULL),
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
        installEventFilter(this);

        rviz::RenderPanel* offScreenPanel = new rviz::RenderPanel(this);

        offScreenPanel->setVisible(false);

        m_offscreenRenderer = (offScreenPanel)->getRenderWindow();
        m_offscreenRenderer->setVisible(false);
        m_offscreenRenderer->setHidden(true);

        RegisterCommand("register", boost::bind(&OpenRaveRviz::RegisterMenuCallback, this, _1, _2), "register [objectName] [menuItemName] [pointer] Registers a python object with the given pointer (as a string) to the menu of a kinbody.");
        RegisterCommand("unregister", boost::bind(&OpenRaveRviz::UnRegisterMenuCallback, this, _1, _2), "register [objectName] [menuItemName] Unregisters the menu command given.");


    }

    bool OpenRaveRviz::RegisterMenuCallback(std::ostream& sout, std::istream& sinput)
    {
        std::string objectName;
        sinput >> objectName;

        std::string menuName;
        sinput >> menuName;

        std::string pyFunctionPtr;
        sinput >> pyFunctionPtr;

        return m_envDisplay->RegisterMenuCallback(objectName, menuName, pyFunctionPtr);
    }

    bool OpenRaveRviz::UnRegisterMenuCallback(std::ostream& sout, std::istream& sinput)
    {

        std::string objectName;
        sinput >> objectName;

        std::string menuName;
        sinput >> menuName;

        return m_envDisplay->UnRegisterMenuCallback(objectName, menuName);
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
        m_rvizManager->removeAllDisplays();
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


    uchar* OpenRaveRviz::OffscreenRender(int desiredWidth, int desiredHeight, int desiredDepth)
    {
        m_offscreenRenderer->getViewport(0)->setDimensions(0, 0, desiredWidth, desiredHeight);
        int left, top, width, height;
        m_offscreenRenderer->getViewport(0)->getActualDimensions(left, top, width, height);

        Ogre::PixelFormat format = Ogre::PF_BYTE_RGBA;
        int outWidth = width;
        int outHeight = height;
        int depth = Ogre::PixelUtil::getNumElemBytes(format);


        unsigned char *data = new unsigned char [outWidth * outHeight * depth];
        Ogre::Box extents(left, top, left + width, top + height);
        Ogre::PixelBox pb(extents, format, data);

        m_offscreenRenderer->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);


        return data;
    }

    uchar* OpenRaveRviz::WriteCurrentView(int& width, int& height, int& depth)
    {
        int left, top;
        render_panel_->getViewport()->getActualDimensions(left, top, width, height);

        Ogre::PixelFormat format = Ogre::PF_BYTE_RGBA;
        int outWidth = width;
        int outHeight = height;
        depth = Ogre::PixelUtil::getNumElemBytes(format);


        unsigned char *data = new unsigned char [outWidth * outHeight * depth];
        Ogre::Box extents(left, top, left + width, top + height);
        Ogre::PixelBox pb(extents, format, data);

        render_panel_->getRenderWindow()->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);


        return data;
    }

    void  OpenRaveRviz::paintEvent(QPaintEvent* e)
    {
        rviz::VisualizationFrame::paintEvent(e);
    }

    bool OpenRaveRviz::eventFilter(QObject *o, QEvent *e)
    {
        if (e->type() == QEvent::Paint)
        {
            paintEvent((QPaintEvent *) e);

            if(m_renderCallbacks.size() > 0)
            {
                int width, height, outBytesPerPixel;
                unsigned char *data = WriteCurrentView(width, height, outBytesPerPixel);

                for(std::map<size_t, ViewerImageCallbackFn>::iterator it = m_renderCallbacks.begin(); it != m_renderCallbacks.end(); it++)
                {
                    ViewerImageCallbackFn& fn = it->second;
                    fn(data, width, height, outBytesPerPixel);
                }
            }
        }
        return rviz::VisualizationFrame::eventFilter(o, e);
    }

    void OpenRaveRviz::Reset()
    {
        m_rvizManager->removeDisplay(m_envDisplay->getName());
        m_envDisplay = NULL;
    }


    void OpenRaveRviz::SetBkgndColor(const OpenRAVE::RaveVector<float> &color)
    {
        m_rvizManager->setBackgroundColor(rviz::Color(color.x, color.y, color.z));
    }

    // registers a function with the viewer that gets called everytime mouse button is clicked
    OpenRAVE::UserDataPtr OpenRaveRviz::RegisterItemSelectionCallback(const ItemSelectionCallbackFn &fncallback)
    {
        static size_t maxID = 0;
        maxID++;
        m_itemCallbacks[maxID] = fncallback;
        return  OpenRAVE::UserDataPtr((new SelectCallbackRegistry(maxID, &m_itemCallbacks)));
    }

    // registers a function with the viewer that gets called for every new image rendered.
    OpenRAVE::UserDataPtr OpenRaveRviz::RegisterViewerImageCallback(const ViewerImageCallbackFn &fncallback)
    {
        static size_t maxID = 0;
        maxID++;
        m_renderCallbacks[maxID] = fncallback;
        return  OpenRAVE::UserDataPtr((new ViewerCallbackRegistry(maxID, &m_renderCallbacks)));
    }


    // registers a function with the viewer that gets called in the viewer's GUI thread for every cycle the viewer refreshes at
    OpenRAVE::UserDataPtr OpenRaveRviz::RegisterViewerThreadCallback(const ViewerThreadCallbackFn &fncallback)
    {
        static size_t maxID = 0;
        maxID++;
        m_syncCallbacks[maxID] = fncallback;
        return  OpenRAVE::UserDataPtr((new SyncCallbackRegistry(maxID, &m_syncCallbacks)));
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
                OpenRAVE::EnvironmentMutex::scoped_lock environment_lock(environmentBase->GetMutex());
                SetCurrentViewEnv(environmentBase);
            }
        }

    }


    void OpenRaveRviz::RemoveKinBody (OpenRAVE::KinBodyPtr pbody)
    {
        if(pbody && m_envDisplay)
        {
            OpenRAVE::EnvironmentMutex::scoped_lock environment_lock(pbody->GetEnv()->GetMutex());
            m_envDisplay->RemoveKinBody(pbody->GetName());
        }
    }

    void OpenRaveRviz::UpdateDisplay()
    {
        if(!m_rvizManager)
        {
            return;
        }

        if (!m_envDisplay)
        {
            rviz::DisplayWrapper* wrapper = m_rvizManager->getDisplayWrapper("OpenRAVE");

            if (!wrapper)
            {
                wrapper = m_rvizManager->createDisplay("or_rviz/Environment", "OpenRAVE", true);
            }

            m_envDisplay = dynamic_cast<EnvironmentDisplay*>(wrapper->getDisplay());



            rviz::DisplayWrapper* interactiveWrapper = m_rvizManager->getDisplayWrapper("OpenRAVEInteraction");

            if (!interactiveWrapper)
            {
                interactiveWrapper = m_rvizManager->createDisplay("rviz/InteractiveMarker", "OpenRAVEInteraction", true);
            }
            rviz::InteractiveMarkerDisplay* markerDisplay = dynamic_cast<InteractiveMarkerDisplay*>(interactiveWrapper->getDisplay());
            markerDisplay->setMarkerUpdateTopic("openrave_markers/update");
        }

        if (m_envDisplay)
        {
            if (m_envDisplay->GetEnvironment() != GetCurrentViewEnv())
            {
                m_envDisplay->SetEnvironment(GetCurrentViewEnv());
            }

            m_envDisplay->UpdateObjects();
        }


        for(size_t i = 0; i < m_graphsToInitialize.size(); i++)
        {
            dynamic_cast<RvizGraphHandle*>(m_graphsToInitialize[i].get())->Initialize();
        }

        m_graphsToInitialize.clear();
    }

    void OpenRaveRviz::HandleMenus()
    {
        std::list<OpenRAVE::EnvironmentBasePtr> envs;
        RaveGetEnvironments(envs);

        std::vector<QAction*> actionRemovals;
        for (int j = 0; j < m_environmentsMenu->actions().count(); j++)
        {
            QAction* action = m_environmentsMenu->actions().at(j);

            if (!RaveGetEnvironment(action->text().toInt()))
            {
                actionRemovals.push_back(action);
            }
            else
            {
                if (RaveGetEnvironmentId(GetCurrentViewEnv()) != action->text().toInt())
                {
                    action->setChecked(false);
                }
                else
                {
                    action->setChecked(true);
                }
            }
        }

        for (size_t j = 0; j < actionRemovals.size(); j++)
        {
            m_environmentsMenu->removeAction(actionRemovals.at(j));
        }

        for (std::list<OpenRAVE::EnvironmentBasePtr>::iterator it = envs.begin(); it != envs.end(); it++)
        {
            OpenRAVE::EnvironmentBasePtr& env = *it;
            std::string name = GetEnvironmentHash(env);

            bool hasAction = false;

            for (int j = 0; j < m_environmentsMenu->actions().count(); j++)
            {
                QAction* action = m_environmentsMenu->actions().at(j);

                if (action->text().toStdString() == name)
                {
                    hasAction = true;
                    break;
                }
            }

            if (!hasAction)
            {
                QAction* action = m_environmentsMenu->addAction(QString::fromStdString(name));
                action->setCheckable(true);
                connect(action, SIGNAL(triggered(bool)), this, SLOT(setEnvironment(bool)));
            }
        }
    }

    // forces synchronization with the environment, returns when the environment is fully synchronized.
    void OpenRaveRviz::EnvironmentSync()
    {
        if(!initialized_)
        {
            return;
        }

        setWindowTitle("Openrave Rviz Viewer[*]");
        {
            OpenRAVE::EnvironmentMutex::scoped_lock environment_lock(GetCurrentViewEnv()->GetMutex());
            UpdateDisplay();
            HandleMenus();
        }

        for(std::map<size_t, ViewerThreadCallbackFn>::iterator it = m_syncCallbacks.begin(); it != m_syncCallbacks.end(); it++)
        {
            it->second();
        }

        //ros::spinOnce();

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

        OpenRAVE::Transform oldTransform = GetCameraTransform();
        float oldFocalLength = GetCameraIntrinsics().focal_length;
        SetCamera(t, intrinsics.focal_length);


        uint8_t* data = (uint8_t*)OffscreenRender(width, height, 24);
        memory = std::vector<uint8_t>(data, data + sizeof(data) / sizeof(data[0]) );

        SetCamera(oldTransform, oldFocalLength);

        return true;
    }


    // Overloading OPENRAVE drawing functions....
    // Note: line width and point size are unsupported in Ogre
    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector< float > &color, int drawstyle)
    {
        if(numPoints <= 0)
        {
            return  OpenRAVE::GraphHandlePtr();
        }

        if(fPointSize > 1)
        {
            RAVELOG_WARN("or_rviz does not yet support point size.\n");
        }

        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();
        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);


        std::vector<float> mypoints(numPoints * 3);
        for(int i = 0; i < numPoints; ++i)
        {
            mypoints[3 * i + 0] = ppoints[0];
            mypoints[3 * i + 1] = ppoints[1];
            mypoints[3 * i + 2] = ppoints[2];
            ppoints = (float*)((char*)ppoints + stride);
        }

        for(int i = 0; i < numPoints; ++i)
        {
            manualObject->position(mypoints[3 * i + 0], mypoints[3 * i + 1], mypoints[3 * i + 2]);
            manualObject->colour(color.x, color.y, color.z, color.w);
        }



        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    // Note: line width and point size are unsupported in Ogre
    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const float *colors, int drawstyle, bool bhasalpha)
    {
        if(numPoints <= 0)
        {
            return  OpenRAVE::GraphHandlePtr();
        }

        if(fPointSize > 1)
        {
            RAVELOG_WARN("or_rviz does not yet support point size.\n");
        }

        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();
        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);


        std::vector<float> mypoints(numPoints * 3);


        int colorSize = bhasalpha ? 4 : 3;

        std::vector<float> myColors(numPoints * colorSize);
        for(int i = 0; i < numPoints; ++i)
        {
            mypoints[3 * i + 0] = ppoints[0];
            mypoints[3 * i + 1] = ppoints[1];
            mypoints[3 * i + 2] = ppoints[2];
            myColors[colorSize * i + 0] = colors[0];
            myColors[colorSize * i + 1] = colors[1];
            myColors[colorSize * i + 2] = colors[2];

            if(bhasalpha)
            {
                myColors[colorSize * i + 3] = colors[3];
            }

            ppoints = (float*)((char*)ppoints + stride);
            colors = (float*)((char*)colors + colorSize * sizeof(float));
        }

        for(int i = 0; i < numPoints; ++i)
        {
            manualObject->position(mypoints[3 * i + 0], mypoints[3 * i + 1], mypoints[3 * i + 2]);
            if(bhasalpha)
            {
                manualObject->colour(myColors[4 * i + 0], myColors[4 * i + 1], myColors[4 * i + 2], myColors[4 * i + 3]);
            }
            else
            {
                manualObject->colour(myColors[3 * i + 0], myColors[3 * i + 1], myColors[3 * i + 2]);
            }
        }

        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    // Note: line width and point size are unsupported in Ogre
    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();

        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);


        if(fwidth > 1)
        {
            RAVELOG_WARN("or_rviz does not yet support line width.\n");
        }

        std::vector<float> mypoints((numPoints - 1) * 6);
        float* next;
        for (int i = 0; i < numPoints - 1; ++i)
        {
            next = (float*) ((char*) ppoints + stride);

            mypoints[6 * i + 0] = ppoints[0];
            mypoints[6 * i + 1] = ppoints[1];
            mypoints[6 * i + 2] = ppoints[2];
            mypoints[6 * i + 3] = next[0];
            mypoints[6 * i + 4] = next[1];
            mypoints[6 * i + 5] = next[2];

            ppoints = next;
        }


        for(int i = 0; i < numPoints - 1; i++)
        {
            manualObject->position(mypoints[6 * i + 0], mypoints[6 * i + 1], mypoints[6 * i + 2]);
            manualObject->colour(color.x, color.y, color.z, color.w);
            manualObject->position(mypoints[6 * i + 3], mypoints[6 * i + 4], mypoints[6 * i + 5]);
            manualObject->colour(color.x, color.y, color.z, color.w);
        }



        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();

        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

        if(fwidth > 1)
        {
            RAVELOG_WARN("or_rviz does not yet support line width.\n");
        }

        std::vector<float> mypoints((numPoints - 1) * 6), mycolors((numPoints - 1) * 6);
        float* next;
        for (int i = 0; i < numPoints - 1; ++i)
        {
            next = (float*) ((char*) ppoints + stride);

            mypoints[6 * i + 0] = ppoints[0];
            mypoints[6 * i + 1] = ppoints[1];
            mypoints[6 * i + 2] = ppoints[2];
            mypoints[6 * i + 3] = next[0];
            mypoints[6 * i + 4] = next[1];
            mypoints[6 * i + 5] = next[2];

            mycolors[6 * i + 0] = colors[3 * i + 0];
            mycolors[6 * i + 1] = colors[3 * i + 1];
            mycolors[6 * i + 2] = colors[3 * i + 2];
            mycolors[6 * i + 3] = colors[3 * i + 3];
            mycolors[6 * i + 4] = colors[3 * i + 4];
            mycolors[6 * i + 5] = colors[3 * i + 5];

            ppoints = next;
        }

        for(int i = 0; i < numPoints - 1; i++)
        {
            manualObject->position(mypoints[6 * i + 0], mypoints[6 * i + 1], mypoints[6 * i + 2]);
            manualObject->colour(mycolors[6 * i + 0], mycolors[6 * i + 1], mycolors[6 * i + 2]);
            manualObject->position(mypoints[6 * i + 3], mypoints[6 * i + 4], mypoints[6 * i + 5]);
            manualObject->colour(mycolors[6 * i + 3], mycolors[6 * i + 4], mycolors[6 * i + 5]);
        }




        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinelist (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();

        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

        if(fwidth > 1)
        {
            RAVELOG_WARN("or_rviz does not yet support line width.\n");
        }


        std::vector<float> mypoints(numPoints * 3);
        for(int i = 0; i < numPoints; ++i)
        {
            mypoints[3*i+0] = ppoints[0];
            mypoints[3*i+1] = ppoints[1];
            mypoints[3*i+2] = ppoints[2];
            ppoints = (float*)((char*)ppoints + stride);
        }


        for(int i = 0; i < numPoints; i++)
        {
            manualObject->position(mypoints[3 * i + 0], mypoints[3 * i + 1], mypoints[3 * i + 2]);
            manualObject->colour(color.x, color.y, color.z, color.w);
        }


        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinelist (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();

        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);


        if(fwidth > 1)
        {
            RAVELOG_WARN("or_rviz does not yet support line width.\n");
        }


        boost::multi_array<float,2> vcolors;
        vcolors.resize(boost::extents[numPoints][3]);

        for(int i = 0; i < numPoints; ++i)
        {
            vcolors[i][0] = colors[3 * i + 0];
            vcolors[i][1] = colors[3 * i + 1];
            vcolors[i][2] = colors[3 * i + 2];
        }

        std::vector<float> mypoints(numPoints*3);
        for(int i = 0; i < numPoints; ++i)
        {
            mypoints[3*i+0] = ppoints[0];
            mypoints[3*i+1] = ppoints[1];
            mypoints[3*i+2] = ppoints[2];
            ppoints = (float*)((char*)ppoints + stride);
        }

        for(int i = 0; i < numPoints; i++)
        {
            manualObject->position(mypoints[3 * i + 0], mypoints[3 * i + 1], mypoints[3 * i + 2]);
            manualObject->colour(vcolors[i][0], vcolors[i][1], vcolors[i][2], 1);
        }




        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawarrow (const OpenRAVE::RaveVector< float > &p1, const OpenRAVE::RaveVector< float > &p2, float fwidth, const OpenRAVE::RaveVector< float > &color)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        float len = (p2 - p1).lengthsqr3();
        //  float shaft_length = 1.0f, float shaft_radius = 0.1f, float head_length = 0.3f, float head_radius =  0.2f
        rviz::Arrow* arrow = new rviz::Arrow(getManager()->getSceneManager(), sceneNode, len, fwidth,  0.3 * len, fwidth * 1.25);
        arrow->setColor(Ogre::ColourValue(color.x, color.y, color.z, color.w));
        arrow->setDirection(converters::ToOgreVector(p2 - p1).normalisedCopy());
        arrow->setPosition(converters::ToOgreVector(p1));
        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawbox (const OpenRAVE::RaveVector< float > &vpos, const OpenRAVE::RaveVector< float > &vextents)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();
        Ogre::ManualObject* cube = render_panel_->getManager()->getSceneManager()->createManualObject();
        cube->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);


        // This is probably the worst way to draw a cube in the world, sorry.

        cube->position(vextents.x + vpos.x, -vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(0.408248, -0.816497, 0.408248);
        cube->textureCoord(1, 0);
        cube->position(-vextents.x + vpos.x, -vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(-0.408248, -0.816497, -0.408248);
        cube->textureCoord(0, 1);
        cube->position(vextents.x + vpos.x, -vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(0.666667, -0.333333, -0.666667);
        cube->textureCoord(1, 1);
        cube->position(-vextents.x + vpos.x, -vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(-0.666667, -0.333333, 0.666667);
        cube->textureCoord(0, 0);
        cube->position(vextents.x + vpos.x, vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(0.666667, 0.333333, 0.666667);
        cube->textureCoord(1, 0);
        cube->position(-vextents.x + vpos.x, -vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(-0.666667, -0.333333, 0.666667);
        cube->textureCoord(0, 1);
        cube->position(vextents.x + vpos.x, -vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(0.408248, -0.816497, 0.408248);
        cube->textureCoord(1, 1);
        cube->position(-vextents.x + vpos.x, vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(-0.408248, 0.816497, 0.408248);
        cube->textureCoord(0, 0);
        cube->position(-vextents.x + vpos.x, vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(-0.666667, 0.333333, -0.666667);
        cube->textureCoord(0, 1);
        cube->position(-vextents.x + vpos.x, -vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(-0.408248, -0.816497, -0.408248);
        cube->textureCoord(1, 1);
        cube->position(-vextents.x + vpos.x, -vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(-0.666667, -0.333333, 0.666667);
        cube->textureCoord(1, 0);
        cube->position(vextents.x + vpos.x, -vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(0.666667, -0.333333, -0.666667);
        cube->textureCoord(0, 1);
        cube->position(vextents.x + vpos.x, vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(0.408248, 0.816497, -0.408248);
        cube->textureCoord(1, 1);
        cube->position(vextents.x + vpos.x, -vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(0.408248, -0.816497, 0.408248);
        cube->textureCoord(0, 0);
        cube->position(vextents.x + vpos.x, -vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(0.666667, -0.333333, -0.666667);
        cube->textureCoord(1, 0);
        cube->position(-vextents.x + vpos.x, -vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(-0.408248, -0.816497, -0.408248);
        cube->textureCoord(0, 0);
        cube->position(-vextents.x + vpos.x, vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(-0.408248, 0.816497, 0.408248);
        cube->textureCoord(1, 0);
        cube->position(vextents.x + vpos.x, vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(0.408248, 0.816497, -0.408248);
        cube->textureCoord(0, 1);
        cube->position(-vextents.x + vpos.x, vextents.y + vpos.y,  -vextents.z + vpos.z);
        cube->normal(-0.666667, 0.333333, -0.666667);
        cube->textureCoord(1, 1);
        cube->position(vextents.x + vpos.x, vextents.y + vpos.y,  vextents.z + vpos.z);
        cube->normal(0.666667, 0.333333, 0.666667);
        cube->textureCoord(0, 0);

        cube->triangle(0, 1, 2);
        cube->triangle(3, 1, 0);
        cube->triangle(4, 5, 6);
        cube->triangle(4, 7, 5);
        cube->triangle(8, 9, 10);
        cube->triangle(10, 7, 8);
        cube->triangle(4, 11, 12);
        cube->triangle(4, 13, 11);
        cube->triangle(14, 8, 12);
        cube->triangle(14, 15, 8);
        cube->triangle(16, 17, 18);
        cube->triangle(16, 19, 17);

        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, cube));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawplane (const OpenRAVE::RaveTransform< float > &tplane, const OpenRAVE::RaveVector< float > &vextents, const boost::multi_array< float, 3 > &vtexture)
    {
        RAVELOG_WARN("or_rviz does not yet support planes.\n");
        // This is not yet implemented
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const OpenRAVE::RaveVector< float > &color)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();
        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();
        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);


        if(color.w < 1)
        {
            RAVELOG_WARN("or_rviz does not yet support alpha-blended trimeshes.\n");
        }

        if (pIndices != NULL)
        {
            for (int i = 0; i < 3 * numTriangles; ++i)
            {
                float* p = (float*) ((char*) ppoints + stride * pIndices[i]);
                manualObject->position(p[0], p[1], p[2]);
                manualObject->colour(color.x, color.y, color.z, color.w);
                manualObject->index(i);
            }
        }
        else
        {

            for (int i = 0; i < 3 * numTriangles; ++i)
            {
                manualObject->position(ppoints[0], ppoints[1], ppoints[2]);
                ppoints = (float*) ((char*) ppoints + stride);
                manualObject->colour(color.x, color.y, color.z, color.w);
                manualObject->index(i);
            }


        }


         OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
         m_graphsToInitialize.push_back(ptr);
         return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array< float, 2 > &colors)
    {

        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();
        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();
        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);


        if (pIndices != NULL)
        {
            for (int i = 0; i < 3 * numTriangles; ++i)
            {
                float* p = (float*) ((char*) ppoints + stride * pIndices[i]);
                manualObject->position(p[0], p[1], p[2]);
                manualObject->colour((float)colors[i][0], (float)(colors[i][1]), (float)colors[i][2], 1.0f);
                manualObject->index(i);
            }
        }
        else
        {

            for (int i = 0; i < 3 * numTriangles; ++i)
            {
                manualObject->position(ppoints[0], ppoints[1], ppoints[2]);
                ppoints = (float*) ((char*) ppoints + stride);
                manualObject->colour((float)colors[i][0], (float)(colors[i][1]), (float)colors[i][2], 1.0f);
                manualObject->index(i);
            }


        }


         OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
         m_graphsToInitialize.push_back(ptr);
         return ptr;
    }

    void OpenRaveRviz::Clear()
    {
        m_envDisplay->Clear();
    }

    void OpenRaveRviz::syncUpdate()
    {
        EnvironmentSync();
    }



    RvizGraphHandle::RvizGraphHandle()
    {
        m_object = NULL;
        m_node = NULL;
    }

    RvizGraphHandle::RvizGraphHandle(Ogre::SceneNode* node)
    {
        m_node = node;
        m_object = NULL;
    }

    RvizGraphHandle::RvizGraphHandle(Ogre::SceneNode* node, Ogre::ManualObject* object)
    {
        m_node = node;
        m_object = object;
    }

    RvizGraphHandle::~RvizGraphHandle()
    {
        if(m_node)
        {
            delete m_node;
        }
    }

    void  RvizGraphHandle::SetShow(bool show)
    {
        m_node->setVisible(show, true);
    }

    void  RvizGraphHandle::SetTransform(const OpenRAVE::RaveTransform<float>& transform)
    {
        m_node->setPosition(transform.trans.x, transform.trans.y, transform.trans.z);
        m_node->setOrientation(converters::ToOgreQuaternion(transform.rot));
    }

    void  RvizGraphHandle::Initialize()
    {
        Ogre::ManualObject* manual = dynamic_cast<Ogre::ManualObject*>(m_object);

        if(manual)
        {
            manual->end();
            m_node->attachObject(m_object);
        }

    }


}

static char* argv[1] = {const_cast<char *>("or_rviz")};
static int argc = 1;


OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if (type == OpenRAVE::PT_Viewer && interfacename == "or_rviz")
    {

        // Hack to prevent screen printing from RVIZ!
        Ogre::LogManager* logger = new Ogre::LogManager();
        logger->createLog("ogre_log.log", true, false, false);


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


