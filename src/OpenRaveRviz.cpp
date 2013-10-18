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
                environmentBase->GetMutex().lock();
                SetCurrentViewEnv(environmentBase);
                environmentBase->GetMutex().unlock();


            }
        }

    }

    void OpenRaveRviz::UpdateDisplay()
    {
        if (!m_envDisplay)
        {
            rviz::DisplayWrapper* wrapper = m_rvizManager->getDisplayWrapper("OpenRAVE");

            if (!wrapper)
            {
                wrapper = m_rvizManager->createDisplay("or_rviz/Environment", "OpenRAVE", true);
            }

            m_envDisplay = dynamic_cast<EnvironmentDisplay*>(wrapper->getDisplay());
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
        setWindowTitle("Openrave Rviz Viewer[*]");
        GetCurrentViewEnv()->GetMutex().lock();

        UpdateDisplay();
        HandleMenus();

        GetCurrentViewEnv()->GetMutex().unlock();

        for(std::map<size_t, ViewerThreadCallbackFn>::iterator it = m_syncCallbacks.begin(); it != m_syncCallbacks.end(); it++)
        {
            it->second();
        }

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
    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector< float > &color, int drawstyle)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();

        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);

        for(int i = 0; i < numPoints; i++)
        {
            manualObject->colour(color.x, color.y, color.z, color.w);
            manualObject->position(ppoints[i * stride + 0], ppoints[i * stride + 1], ppoints[i * stride + 2]);
        }



        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const float *colors, int drawstyle, bool bhasalpha)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();

        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);

        for(int i = 0; i < numPoints; i++)
        {
            manualObject->colour(colors[i * 4 + 0], colors[i * 4 + 1], colors[i * 4 + 2], colors[i * 4 + 3]);
            manualObject->position(ppoints[i * stride + 0], ppoints[i * stride + 1], ppoints[i * stride + 2]);
        }


        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color)
    {
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();

        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();

        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

        for(int i = 0; i < numPoints - 1; i++)
        {
            manualObject->colour(color.x, color.y, color.z, color.w);
            manualObject->position(ppoints[i * stride + 0], ppoints[i * stride + 1], ppoints[i * stride + 2]);
            manualObject->colour(color.x, color.y, color.z, color.w);
            manualObject->position(ppoints[(i + 1) * stride + 0], ppoints[(i + 1) * stride + 1], ppoints[(i + 1) * stride + 2]);
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

        for(int i = 0; i < numPoints - 1; i++)
        {
            manualObject->colour(colors[i * 4 + 0], colors[i * 4 + 1], colors[i * 4 + 2], colors[i * 4 + 3]);
            manualObject->position(ppoints[i * stride + 0], ppoints[i * stride + 1], ppoints[i * stride + 2]);
            manualObject->colour(colors[(i + 1) * 4 + 0], colors[(i + 1) * 4 + 1], colors[(i + 1) * 4 + 2], colors[(i + 1) * 4 + 3]);
            manualObject->position(ppoints[(i + 1) * stride + 0], ppoints[(i + 1) * stride + 1], ppoints[(i + 1) * stride + 2]);
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

        for(int i = 0; i < numPoints; i++)
        {
            manualObject->colour(color.x, color.y, color.z, color.w);
            manualObject->position(ppoints[i * stride + 0], ppoints[i * stride + 1], ppoints[i * stride + 2]);
            manualObject->colour(color.x, color.y, color.z, color.w);
            manualObject->position(ppoints[(i + 1) * stride + 0], ppoints[(i + 1) * stride + 1], ppoints[(i + 1) * stride + 2]);
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

        for(int i = 0; i < numPoints; i++)
        {
            manualObject->colour(colors[i * 4 + 0], colors[i * 4 + 1], colors[i * 4 + 2], colors[i * 4 + 3]);
            manualObject->position(ppoints[i * stride + 0], ppoints[i * stride + 1], ppoints[i * stride + 2]);
            manualObject->colour(colors[(i + 1) * 4 + 0], colors[(i + 1) * 4 + 1], colors[(i + 1) * 4 + 2], colors[(i + 1) * 4 + 3]);
            manualObject->position(ppoints[(i + 1) * stride + 0], ppoints[(i + 1) * stride + 1], ppoints[(i + 1) * stride + 2]);
        }



        OpenRAVE::GraphHandlePtr ptr(new RvizGraphHandle(sceneNode, manualObject));
        m_graphsToInitialize.push_back(ptr);
        return ptr;
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
        Ogre::SceneNode* sceneNode = m_envDisplay->GetNode()->createChildSceneNode();
        Ogre::ManualObject* manualObject = render_panel_->getManager()->getSceneManager()->createManualObject();
        manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);


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
        }

        m_node->attachObject(m_object);
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

