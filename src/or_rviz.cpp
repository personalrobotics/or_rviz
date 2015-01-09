#include "or_rviz.h"

//#include "RenderWindow.h"
#include <qapplication.h>
#include <openrave/config.h>
#include <QDockWidget>
#include <OgreCamera.h>
#include "Converters.h"
#include <rviz/display.h>
//#include <rviz/display_wrapper.h>
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
#include <boost/thread/locks.hpp>
#include <boost/signals2.hpp>
#include <boost/algorithm/string.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include "or_conversions.h"
static double const kRefreshRate = 30;
static double const kWidthScaleFactor = 100;
using namespace OpenRAVE;
using namespace rviz;
using namespace or_interactivemarker;

namespace detail
{

    class ScopedConnection: public OpenRAVE::UserData
    {
        public:
            ScopedConnection(boost::signals2::connection const &connection) :
                    scoped_connection_(connection)
            {
            }

            virtual ~ScopedConnection()
            {
            }

        private:
            boost::signals2::scoped_connection scoped_connection_;
    };

    static std::string GetRemainingContent(std::istream &stream, bool trim = false)
    {

        std::istreambuf_iterator<char> eos;
        std::string str(std::istreambuf_iterator<char>(stream), eos);
        if (trim)
        {
            boost::algorithm::trim(str);
        }
        return str;
    }

    class InteractiveMarkerGraphHandle: public OpenRAVE::GraphHandle
    {
        public:
            InteractiveMarkerGraphHandle(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> const &interactive_marker_server, visualization_msgs::InteractiveMarkerPtr const &interactive_marker) :
                    server_(interactive_marker_server), interactive_marker_(interactive_marker), show_(true)
            {
                BOOST_ASSERT(interactive_marker_server);
                BOOST_ASSERT(interactive_marker);

                server_->insert(*interactive_marker_);
            }

            virtual ~InteractiveMarkerGraphHandle()
            {
                server_->erase(interactive_marker_->name);
            }

            virtual void SetTransform(OpenRAVE::RaveTransform<float> const &t)
            {
                // TODO: This could SEGFAULT if it is called too quickly after the
                // marker is created.
                if (show_)
                {
                    server_->setPose(interactive_marker_->name, or_interactivemarker::toROSPose<>(t));
                }
            }

            virtual void SetShow(bool show)
            {
                if (show && !show_)
                {
                    server_->insert(*interactive_marker_);
                }
                else if (!show && show_)
                {
                    server_->erase(interactive_marker_->name);
                }
                show_ = show;
            }

        private:
            boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
            visualization_msgs::InteractiveMarkerPtr interactive_marker_;
            bool show_;
    };

}

namespace or_rviz
{

    OpenRaveRviz::OpenRaveRviz(OpenRAVE::EnvironmentBasePtr env, QWidget * parent, Qt::WindowFlags flags) :
            VisualizationFrame(parent),
            OpenRAVE::ViewerBase(env),
            m_rvizManager(NULL),
            m_mainRenderPanel(NULL),
            m_envDisplay(NULL),
            m_autoSync(false),
            m_name("or_rviz"),
            server_(boost::make_shared<interactive_markers::InteractiveMarkerServer>("openrave")),
            running_(false),
            do_sync_(true)
    {
        BOOST_ASSERT(env);

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
        //m_offscreenRenderer->setHidden(true);

        //RegisterCommand("register", boost::bind(&OpenRaveRviz::RegisterMenuCallback, this, _1, _2), "register [objectName] [menuItemName] [pointer] Registers a python object with the given pointer (as a string) to the menu of a kinbody.");
        //RegisterCommand("unregister", boost::bind(&OpenRaveRviz::UnRegisterMenuCallback, this, _1, _2), "register [objectName] [menuItemName] Unregisters the menu command given.");
        RegisterCommand("AddMenuEntry", boost::bind(&OpenRaveRviz::AddMenuEntryCommand, this, _1, _2), "Attach a custom menu entry to an object.");
        RegisterCommand("GetMenuSelection", boost::bind(&OpenRaveRviz::GetMenuSelectionCommand, this, _1, _2), "Get the name of the last menu selection.");
        m_offscreenCamera = m_rvizManager->getSceneManager()->createCamera("OfscreenCamera");

        rviz::InteractiveMarkerDisplay* markerDisplay =  dynamic_cast< rviz::InteractiveMarkerDisplay*>(m_rvizManager->createDisplay("rviz/InteractiveMarkers", "OpenRAVE Markers", true));
        markerDisplay->setTopic("/openrave/update", "");
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
        running_ = true;
        return qApp->exec();
    }

    void OpenRaveRviz::quitmainloop()
    {
        running_ = false;
        qApp->quit();
    }

    bool OpenRaveRviz::AddMenuEntryCommand(std::ostream &out, std::istream &in)
    {
        std::string type, kinbody_name;
        in >> type >> kinbody_name;

        // Get the KinBodyMarker associated with the target object.
        OpenRAVE::KinBodyPtr const kinbody = GetCurrentViewEnv()->GetKinBody(kinbody_name);
        if (!kinbody)
        {
            throw OpenRAVE::openrave_exception(boost::str(boost::format("There is no KinBody named '%s' in the environment.") % kinbody_name), OpenRAVE::ORE_Failed);
        }

        OpenRAVE::UserDataPtr const marker_raw = kinbody->GetUserData("interactive_marker");
        auto const marker = boost::dynamic_pointer_cast<KinBodyMarker>(marker_raw);
        if (!marker)
        {
            throw OpenRAVE::openrave_exception(boost::str(boost::format("KinBody '%s' does not have an associated marker.") % kinbody_name), OpenRAVE::ORE_InvalidState);
        }

        if (type == "kinbody")
        {
            std::string const name = detail::GetRemainingContent(in, true);
            auto const callback = boost::bind(&OpenRaveRviz::KinBodyMenuCallback, this, kinbody, name);
            marker->AddMenuEntry(name, callback);
        }
        else if (type == "link")
        {
            std::string link_name;
            in >> link_name;

            OpenRAVE::KinBody::LinkPtr const link = kinbody->GetLink(link_name);
            if (!link)
            {
                throw OpenRAVE::openrave_exception(boost::str(boost::format("KinBody '%s' has no link '%s'.") % kinbody_name % link_name), OpenRAVE::ORE_Failed);
            }

            std::string const name = detail::GetRemainingContent(in, true);
            auto const callback = boost::bind(&OpenRaveRviz::LinkMenuCallback, this, link, name);
            marker->AddMenuEntry(link, name, callback);
        }
        else if (type == "manipulator" || type == "ghost_manipulator")
        {
            std::string manipulator_name;
            in >> manipulator_name;

            if (!kinbody->IsRobot())
            {
                throw OpenRAVE::openrave_exception(boost::str(boost::format("KinBody '%s' is not a robot and does not support"
                        " manipulator menus.") % kinbody_name), OpenRAVE::ORE_Failed);
            }

            auto const robot = boost::dynamic_pointer_cast<OpenRAVE::RobotBase>(kinbody);
            OpenRAVE::RobotBase::ManipulatorPtr const manipulator = robot->GetManipulator(manipulator_name);
            if (!manipulator)
            {
                throw OpenRAVE::openrave_exception(boost::str(boost::format("Robot '%s' has no manipulator '%s'.") % kinbody_name % manipulator_name), OpenRAVE::ORE_Failed);
            }

            std::string const name = detail::GetRemainingContent(in, true);
            auto const callback = boost::bind(&OpenRaveRviz::ManipulatorMenuCallback, this, manipulator, name);
            marker->AddMenuEntry(manipulator, name, callback);
        }
        return true;
    }

    bool OpenRaveRviz::GetMenuSelectionCommand(std::ostream &out, std::istream &in)
    {
        out << menu_queue_.rdbuf();
        return true;
    }


    uchar* OpenRaveRviz::OffscreenRender(int desiredWidth, int desiredHeight, int desiredDepth)
    {
        //RAVELOG_DEBUG("Writing to screen %d %d %d\n", desiredWidth, desiredHeight, desiredDepth);

        Ogre::PixelFormat desiredFormat;

        switch(desiredDepth)
        {
            case 8:
                desiredFormat = Ogre::PF_L8;
                break;
            case 16:
                desiredFormat = Ogre::PF_FLOAT16_GR;
                break;
            case 24:
                desiredFormat = Ogre::PF_R8G8B8;
                break;
            case 32:
                desiredFormat = Ogre::PF_R8G8B8A8;
                break;
            default:
                RAVELOG_ERROR("Error: unsupported depth %d. Supported depths: 8 (gray byte), 16 (float16 gray), 24 (RGB bytes), 32 (RGBA bytes)\n");
                return NULL;
                break;

        }

        try
        {
            return WaitForRenderTarget(desiredWidth, desiredHeight, desiredDepth,  desiredFormat, "RttTex");
        }
        catch(std::bad_alloc& error)
        {
            RAVELOG_ERROR(error.what());
            return NULL;
        }

        return NULL;
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
        delete m_envDisplay;
        m_envDisplay = NULL;
    }


    void OpenRaveRviz::SetBkgndColor(const OpenRAVE::RaveVector<float> &color)
    {
        m_rvizManager->getRenderPanel()->setBackgroundColor(Ogre::ColourValue(color.x, color.y, color.z));
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
        do_sync_ = update;
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

        if (pbody)
        {
            pbody->RemoveUserData("interactive_marker");
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
            m_envDisplay = dynamic_cast<EnvironmentDisplay*>(m_rvizManager->createDisplay("or_rviz/Environment", "OpenRAVE", true));
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

    // This has to exist because Ogre hates creating textures in threads. This waits in one thread for
    // the texture to be created in the main thread.
    unsigned char* OpenRaveRviz::WaitForRenderTarget(int w, int h, int depth, Ogre::PixelFormat format, std::string name)
    {
        RenderTargetRequest* request = new RenderTargetRequest();
        request->format = format;
        request->name = name;
        request->width = w;
        request->height = h;
        request->data =  new unsigned char [w * h * depth];
        Ogre::Box extents(0, 0,  w, h);
        request->pixelBox = new Ogre::PixelBox(extents, request->format, request->data);
        request->valid = false;



        RegisterRenderTargetRequest(request);

        int maxIters = 100000;

        for(int i = 0; i < maxIters; i++)
        {
            m_targetMutex.lock();

            if(request->valid)
            {
                //RAVELOG_DEBUG("Got texture!\n");
                m_targetMutex.unlock();
                DeleteRequest(request);
                unsigned char* toReturn = request->data;
                delete request;
                return toReturn;
            }

            m_targetMutex.unlock();
            usleep(10000);
        }

        return NULL;
    }

    void OpenRaveRviz::RegisterRenderTargetRequest(RenderTargetRequest* request)
    {
        boost::mutex::scoped_lock(m_targetMutex);
        m_renderTargetRequests.push_back(request);
    }

    void OpenRaveRviz::HandleRenderTargetRequests()
    {
        boost::mutex::scoped_lock(m_targetMutex);

        for(size_t i = 0; i < m_renderTargetRequests.size(); i++)
        {
            RenderTargetRequest* req = m_renderTargetRequests[i];

            if(req->valid)
            {
                continue;
            }

            Ogre::TextureManager::getSingleton().unload(req->name);
            Ogre::TextureManager::getSingleton().remove(req->name);

           req->texture = Ogre::TextureManager::getSingleton().createManual(req->name,
                              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                              Ogre::TEX_TYPE_2D, req->width,
                              req->height, 0, req->format, Ogre::TU_RENDERTARGET);

           if(!req->texture.get())
           {
               RAVELOG_ERROR("Texture is null!\n");
           }


           Ogre::RenderTexture* renderTexture = req->texture->getBuffer()->getRenderTarget();
           renderTexture->addViewport(m_offscreenCamera);
           renderTexture->copyContentsToMemory(*req->pixelBox, Ogre::RenderTarget::FB_AUTO);

           req->valid = true;

        }
    }

    void OpenRaveRviz::DeleteRequest(RenderTargetRequest* request)
    {
        boost::mutex::scoped_lock(m_targetMutex);

        for(std::vector<RenderTargetRequest*>::iterator it = m_renderTargetRequests.begin();
            it != m_renderTargetRequests.end(); it++)
        {
            RenderTargetRequest* req =  *it;

            if(req == request)
            {
                delete req->pixelBox;


                m_renderTargetRequests.erase(it);
                break;
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

        HandleRenderTargetRequests();
        setWindowTitle("Openrave Rviz Viewer[*]");
        if (GetCurrentViewEnv()->GetMutex().try_lock())
        {
            //UpdateDisplay();
            HandleMenus();
            GetCurrentViewEnv()->GetMutex().unlock();

            std::vector<KinBodyPtr> bodies;
            GetCurrentViewEnv()->GetBodies(bodies);

            for (KinBodyPtr body : bodies)
            {
                OpenRAVE::UserDataPtr const raw = body->GetUserData("interactive_marker");
                auto body_marker = boost::dynamic_pointer_cast<KinBodyMarker>(raw);
                BOOST_ASSERT(!raw || body_marker);

                // Create the new geometry if neccessary.
                if (!raw)
                {
                    RAVELOG_INFO("Creating KinBodyMarker for '%s'.\n", body->GetName().c_str());
                    body_marker = boost::make_shared<KinBodyMarker>(server_, body);
                    body->SetUserData("interactive_marker", body_marker);
                }

                body_marker->EnvironmentSync();
            }
        }
        for(std::map<size_t, ViewerThreadCallbackFn>::iterator it = m_syncCallbacks.begin(); it != m_syncCallbacks.end(); it++)
        {
            it->second();
        }

        server_->applyChanges();
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
        SetCamera(getManager()->getRenderPanel()->getCamera(), trans, focalDistance);
    }

    void OpenRaveRviz::SetCamera (Ogre::Camera* camera, const OpenRAVE::RaveTransform<float> &trans, float focalDistance)
    {
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
        SetCamera(m_offscreenCamera, t, intrinsics.focal_length);
        m_offscreenCamera->setNearClipDistance(intrinsics.focal_length);
        m_offscreenCamera->setFarClipDistance(intrinsics.focal_length*10000);
        m_offscreenCamera->setAspectRatio((intrinsics.fy/(float)height) / (intrinsics.fx/(float)width));
        m_offscreenCamera->setFOVy(Ogre::Radian(2.0f*atan(0.5f*height/intrinsics.fy)));



        uint8_t* data = (uint8_t*)OffscreenRender(width, height, 24);

        if(!data)
        {
            RAVELOG_ERROR("Camera data was NULL!\n");
            return false;
        }


        // This is here just to not leak memory.
        std::vector<uint8_t> copyBuffer(data, data + sizeof(data) / sizeof(data[0]) );

        memory.clear();

        for(size_t i = 0; i < memory.size(); i++)
        {
            memory.push_back(copyBuffer[i]);
        }


        delete data;

        return true;
    }

    GraphHandlePtr OpenRaveRviz::plot3_marker(float const *points, int num_points, int stride, float point_size, OpenRAVE::RaveVector<float> const &color, int draw_style)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.color = toROSColor<>(color);

        if (draw_style == 0)
        {
            marker.type = visualization_msgs::Marker::POINTS;
            marker.scale.x = point_size;
            marker.scale.y = point_size;
        }
        else if (draw_style == 1)
        {
            marker.type = visualization_msgs::Marker::SPHERE_LIST;
            marker.scale.x = point_size;
            marker.scale.y = point_size;
            marker.scale.z = point_size;
        }
        else
        {
            throw OpenRAVE::openrave_exception(boost::str(boost::format("Unsupported drawstyle %d; expected 0 or 1.") % draw_style), OpenRAVE::ORE_InvalidArguments);
        }

        ConvertPoints(points, num_points, stride, &marker.points);

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3_marker(float const *points, int num_points, int stride, float point_size, float const *colors, int draw_style, bool has_alpha)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();

        if (draw_style == 0)
        {
            marker.type = visualization_msgs::Marker::POINTS;
            marker.scale.x = point_size;
            marker.scale.y = point_size;
        }
        else if (draw_style == 1)
        {
            // TODO: Does this support individual colors?
            marker.type = visualization_msgs::Marker::SPHERE_LIST;
            marker.scale.x = point_size;
            marker.scale.y = point_size;
            marker.scale.z = point_size;
        }
        else
        {
            throw OpenRAVE::openrave_exception(boost::str(boost::format("Unsupported drawstyle %d; expected 0 or 1.") % draw_style), OpenRAVE::ORE_InvalidArguments);
        }

        ConvertPoints(points, num_points, stride, &marker.points);
        ConvertColors(colors, num_points, has_alpha, &marker.colors);

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    GraphHandlePtr OpenRaveRviz::drawarrow_marker (const OpenRAVE::RaveVector< float > &p1, const OpenRAVE::RaveVector< float > &p2, float fwidth, const OpenRAVE::RaveVector< float > &color)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.type = visualization_msgs::Marker::ARROW;
        marker.color = toROSColor<>(color);
        marker.scale.x = fwidth * 1.0f;
        marker.scale.y = fwidth * 1.5f;
        marker.scale.z = fwidth * 2.0f;
        marker.points.push_back(toROSPoint(p1));
        marker.points.push_back(toROSPoint(p2));

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    GraphHandlePtr OpenRaveRviz::drawlinestrip_marker(float const *points, int num_points, int stride, float width, OpenRAVE::RaveVector<float> const &color)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.color = toROSColor<>(color);
        marker.scale.x = width;

        ConvertPoints(points, num_points, stride, &marker.points);

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    GraphHandlePtr OpenRaveRviz::drawlinestrip_marker(float const *points, int num_points, int stride, float width, float const *colors)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = width;

        ConvertPoints(points, num_points, stride, &marker.points);
        ConvertColors(colors, num_points, false, &marker.colors);

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    GraphHandlePtr OpenRaveRviz::drawlinelist_marker(float const *points, int num_points, int stride, float width, OpenRAVE::RaveVector<float> const &color)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.color = toROSColor<>(color);
        marker.scale.x = width / kWidthScaleFactor;

        ConvertPoints(points, num_points, stride, &marker.points);

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    GraphHandlePtr OpenRaveRviz::drawlinelist_marker(float const *points, int num_points, int stride, float width, float const *colors)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = width / kWidthScaleFactor;

        ConvertPoints(points, num_points, stride, &marker.points);
        ConvertColors(colors, num_points, false, &marker.colors);

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    GraphHandlePtr OpenRaveRviz::drawbox_marker(OpenRAVE::RaveVector<float> const &pos, OpenRAVE::RaveVector<float> const &extents)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position = toROSPoint<>(pos);
        marker.scale = toROSVector<>(2.0 * extents);

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawplane_marker(OpenRAVE::RaveTransform<float> const &transform, OpenRAVE::RaveVector<float> const &extents, boost::multi_array<float, 3> const &texture)
    {
        throw OpenRAVE::openrave_exception("drawplane is not implemented on InteractiveMarkerViewer", OpenRAVE::ORE_NotImplemented);
    }

    GraphHandlePtr OpenRaveRviz::drawtrimesh_marker(float const *points, int stride, int const *indices, int num_triangles, OpenRAVE::RaveVector<float> const &color)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.color = toROSColor<>(color);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        ConvertMesh(points, stride, indices, num_triangles, &marker.points);

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }

    GraphHandlePtr OpenRaveRviz::drawtrimesh_marker(float const *points, int stride, int const *indices, int num_triangles, boost::multi_array<float, 2> const &colors)
    {
        visualization_msgs::InteractiveMarkerPtr interactive_marker = CreateMarker();
        visualization_msgs::Marker &marker = interactive_marker->controls.front().markers.front();
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        ConvertMesh(points, stride, indices, num_triangles, &marker.points);

        // TODO: Colors should be per-vertex, not per-face.
        size_t const *color_shape = colors.shape();
        if (color_shape[0] != num_triangles)
        {
            throw OpenRAVE::openrave_exception(boost::str(boost::format("Number of colors does not equal number of triangles;"
                    " expected %d, got %d.") % color_shape[0] % num_triangles), OpenRAVE::ORE_InvalidArguments);
        }
        else if (color_shape[1] != 3 && color_shape[1] != 4)
        {
            throw OpenRAVE::openrave_exception(boost::str(boost::format("Invalid number of channels; expected 3 or 4, got %d.") % color_shape[1]), OpenRAVE::ORE_InvalidArguments);
        }

        marker.colors.resize(3 * num_triangles);
        for (int itri = 0; itri < num_triangles; ++itri)
        {
            std_msgs::ColorRGBA color;
            color.r = colors[itri][0];
            color.g = colors[itri][1];
            color.b = colors[itri][2];

            if (color_shape[1] == 4)
            {
                color.a = colors[itri][3];
            }
            else
            {
                color.a = 1.0;
            }

            for (int ivertex = 0; ivertex < 3; ++ivertex)
            {
                int const index_offset = 3 * itri + ivertex;
                int const index = stride * indices[index_offset];
                marker.colors[index] = color;
            }
        }

        return boost::make_shared<detail::InteractiveMarkerGraphHandle>(server_, interactive_marker);
    }


    // Overloading OPENRAVE drawing functions....
    // Note: line width and point size are unsupported in Ogre
    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3_native (const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector< float > &color, int drawstyle)
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
    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3_native (const float *ppoints, int numPoints, int stride, float fPointSize, const float *colors, int drawstyle, bool bhasalpha)
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
    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip_native (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color)
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

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip_native (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors)
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

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinelist_native (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color)
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

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinelist_native (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors)
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

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawarrow_native (const OpenRAVE::RaveVector< float > &p1, const OpenRAVE::RaveVector< float > &p2, float fwidth, const OpenRAVE::RaveVector< float > &color)
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

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawbox_native (const OpenRAVE::RaveVector< float > &vpos, const OpenRAVE::RaveVector< float > &vextents)
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

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawplane_native (const OpenRAVE::RaveTransform< float > &tplane, const OpenRAVE::RaveVector< float > &vextents, const boost::multi_array< float, 3 > &vtexture)
    {
        RAVELOG_WARN("or_rviz does not yet support planes.\n");
        // This is not yet implemented
        return OpenRAVE::GraphHandlePtr();
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawtrimesh_native (const float *ppoints, int stride, const int *pIndices, int numTriangles, const OpenRAVE::RaveVector< float > &color)
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

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawtrimesh_native (const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array< float, 2 > &colors)
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

    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3(const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector<float> &color, int drawstyle)
    {
        return plot3_marker(ppoints, numPoints, stride, fPointSize, color, drawstyle);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::plot3(const float *ppoints, int numPoints, int stride, float fPointSize, const float *colors, int drawstyle, bool bhasalpha)
    {
        return plot3_marker(ppoints, numPoints, stride, fPointSize, colors, drawstyle, bhasalpha);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip(const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector<float> &color)
    {
        return drawlinestrip_marker(ppoints, numPoints, stride, fwidth, color);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinestrip(const float *ppoints, int numPoints, int stride, float fwidth, const float *colors)
    {
        return drawlinestrip_marker(ppoints, numPoints, stride, fwidth, colors);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinelist(const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector<float> &color)
    {
        return drawlinelist_marker(ppoints, numPoints, stride, fwidth, color);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawlinelist(const float *ppoints, int numPoints, int stride, float fwidth, const float *colors)
    {
        return drawlinelist_marker(ppoints, numPoints, stride, fwidth, colors);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawarrow(const OpenRAVE::RaveVector<float> &p1, const OpenRAVE::RaveVector<float> &p2, float fwidth, const OpenRAVE::RaveVector<float> &color)
    {
        return drawarrow_marker(p1, p2, fwidth, color);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawbox(const OpenRAVE::RaveVector<float> &vpos, const OpenRAVE::RaveVector<float> &vextents)
    {
        return drawbox_marker(vpos, vextents);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawplane(const OpenRAVE::RaveTransform<float> &tplane, const OpenRAVE::RaveVector<float> &vextents, const boost::multi_array<float, 3> &vtexture)
    {
        return drawplane_marker(tplane, vextents, vtexture);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawtrimesh(const float *ppoints, int stride, const int *pIndices, int numTriangles, const OpenRAVE::RaveVector<float> &color)
    {
        return drawtrimesh_marker(ppoints, stride, pIndices, numTriangles, color);
    }

    OpenRAVE::GraphHandlePtr OpenRaveRviz::drawtrimesh(const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array<float, 2> &colors)
    {
        return drawtrimesh_marker(ppoints, stride, pIndices, numTriangles, colors);
    }

    void OpenRaveRviz::Clear()
    {
        if(m_envDisplay)
        {
            m_envDisplay->Clear();
        }

        std::vector<KinBodyPtr> bodies;
        GetCurrentViewEnv()->GetBodies(bodies);

        for (KinBodyPtr body : bodies)
        {
           body->RemoveUserData("interactive_marker");
        }
    }

    void OpenRaveRviz::syncUpdate()
    {
        if(!running_) return;

        if(do_sync_)
        {
            EnvironmentSync();
        }
        viewer_callbacks_();
    }


    void OpenRaveRviz::KinBodyMenuCallback(OpenRAVE::KinBodyPtr kinbody, std::string const &name)
    {
        menu_queue_ << "kinbody " << kinbody->GetName() << " " << name << '\n';
        selection_callbacks_(OpenRAVE::KinBody::LinkPtr(), OpenRAVE::RaveVector<float>(), OpenRAVE::RaveVector<float>());
    }

    void OpenRaveRviz::LinkMenuCallback(OpenRAVE::KinBody::LinkPtr link, std::string const &name)
    {
        menu_queue_ << "link " << link->GetParent()->GetName() << " " << link->GetName() << " " << name << '\n';
    }

    void OpenRaveRviz::ManipulatorMenuCallback(OpenRAVE::RobotBase::ManipulatorPtr manipulator, std::string const &name)
    {
        menu_queue_ << "manipulator " << manipulator->GetRobot()->GetName() << " " << manipulator->GetName() << " " << name << '\n';
    }

    visualization_msgs::InteractiveMarkerPtr OpenRaveRviz::CreateMarker() const
    {
        boost::shared_ptr<visualization_msgs::InteractiveMarker> interactive_marker = boost::make_shared<visualization_msgs::InteractiveMarker>();

        interactive_marker->header.frame_id = manager_->getFixedFrame().toStdString();
        interactive_marker->pose = or_interactivemarker::toROSPose(OpenRAVE::Transform());
        interactive_marker->name = boost::str(boost::format("GraphHandle[%p]") % interactive_marker.get());
        interactive_marker->scale = 1.0;

        interactive_marker->controls.resize(1);
        visualization_msgs::InteractiveMarkerControl &control = interactive_marker->controls.front();
        control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        control.always_visible = true;

        control.markers.resize(1);
        visualization_msgs::Marker &marker = control.markers.front();
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        return interactive_marker;
    }

    void OpenRaveRviz::ConvertPoints(float const *points, int num_points, int stride, std::vector<geometry_msgs::Point> *out_points) const
    {
        BOOST_ASSERT(points);
        BOOST_ASSERT(num_points >= 0);
        BOOST_ASSERT(stride >= 0 && stride % sizeof(float) == 0);
        BOOST_ASSERT(out_points);

        stride = stride / sizeof(float);

        out_points->resize(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint)
        {
            geometry_msgs::Point &out_point = out_points->at(ipoint);
            out_point.x = points[stride * ipoint + 0];
            out_point.y = points[stride * ipoint + 1];
            out_point.z = points[stride * ipoint + 2];
        }
    }

    void OpenRaveRviz::ConvertColors(float const *colors, int num_colors, bool has_alpha, std::vector<std_msgs::ColorRGBA> *out_colors) const
    {
        BOOST_ASSERT(colors);
        BOOST_ASSERT(num_colors >= 0);
        BOOST_ASSERT(out_colors);

        int stride;
        if (has_alpha)
        {
            stride = 4;
        }
        else
        {
            stride = 3;
        }

        out_colors->resize(num_colors);
        for (size_t icolor = 0; icolor < num_colors; ++icolor)
        {
            std_msgs::ColorRGBA &out_color = out_colors->at(icolor);
            out_color.r = colors[icolor * stride + 0];
            out_color.g = colors[icolor * stride + 1];
            out_color.b = colors[icolor * stride + 2];

            if (has_alpha)
            {
                out_color.a = colors[icolor * stride + 3];
            }
            else
            {
                out_color.a = 1.0;
            }
        }
    }

    void OpenRaveRviz::ConvertMesh(float const *points, int stride, int const *indices, int num_triangles, std::vector<geometry_msgs::Point> *out_points) const
    {
        BOOST_ASSERT(points);
        BOOST_ASSERT(stride > 0);
        BOOST_ASSERT(indices);
        BOOST_ASSERT(num_triangles >= 0);
        BOOST_ASSERT(out_points);

        auto const points_raw = reinterpret_cast<uint8_t const *>(points);

        out_points->resize(3 * num_triangles);
        for (int iindex = 0; iindex < num_triangles; ++iindex)
        {
            for (int ivertex = 0; ivertex < 3; ++ivertex)
            {
                int const index_offset = 3 * iindex + ivertex;
                float const *or_point = reinterpret_cast<float const *>(points_raw + stride * indices[index_offset]);

                geometry_msgs::Point &out_point = out_points->at(3 * iindex + ivertex);
                out_point.x = or_point[0];
                out_point.y = or_point[1];
                out_point.z = or_point[2];
            }
        }
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

static char *argv[1] = { const_cast<char *>("or_rviz") };
static int argc = 1;

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, std::string const& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
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
