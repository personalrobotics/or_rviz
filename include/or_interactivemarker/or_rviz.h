#ifndef SUPERVIEWER_H
#define SUPERVIEWER_H

//#include "RenderWindow.h"
#include <openrave/openrave.h>
#include <openrave/viewer.h>
#include <QMainWindow>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_frame.h>
#include "Plugins/EnvironmentDisplay.h"
#include <map>
#include <boost/signals2.hpp>
#include "Markers/KinBodyMarker.h"

namespace or_rviz
{

    /////
    // These classes need to exist to make OpenRAVEs callback stuff work.
    // They are classes which just erase references to themselves in a map.
    // When the caller deletes the shared pointer to them, they are erased from
    // the callback registry.
    // Basically follows qtcoinviewer's registry stuff.
    /////-----------
    class ViewerCallbackRegistry: public OpenRAVE::UserData
    {
        public:
            ViewerCallbackRegistry(size_t index, std::map<size_t, OpenRAVE::ViewerBase::ViewerImageCallbackFn>* callbacks) { m_index = index; m_callbacks = callbacks; }
            virtual ~ViewerCallbackRegistry() { m_callbacks->erase(m_index); }
            size_t m_index;
            std::map<size_t, OpenRAVE::ViewerBase::ViewerImageCallbackFn>* m_callbacks;
    };

    class SyncCallbackRegistry: public OpenRAVE::UserData
    {
        public:
            SyncCallbackRegistry(size_t index, std::map<size_t, OpenRAVE::ViewerBase::ViewerThreadCallbackFn>* callbacks) { m_index = index; m_callbacks = callbacks; }
            virtual ~SyncCallbackRegistry() { m_callbacks->erase(m_index); }
            size_t m_index;
            std::map<size_t, OpenRAVE::ViewerBase::ViewerThreadCallbackFn>* m_callbacks;
    };

    class SelectCallbackRegistry : public OpenRAVE::UserData
    {
        public:
            SelectCallbackRegistry(size_t index, std::map<size_t, OpenRAVE::ViewerBase::ItemSelectionCallbackFn>* callbacks) { m_index = index; m_callbacks = callbacks; }
            virtual ~SelectCallbackRegistry() { m_callbacks->erase(m_index); }
            size_t m_index;
            std::map<size_t, OpenRAVE::ViewerBase::ItemSelectionCallbackFn>* m_callbacks;
    };


    ////--------- Callback classes

    ////
    // This class allows the viewer to register "graphs" with OpenRAVE. They are implemented in RVIZ as ogre dynamic objects.
    //
    class RvizGraphHandle : public OpenRAVE::GraphHandle
    {
        public:
            RvizGraphHandle();
            RvizGraphHandle(Ogre::SceneNode* node);
            RvizGraphHandle(Ogre::SceneNode* node, Ogre::ManualObject* object);
            virtual ~RvizGraphHandle();

            inline Ogre::MovableObject* GetObject() { return m_object;}
            inline void SetObject(Ogre::MovableObject* object) { m_object = object; }

            virtual void SetShow(bool show);
            virtual void SetTransform(const OpenRAVE::RaveTransform<float>& transform);

            void Initialize();

        protected:
            Ogre::MovableObject* m_object;
            Ogre::SceneNode* m_node;

    };

    struct RenderTargetRequest
    {
            int width;
            int height;
            Ogre::PixelFormat format;
            std::string name;
            bool valid;
            Ogre::TexturePtr texture;
            unsigned char * data;
            Ogre::PixelBox* pixelBox;
    };

    class OpenRaveRviz : public rviz::VisualizationFrame, public OpenRAVE::ViewerBase
    {
        Q_OBJECT
        public:
            OpenRaveRviz(OpenRAVE::EnvironmentBasePtr env, QWidget * parent = 0, Qt::WindowFlags flags = 0);
            virtual ~OpenRaveRviz();


            // OPENRAVE API overriding

            // OpenRAVE calls on us to run the main loop...
            virtual int main(bool show=true);
            virtual void quitmainloop();
            virtual void RemoveKinBody (OpenRAVE::KinBodyPtr pbody);
            virtual void Reset();

            // OpenRAVE API can set the background color for us...
            virtual void SetBkgndColor(const OpenRAVE::RaveVector<float> &color);

            // registers a function with the viewer that gets called everytime mouse button is clicked
            virtual OpenRAVE::UserDataPtr RegisterItemSelectionCallback(const ItemSelectionCallbackFn &fncallback);

            // registers a function with the viewer that gets called for every new image rendered.
            virtual OpenRAVE::UserDataPtr RegisterViewerImageCallback(const ViewerImageCallbackFn &fncallback);

            // registers a function with the viewer that gets called in the viewer's GUI thread for every cycle the viewer refreshes at
            virtual OpenRAVE::UserDataPtr RegisterViewerThreadCallback(const ViewerThreadCallbackFn &fncallback);

            // controls whether the viewer synchronizes with the newest environment automatically
            virtual void SetEnvironmentSync(bool update);

            // forces synchronization with the environment, returns when the environment is fully synchronized.
            virtual void EnvironmentSync();

            // Viewer size and position can be set outside in the
            // OpenRAVE API
            virtual void SetSize (int w, int h);
            virtual void Move (int x, int y);

            // Name is set by OpenRAVE outside
            virtual void SetName (const std::string &name);
            virtual const std::string & GetName () const;

            // Keeps camera transform consistent
            virtual void UpdateCameraTransform();

            // Set the camera transformation.
            virtual void SetCamera (const OpenRAVE::RaveTransform<float> &trans, float focalDistance=0);
            virtual void SetCamera (Ogre::Camera* camera, const OpenRAVE::RaveTransform<float> &trans, float focalDistance=0);

            // Return the current camera transform that the viewer is rendering the environment at.
            virtual OpenRAVE::RaveTransform<float>  GetCameraTransform() const;

            // Return the closest camera intrinsics that the viewer is rendering the environment at.
            virtual OpenRAVE::geometry::RaveCameraIntrinsics<float> GetCameraIntrinsics();

            // Renders a 24bit RGB image of dimensions width and height from the current scene.
            virtual bool GetCameraImage(std::vector<uint8_t> &memory, int width, int height, const OpenRAVE::RaveTransform<float> &t, const OpenRAVE::SensorBase::CameraIntrinsics &intrinsics);

            inline bool IsAutoSyncEnabled() { return m_autoSync; }
            inline void SetAutoSync(bool value) { m_autoSync = value; }


            void RegisterRenderTargetRequest(RenderTargetRequest* request);
            void HandleRenderTargetRequests();
            void DeleteRequest(RenderTargetRequest* request);

            unsigned char* WaitForRenderTarget(int w, int h, int depth, Ogre::PixelFormat format, std::string name);

            void Clear();
            std::string GetEnvironmentHash(OpenRAVE::EnvironmentBasePtr env);

            OpenRAVE::EnvironmentBasePtr GetCurrentViewEnv() { return m_currentViewEnv; }
            void SetCurrentViewEnv(OpenRAVE::EnvironmentBasePtr value) {  m_currentViewEnv = value; }

            uchar* OffscreenRender(int width, int height, int depth);
            uchar* WriteCurrentView(int& width, int& height, int& depth);


            // RAVE Commands
            //virtual OpenRAVE::UserDataPtr RegisterItemSelectionCallback(OpenRAVE::ViewerBase::ItemSelectionCallbackFn const &fncallback);
            //virtual OpenRAVE::UserDataPtr RegisterViewerThreadCallback(OpenRAVE::ViewerBase::ViewerThreadCallbackFn const &fncallback);

            public Q_SLOTS:
                 void syncUpdate();
                 void loadEnvironment();
                 void setEnvironment(bool checked);

        protected:

            rviz::VisualizationManager* m_rvizManager;
            rviz::RenderPanel* m_mainRenderPanel;
            Ogre::RenderWindow* m_offscreenRenderer;
            Ogre::Camera* m_offscreenCamera;
            EnvironmentDisplay* m_envDisplay;
            QMenu* m_environmentsMenu;
            std::map<size_t, ViewerImageCallbackFn> m_renderCallbacks;
            std::map<size_t, ViewerThreadCallbackFn> m_syncCallbacks;
            std::map<size_t, ItemSelectionCallbackFn> m_itemCallbacks;


            boost::mutex m_targetMutex;
            std::vector<RenderTargetRequest*> m_renderTargetRequests;

            std::vector<OpenRAVE::GraphHandlePtr> m_graphsToInitialize;

            QAction* LoadEnvironmentAction();
            void UpdateDisplay();
            void HandleMenus();
            virtual void paintEvent(QPaintEvent* e);
            virtual bool eventFilter(QObject *o, QEvent *e);



            OpenRAVE::EnvironmentBasePtr m_currentViewEnv;

            bool m_autoSync;
            std::string m_name;
            boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
            bool running_;
            bool do_sync_;
            int graph_id_;

            typedef void ViewerCallbackFn();
            typedef bool SelectionCallbackFn(OpenRAVE::KinBody::LinkPtr plink, OpenRAVE::RaveVector<float>, OpenRAVE::RaveVector<float>);

            boost::signals2::signal<ViewerCallbackFn> viewer_callbacks_;
            boost::signals2::signal<SelectionCallbackFn> selection_callbacks_;
            std::stringstream menu_queue_;

            bool AddMenuEntryCommand(std::ostream &out, std::istream &in);
            bool GetMenuSelectionCommand(std::ostream &out, std::istream &in);

            void KinBodyMenuCallback(OpenRAVE::KinBodyPtr kinbody, std::string const &name);
            void LinkMenuCallback(OpenRAVE::KinBody::LinkPtr link, std::string const &name);
            void ManipulatorMenuCallback(OpenRAVE::RobotBase::ManipulatorPtr manipulator, std::string const &name);

            visualization_msgs::InteractiveMarkerPtr CreateMarker() const;
            void ConvertPoints(float const *points, int num_points, int stride, std::vector<geometry_msgs::Point> *out_points) const;
            void ConvertColors(float const *colors, int num_colors, bool has_alpha, std::vector<std_msgs::ColorRGBA> *out_colors) const;
            void ConvertMesh(float const *points, int stride, int const *indices, int num_triangles, std::vector<geometry_msgs::Point> *out_points) const;

            // Overloading OPENRAVE drawing functions....
            virtual OpenRAVE::GraphHandlePtr  plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector< float > &color, int drawstyle=0);
            virtual OpenRAVE::GraphHandlePtr  plot3 (const float *ppoints, int numPoints, int stride, float fPointSize, const float *colors, int drawstyle=0, bool bhasalpha=false);
            virtual OpenRAVE::GraphHandlePtr  drawlinestrip (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawlinestrip (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors);
            virtual OpenRAVE::GraphHandlePtr  drawlinelist (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawlinelist (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors);
            virtual OpenRAVE::GraphHandlePtr  drawarrow (const OpenRAVE::RaveVector< float > &p1, const OpenRAVE::RaveVector< float > &p2, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawbox (const OpenRAVE::RaveVector< float > &vpos, const OpenRAVE::RaveVector< float > &vextents);
            virtual OpenRAVE::GraphHandlePtr  drawplane (const OpenRAVE::RaveTransform< float > &tplane, const OpenRAVE::RaveVector< float > &vextents, const boost::multi_array< float, 3 > &vtexture);
            virtual OpenRAVE::GraphHandlePtr  drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array< float, 2 > &colors);

            // Using OpenGL
            virtual OpenRAVE::GraphHandlePtr  plot3_native (const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector< float > &color, int drawstyle=0);
            virtual OpenRAVE::GraphHandlePtr  plot3_native (const float *ppoints, int numPoints, int stride, float fPointSize, const float *colors, int drawstyle=0, bool bhasalpha=false);
            virtual OpenRAVE::GraphHandlePtr  drawlinestrip_native (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawlinestrip_native (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors);
            virtual OpenRAVE::GraphHandlePtr  drawlinelist_native (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawlinelist_native (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors);
            virtual OpenRAVE::GraphHandlePtr  drawarrow_native (const OpenRAVE::RaveVector< float > &p1, const OpenRAVE::RaveVector< float > &p2, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawbox_native (const OpenRAVE::RaveVector< float > &vpos, const OpenRAVE::RaveVector< float > &vextents);
            virtual OpenRAVE::GraphHandlePtr  drawplane_native (const OpenRAVE::RaveTransform< float > &tplane, const OpenRAVE::RaveVector< float > &vextents, const boost::multi_array< float, 3 > &vtexture);
            virtual OpenRAVE::GraphHandlePtr  drawtrimesh_native (const float *ppoints, int stride, const int *pIndices, int numTriangles, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawtrimesh_native (const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array< float, 2 > &colors);

            // Using interactive marker
            virtual OpenRAVE::GraphHandlePtr  plot3_marker (const float *ppoints, int numPoints, int stride, float fPointSize, const OpenRAVE::RaveVector< float > &color, int drawstyle=0);
            virtual OpenRAVE::GraphHandlePtr  plot3_marker (const float *ppoints, int numPoints, int stride, float fPointSize, const float *colors, int drawstyle=0, bool bhasalpha=false);
            virtual OpenRAVE::GraphHandlePtr  drawlinestrip_marker (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawlinestrip_marker (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors);
            virtual OpenRAVE::GraphHandlePtr  drawlinelist_marker (const float *ppoints, int numPoints, int stride, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawlinelist_marker (const float *ppoints, int numPoints, int stride, float fwidth, const float *colors);
            virtual OpenRAVE::GraphHandlePtr  drawarrow_marker (const OpenRAVE::RaveVector< float > &p1, const OpenRAVE::RaveVector< float > &p2, float fwidth, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawbox_marker (const OpenRAVE::RaveVector< float > &vpos, const OpenRAVE::RaveVector< float > &vextents);
            virtual OpenRAVE::GraphHandlePtr  drawplane_marker (const OpenRAVE::RaveTransform< float > &tplane, const OpenRAVE::RaveVector< float > &vextents, const boost::multi_array< float, 3 > &vtexture);
            virtual OpenRAVE::GraphHandlePtr  drawtrimesh_marker (const float *ppoints, int stride, const int *pIndices, int numTriangles, const OpenRAVE::RaveVector< float > &color);
            virtual OpenRAVE::GraphHandlePtr  drawtrimesh_marker (const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array< float, 2 > &colors);
    };
}




#endif
