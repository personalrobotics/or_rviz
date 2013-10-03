#ifndef SUPERVIEWER_H
#define SUPERVIEWER_H

//#include "RenderWindow.h"
#include <openrave/openrave.h>
#include <openrave/viewer.h>
#include <QMainWindow>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include "Plugins/KinBodyDisplay.h"
#include "Plugins/KinBodyVisual.h"
#include "Plugins/LinkVisual.h"
#include <map>

namespace superviewer
{
    class SuperViewer : public QMainWindow, public OpenRAVE::ViewerBase
    {
        Q_OBJECT
        public:
            SuperViewer(OpenRAVE::EnvironmentBasePtr env, QWidget * parent = 0, Qt::WindowFlags flags = 0);
            virtual ~SuperViewer();


            // OPENRAVE API overriding

            // OpenRAVE calls on us to run the main loop...
            virtual int main(bool show=true);
            virtual void quitmainloop();

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

            // Return the current camera transform that the viewer is rendering the environment at.
            virtual OpenRAVE::RaveTransform<float>  GetCameraTransform() const;

            // Return the closest camera intrinsics that the viewer is rendering the environment at.
            virtual OpenRAVE::geometry::RaveCameraIntrinsics<float> GetCameraIntrinsics();

            // Renders a 24bit RGB image of dimensions width and height from the current scene.
            virtual bool GetCameraImage(std::vector<uint8_t> &memory, int width, int height, const OpenRAVE::RaveTransform<float> &t, const OpenRAVE::SensorBase::CameraIntrinsics &intrinsics);

            inline bool IsAutoSyncEnabled() { return m_autoSync; }
            inline void SetAutoSync(bool value) { m_autoSync = value; }

            virtual void RemoveKinBody(OpenRAVE::KinBodyPtr kinBody);

            inline bool HasKinBody(std::string name) { return m_kinBodies.find(name) != m_kinBodies.end(); }


            public Q_SLOTS:
                 void syncUpdate();

        protected:

            rviz::VisualizationManager* m_rvizManager;
            rviz::RenderPanel* m_mainRenderPanel;
            std::map<std::string, KinBodyDisplay*> m_kinBodies;


            bool m_autoSync;
            std::string m_name;

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

    };
}




#endif
