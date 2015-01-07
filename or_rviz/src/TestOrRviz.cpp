#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include "OpenRaveRviz.h"
#include <qapplication.h>

using namespace OpenRAVE;
using namespace std;


void OpenraveThread(EnvironmentBasePtr penv, std::string scenefilename)
{
    penv->Load(scenefilename);
    sleep(2);
}

int main(int argc, char ** argv)
{
    string scenefilename = "/opt/pr/herb_description/ordata/robots/herb.robot.xml";
    string viewername = "or_rviz";

    RaveInitialize(true);

    EnvironmentBasePtr penv = RaveCreateEnvironment();
    RaveSetDebugLevel(Level_Verbose);

    ViewerBasePtr viewer = RaveCreateViewer(penv, "or_rviz");
    penv->Add(viewer);

    boost::thread threadRave(boost::bind(OpenraveThread,penv,scenefilename));

    viewer->main(true);

    threadRave.join();

    penv->Destroy();
    return 0;
}
