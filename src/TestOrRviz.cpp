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
    penv->Load(scenefilename); // load the scene
    sleep(2);

    /*
    std::vector<OpenRAVE::RobotBasePtr> robots;
    penv->GetRobots(robots);
    OpenRAVE::RobotBasePtr robot = robots[0];

    OpenRAVE::KinBodyPtr mug = penv->GetKinBody("plasticmugb1");


    float theta = 0;
    while(true)
    {
        usleep(10000);
        OpenRAVE::Transform t = robot->GetTransform();
        t.trans.z += 0.8 + 0.4 * sin(theta * 3);
        t.trans.x = 0.4 * cos(theta);
        t.trans.y = 0.3 * sin(theta);

        mug->SetTransform(t);

        theta += 0.01;
        std::vector<double> joints(robot->GetActiveManipulator()->GetArmIndices().size());
        OpenRAVE::IkParameterization params(t, OpenRAVE::IKP_Transform6D);

        if(robot->GetManipulators().at(0)->FindIKSolution(params, joints, OpenRAVE::IKFO_CheckEnvCollisions))
        {
            robot->SetDOFValues(joints, true, robot->GetManipulators().at(0)->GetArmIndices());
        }
        else
        {
            RAVELOG_INFO("NO solution\n");
        }
    }
    */
}

int main(int argc, char ** argv)
{
    //int num = 1;
    //QApplication app(argc, argv);
    //ros::init(argc, argv, "superviewer", ros::init_options::AnonymousName);
    string scenefilename = "/homes/mklingen/prdev/herb_description/ordata/robots/herb.robot.xml";
    string viewername = "superviewer";
    // parse the command line options
    int i = 1;
    while (i < argc)
    {
        if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "-?") == 0) || (strcmp(argv[i], "/?") == 0) || (strcmp(argv[i], "--help") == 0) || (strcmp(argv[i], "-help") == 0))
        {
            RAVELOG_INFO("orloadviewer [--num n] [--scene filename] viewername\n");
            return 0;
        }
        else if (strcmp(argv[i], "--scene") == 0)
        {
            scenefilename = argv[i + 1];
            i += 2;
        }
        else
            break;
    }

    if (i < argc)
    {
        viewername = argv[i++];
    }

    RaveInitialize(true); // start openrave core

    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    RaveSetDebugLevel(Level_Verbose);
    RaveLoadPlugin("./lib/superviewer.so");
    ViewerBasePtr viewer = RaveCreateViewer(penv, "superviewer");
    penv->Add(viewer);

    boost::thread threadRave(boost::bind(OpenraveThread,penv,scenefilename));

    viewer->main(true);

    threadRave.join();

    penv->Destroy(); // destroy
    return 0;
}
