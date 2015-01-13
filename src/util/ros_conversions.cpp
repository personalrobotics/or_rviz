#include "util/ros_conversions.h"

namespace or_interactivemarker {
namespace util {

OpenRAVE::geometry::RaveVector<double> ToRaveVector(Ogre::Vector3 const & vec)
{
    return OpenRAVE::geometry::RaveVector<double>(vec.x, vec.y, vec.z);
}

Ogre::Vector3 ToOgreVector(OpenRAVE::geometry::RaveVector<double> const &vec)
{
    return Ogre::Vector3(vec.x, vec.y, vec.z);
}

OpenRAVE::geometry::RaveVector<double> ToRaveQuaternion(Ogre::Quaternion const &quat)
{
    return OpenRAVE::geometry::RaveVector<double>(quat.x, quat.y, quat.z, quat.w);
}

Ogre::Quaternion ToOgreQuaternion(OpenRAVE::geometry::RaveVector<double> const &vec)
{
    return Ogre::Quaternion(vec.x, vec.y, vec.z, vec.w);
}

geometry_msgs::Pose ToGeomMsgPose(OpenRAVE::geometry::RaveTransform<double> const &transform)
{
    geometry_msgs::Pose toReturn;
    toReturn.position.x = transform.trans.x;
    toReturn.position.y = transform.trans.y;
    toReturn.position.z = transform.trans.z;
    toReturn.orientation.w = transform.rot.x;
    toReturn.orientation.x = transform.rot.y;
    toReturn.orientation.y = transform.rot.z;
    toReturn.orientation.z = transform.rot.w;
    return toReturn;
}


OpenRAVE::geometry::RaveTransform<double> ToRaveTransform(geometry_msgs::Pose const &pose)
{
    OpenRAVE::geometry::RaveVector<double> const quat(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    );
    OpenRAVE::geometry::RaveVector<double> const translation(
        pose.position.x,
        pose.position.y,
        pose.position.z
    );

    return OpenRAVE::geometry::RaveTransform<double>(quat, translation);
}

}
}
