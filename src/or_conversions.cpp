#include "or_conversions.h"

using std_msgs::ColorRGBA;
using geometry_msgs::Vector3;
using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;

namespace or_interactivemarker {

ColorRGBA toROSColor(OpenRAVE::Vector const &or_color)
{
    ColorRGBA color;
    color.r = or_color.x;
    color.g = or_color.y;
    color.b = or_color.z;
    color.a = or_color.w;
    return color;
}

Vector3 toROSVector(OpenRAVE::Vector const &or_vector)
{
    Vector3 v;
    v.x = or_vector.x;
    v.y = or_vector.y;
    v.z = or_vector.z;
    return v;
}

Pose toROSPose(OpenRAVE::Transform const &or_pose)
{
    Pose pose;
    pose.position = toROSPoint(or_pose.trans);
    pose.orientation = toROSQuaternion(or_pose.rot);
    return pose;
}

Point toROSPoint(OpenRAVE::Vector const &or_point)
{
    Point point;
    point.x = or_point.x;
    point.y = or_point.y;
    point.z = or_point.z;
    return point;
}

Quaternion toROSQuaternion(OpenRAVE::Vector const &or_quat)
{
    Quaternion quaternion;
    quaternion.w = or_quat[0];
    quaternion.x = or_quat[1];
    quaternion.y = or_quat[2];
    quaternion.z = or_quat[3];
    return quaternion;
}

}
