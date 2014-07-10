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
    v.x = or_vector[0];
    v.y = or_vector[1];
    v.z = or_vector[2];
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

Quaternion toROSQuaternion(OpenRAVE::Vector const &or_point)
{
    Quaternion quaternion;
    quaternion.x = or_point.y;
    quaternion.y = or_point.z;
    quaternion.z = or_point.w;
    quaternion.w = or_point.x;
    return quaternion;
}

}
