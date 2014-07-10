#ifndef ORCONVERSIONS_H_
#define ORCONVERSIONS_H_
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <openrave/openrave.h>

namespace or_interactivemarker {

std_msgs::ColorRGBA toROSColor(OpenRAVE::Vector const &color);
geometry_msgs::Vector3 toROSVector(OpenRAVE::Vector const &or_vector);
geometry_msgs::Pose toROSPose(OpenRAVE::Transform const &or_pose);
geometry_msgs::Point toROSPoint(OpenRAVE::Vector const &or_point);
geometry_msgs::Quaternion toROSQuaternion(OpenRAVE::Vector const &or_point);

OpenRAVE::Vector toORPoint(geometry_msgs::Point const &point);
OpenRAVE::Vector toORQuaternion(geometry_msgs::Quaternion const &quat);
OpenRAVE::Transform toORPose(geometry_msgs::Pose const &pose);

}

#endif
