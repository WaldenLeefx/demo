//
// Created by lee on 2021/5/10.
//

#ifndef SAVEPCD_UTILITY_H
#define SAVEPCD_UTILITY_H

#include <ros/ros.h>
//#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
//#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <nabo/nabo.h>
#include <unsupported/Eigen/Polynomials>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.8/pcl/filters/filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <unordered_map>
class ParamServer {
public:
    ros::NodeHandle nh;
    ros::Subscriber imls_sub;
    ros::Publisher imlsOdom_pub;
    ros::Publisher imlsPath_pub;
    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prePointCloud;
    bool m_isFirstFrame = false;

    std::unordered_map<double,Eigen::Matrix3d> SRImap;

    bool isHashMatrix = false;
    std::vector<double> SRIAngle;
    double angle_increment = 0.00871451;




};

#endif //SAVEPCD_UTILITY_H
