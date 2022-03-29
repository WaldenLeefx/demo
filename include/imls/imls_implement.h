//
// Created by lee on 2021/5/10.
//

#ifndef SAVEPCD_IMLS_IMPLEMENT_H
#define SAVEPCD_IMLS_IMPLEMENT_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.8/pcl/filters/filter.h>
#include "/home/lee/catkinReadPCD_ws/src/savepcd/include/imls/imlsICP.h"
class imls_implement {
public:
    ros::NodeHandle nh;
    ros::Subscriber imls_sub;
    ros::Publisher imls_pub;
    bool m_isFirstFrame = false;
    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prePointCloud;
    IMLSICPMatcher m_imlsMatcher;
public:
    imls_implement();
    void imlsOdomCallBack(const sensor_msgs::PointCloud2Ptr &cloudIn);
    static void convertPointCloud(const sensor_msgs::PointCloud2Ptr &msg,std::vector<Eigen::Vector2d> &points);
};
#endif //SAVEPCD_IMLS_IMPLEMENT_H
