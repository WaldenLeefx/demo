//
// Created by lee on 2021/8/17.
//

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
using namespace std;
void callBack(const geometry_msgs::PoseStampedPtr &ptr) {//const geometry_msgs::PoseStampedPtr &ptr,nav_msgs::OdometryPtr &ptr
    ofstream of;
    //of.open("/home/lee/mywork08_ws/src/lidarodomslam/src/02hector-hectorslam.txt",ios::app);
    of.open("/home/lee/mywork08_ws/src/lidarodomslam/src/traj/D_hector.txt",ios::app);
    if (!of.is_open()) {
        std::cout<<"open file failed."<<std::endl;
    }
    ros::Time time1;
    /*time1 = ptr->header.stamp;
    of<<time1<<" "<<ptr->pose.pose.position.x
             <<" "<<ptr->pose.pose.position.y
             <<" "<<ptr->pose.pose.position.z
             <<" "<<ptr->pose.pose.orientation.x
             <<" "<<ptr->pose.pose.orientation.y
             <<" "<<ptr->pose.pose.orientation.z
             <<" "<<ptr->pose.pose.orientation.w
             <<"\n";*/
    time1 = ptr->header.stamp;
    of<<time1<<" "<<ptr->pose.position.x
             <<" "<<ptr->pose.position.y
             <<" "<<ptr->pose.position.z
             <<" "<<ptr->pose.orientation.x
             <<" "<<ptr->pose.orientation.y
             <<" "<<ptr->pose.orientation.z
             <<" "<<ptr->pose.orientation.w
             <<"\n";
    of.close();
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "record");
    ros::NodeHandle nh;
    ros::Subscriber trajSub = nh.subscribe("slam_out_pose",10,callBack);//"slam_out_pose"
    ros::spin();
    return 0;
}