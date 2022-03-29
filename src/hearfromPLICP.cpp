//
// Created by lee on 2021/11/23.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <Eigen/Core>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2 ,nav_msgs::Odometry> msgSyncPolicy;

class messagesHandler {
protected:
    ros::NodeHandle nh;
public:
    messagesHandler() {
        message_filters::Subscriber<sensor_msgs::PointCloud2>* scan_;
        message_filters::Subscriber<nav_msgs::Odometry>* odom_;
        message_filters::Synchronizer<msgSyncPolicy>* msgSync_;
        scan_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/distortion_points",10);
        odom_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh,"/lsm_odom",10);
        msgSync_= new message_filters::Synchronizer<msgSyncPolicy>(msgSyncPolicy(10),*scan_,*odom_);
        msgSync_->registerCallback(boost::bind(&messagesHandler::msgCallBack,this,_1,_2));
    }
    void msgCallBack(const sensor_msgs::PointCloud2ConstPtr &c,
                     const nav_msgs::OdometryConstPtr &o) {
        std::ofstream ofstream;
        ofstream.open("/home/lee/mywork08_ws/src/lidarodomslam/src/traj/PL_ICP.txt",std::ios::app);
        if (!ofstream.is_open())
            std::cout<<"open file failed."<<std::endl;

        ros::Time time1;
        time1 = c->header.stamp;
        double tx,ty,tz,qx,qy,qz,qw;
        tx = o->pose.pose.position.x;
        ty = o->pose.pose.position.y;
        tz = o->pose.pose.position.z;
        qx = o->pose.pose.orientation.x;
        qy = o->pose.pose.orientation.y;
        qz = o->pose.pose.orientation.z;
        qw = o->pose.pose.orientation.w;
        ofstream<<time1<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<"\n";
        ofstream.close();

        tf::Quaternion quaternion;
        tf::quaternionMsgToTF(o->pose.pose.orientation,quaternion);
        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        std::cout<<"seq: "<<c->header.seq
                <<" tx: "<<tx
                <<" ty: "<<ty
                <<" theta: "<<yaw
                <<std::endl;
        Eigen::Matrix3d T;
        T << cos(yaw),-sin(yaw),tx,
             sin(yaw), cos(yaw),ty,
             0,0,1;
        std::cout<<"T=\n"<<T<<std::endl;

    }
};

int main(int argc,char** argv) {
    ros::init(argc, argv, "msg");
    messagesHandler messagesHandler;
    ros::spin();
    return 0;
}