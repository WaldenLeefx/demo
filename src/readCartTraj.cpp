//
// Created by lee on 2021/5/17.
//
#include "../include/utility.h"
#include <nav_msgs/Odometry.h>
using namespace std;
namespace lidar_slam {
    class readCartTraj : ParamServer{
    public:
        ros::Subscriber cartTraj_sub;
        readCartTraj() {
            cartTraj_sub = nh.subscribe("/laser_odom",50,&readCartTraj::saveCartTraj,this);
        }

        void saveCartTraj(const nav_msgs::OdometryPtr &traj) {
            std::ofstream cart_traj_of;
            cart_traj_of.open("/home/lee/mywork08_ws/src/lidarodomslam/src/traj/D_carto.txt",ios::app);
            if (!cart_traj_of.is_open())
                cout<<"open file failed."<<endl;

            ros::Time time1;
            time1 = traj->header.stamp;
            double tx,ty,tz,qx,qy,qz,qw;
            tx = traj->pose.pose.position.x;
            ty = traj->pose.pose.position.y;
            tz = traj->pose.pose.position.z;
            qx = traj->pose.pose.orientation.x;
            qy = traj->pose.pose.orientation.y;
            qz = traj->pose.pose.orientation.z;
            qw = traj->pose.pose.orientation.w;
            cart_traj_of<<time1<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<"\n";
            cart_traj_of.close();
        }
    };
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "readCartTraj");

    lidar_slam::readCartTraj trajectory;

    ros::spin();

    return 0;
}