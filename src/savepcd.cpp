//
// Created by lee on 2021/4/30.
//
#include <ros/ros.h>

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
class cloudHandler {
public:
    int count = 0;
    bool flag = false;
public:
    cloudHandler() {
        pointSub = nh.subscribe("distortion_points",100,&cloudHandler::cloudCB,this);
    }
    void cloudCB(const sensor_msgs::PointCloud2 &cloud) {
        pcl::PointCloud<pcl::PointXYZ> cloudIn;
        pcl::PointCloud<pcl::PointXYZ> cloudOut;

        pcl::fromROSMsg(cloud,cloudIn);
        if (!flag) {
            count = cloudIn.header.seq;
            flag = true;
        }
        if (count == cloudIn.header.seq) {
            std::cout<<"seq: "<<cloudIn.header.seq<<std::endl;
            std::cout<<"size: "<<cloudIn.points.size()<<std::endl;
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl01.pcd",cloudIn);
        }

        if (count + 2 == cloudIn.header.seq) {
            std::cout<<"seq: "<<cloudIn.header.seq<<std::endl;
            std::cout<<"size: "<<cloudIn.points.size()<<std::endl;
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl02.pcd",cloudIn);
        }

        if (count + 4 == cloudIn.header.seq) {
            std::cout<<"seq: "<<cloudIn.header.seq<<std::endl;
            std::cout<<"size: "<<cloudIn.points.size()<<std::endl;
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl03.pcd",cloudIn);
            ros::Duration(5).sleep();
        }



    }


protected:
    ros::NodeHandle nh;
    ros::Subscriber pointSub;
};
int main(int argc,char** argv){
ros::init(argc,argv,"savePCD");
cloudHandler handler;
ros::spin();
return 0;
}