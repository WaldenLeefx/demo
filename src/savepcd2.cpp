//
// Created by lee on 2021/7/31.
//

#include <ros/ros.h>

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
sensor_msgs::PointCloud2,
sensor_msgs::PointCloud2,
sensor_msgs::PointCloud2,
sensor_msgs::PointCloud2> cloudSyncPolicy;

class cloudHandler2 {
public:
    int count = 0;
    bool flag = false;
public:
    cloudHandler2() {
        message_filters::Subscriber<sensor_msgs::PointCloud2>* fullCloud_;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* cloudSharp_;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* cloudLessSharp_;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* cloudFlat_;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* cloudLessFlat;
        message_filters::Synchronizer<cloudSyncPolicy>* cloudSync_;
        fullCloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/distortion_points",10);
        cloudSharp_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/sharp",10);
        cloudLessSharp_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/less_sharp",10);
        cloudFlat_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/flat",10);
        cloudLessFlat = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/less_flat",10);
        cloudSync_ = new message_filters::Synchronizer<cloudSyncPolicy>(cloudSyncPolicy(10),*fullCloud_,*cloudSharp_,*cloudLessSharp_,*cloudFlat_,*cloudLessFlat);
        cloudSync_->registerCallback(boost::bind(&cloudHandler2::cloudCB,this,_1,_2,_3,_4,_5));
    }
    void cloudCB(const sensor_msgs::PointCloud2ConstPtr &cloudIn,
                 const sensor_msgs::PointCloud2ConstPtr &cloudSharp,
                 const sensor_msgs::PointCloud2ConstPtr &cloudLessSharp,
                 const sensor_msgs::PointCloud2ConstPtr &cloudFlat,
                 const sensor_msgs::PointCloud2ConstPtr &cloudLessFlat) {
        pcl::PointCloud<pcl::PointXYZ> pointCloudIn;
        pcl::PointCloud<pcl::PointXYZ> pointCloudSharp;
        pcl::PointCloud<pcl::PointXYZ> pointCloudLessSharp;
        pcl::PointCloud<pcl::PointXYZ> pointCloudFlat;
        pcl::PointCloud<pcl::PointXYZ> pointCloudLessFlat;
        pcl::fromROSMsg(*cloudIn,pointCloudIn);
        pcl::fromROSMsg(*cloudSharp,pointCloudSharp);
        pcl::fromROSMsg(*cloudLessSharp,pointCloudLessSharp);
        pcl::fromROSMsg(*cloudFlat,pointCloudFlat);
        pcl::fromROSMsg(*cloudLessFlat,pointCloudLessFlat);

        if (!flag) {
            count = pointCloudIn.header.seq;
            flag = true;
        }

        if (count == pointCloudIn.header.seq) {
            std::cout<<"seq: "<<pointCloudIn.header.seq<<std::endl;
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl04full.pcd",pointCloudIn);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl04sharp.pcd",pointCloudSharp);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl04lessSharp.pcd",pointCloudLessSharp);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl04Flat.pcd",pointCloudFlat);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl04lessFlat.pcd",pointCloudLessFlat);
        }
        if (count + 3 == pointCloudIn.header.seq) {
            std::cout<<"seq: "<<pointCloudIn.header.seq<<std::endl;
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl05full.pcd",pointCloudIn);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl05sharp.pcd",pointCloudSharp);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl05lessSharp.pcd",pointCloudLessSharp);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl05Flat.pcd",pointCloudFlat);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl05lessFlat.pcd",pointCloudLessFlat);
        }
        if (count + 6 == pointCloudIn.header.seq) {
            std::cout<<"seq: "<<pointCloudIn.header.seq<<std::endl;
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl06full.pcd",pointCloudIn);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl06sharp.pcd",pointCloudSharp);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl06lessSharp.pcd",pointCloudLessSharp);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl06Flat.pcd",pointCloudFlat);
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/zl06lessFlat.pcd",pointCloudLessFlat);
        }

    }


protected:
    ros::NodeHandle nh;
    ros::Subscriber pointSub;
};
int main(int argc,char** argv) {
    ros::init(argc, argv, "savePCD");
    cloudHandler2 handler;
    ros::spin();
    return 0;
}