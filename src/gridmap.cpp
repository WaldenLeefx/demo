//
// Created by lee on 2021/11/4.
//
#include <iostream>
#include <ros/ros.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

nav_msgs::OccupancyGrid gridValue;
typedef struct index_ {
    int x;
    int y;
    void setIndex(int x_, int y_) {
        x = x_;
        y = y_;
    }
} gridIndex;
typedef struct map_params {
    int logOcc = 20;
    int logFree = -10;
    int logMax = 90;
    int logMin = 10;
    float res = 0;//
    int height = 0;//
    int width = 0;//
    int offsetX = 0;//
    int offsetY = 0;//
    float originX = 0;
    float originY = 0;
    void setRes(float res_) {res = res_;}
    void setArea(int h_,int w_) {
        height = h_;
        width = w_;
    }
    void setOffset(int ox,int oy) {
        offsetX = ox;
        offsetY = oy;
        originX = -width * res / 2;
        originY = -height * res / 2;
    }
} mapParams;
mapParams mapInfo;
std::vector<int> mapLineIdx;

std::vector<gridIndex> computeEndPointIndex(const pcl::PointCloud<pcl::PointXYZ> &p) {
    std::vector<gridIndex> idx;
    for (auto i : p.points) {
        int idxX = std::ceil((i.x - mapInfo.originX) / mapInfo.res) + mapInfo.offsetX;
        int idxY = std::ceil((i.y - mapInfo.originY) / mapInfo.res) + mapInfo.offsetY;
        gridIndex gIdx;
        gIdx.setIndex(idxX,idxY);
        idx.push_back(gIdx);
    }
    return idx;
}


int gridIndex2lineIndex(gridIndex idx) {
    int lineIndex;
    return lineIndex = idx.x + idx.y * mapInfo.width;
}

bool isValidLineIndex(int idx) {
    if (idx > 0 && idx < mapInfo.width * mapInfo.height)
        return true;
    return false;
}

void initMapParams() {
    mapInfo.setRes(0.05);
    mapInfo.setArea(1000,1000);
    mapInfo.setOffset(0,0);
    for (int i = 0; i < mapInfo.width * mapInfo.height; ++i) {
        mapLineIdx.push_back(-1);
    }
}

std::vector<gridIndex> Bresenham(int x0, int y0, int x1, int y1)
{
    gridIndex tmpIndex;
    std::vector<gridIndex> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }

    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            pointX = y;
            pointY = x;
        } else {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX) {
            y += ystep;
            error -= deltaX;
        }

        //不包含最后一个点．
        if (pointX == x1 && pointY == y1)
            continue;

        //保存所有的点
        tmpIndex.setIndex(pointX, pointY);
        gridIndexVector.push_back(tmpIndex);
    }
    return gridIndexVector;
}

void gridMap(const pcl::PointCloud<pcl::PointXYZ> &p, const nav_msgs::Odometry &o) {
    gridValue.info.resolution = mapInfo.res;
    gridValue.info.width = mapInfo.width;
    gridValue.info.height = mapInfo.height;
    gridValue.info.origin.position.x = mapInfo.originX;
    gridValue.info.origin.position.y = mapInfo.originY;
    gridValue.info.origin.position.z = 0;
    gridValue.info.origin.orientation.w = 1;
    gridValue.info.origin.orientation.x = 0;
    gridValue.info.origin.orientation.y = 0;
    gridValue.info.origin.orientation.z = 0;
    gridValue.data.resize(mapInfo.width * mapInfo.height);

    //odom coordinate
    double x = o.pose.pose.position.x;
    double y = o.pose.pose.position.y;
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(o.pose.pose.orientation,quaternion);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    double theta = yaw;
    std::cout<<"yaw: "<< yaw <<" rad"<<std::endl;
    std::cout<<"theta: "<< theta * 180 / M_PI <<" deg "<<std::endl;
    std::cout<<"cos()= "<<cos(theta)<<" sin()= "<<sin(theta)<<std::endl;

    pcl::PointCloud<pcl::PointXYZ> f_temp;
    pcl::PointCloud<pcl::PointXYZ> fw;//less flat point cloud in world coordinate
    fw = p;
    //pcl::fromROSMsg(*p,f_temp);
    for (auto i : f_temp.points) {
        double xw = cos(theta) * i.x - sin(theta) * i.y + x;
        double yw = sin(theta) + i.x + cos(theta) * i.y + y;
        fw.points.emplace_back(xw,yw,0);
    }
    //grid index for end point of a beam
    std::vector<gridIndex> endPtIdx = computeEndPointIndex(fw);
    //grid index for start point of a beam
    gridIndex startPtIdx;
    int sIdxX = std::ceil((x - mapInfo.originX) / mapInfo.res) + mapInfo.offsetX;
    int sIdxY = std::ceil((y - mapInfo.originY) / mapInfo.res) + mapInfo.offsetY;
    startPtIdx.setIndex(sIdxX,sIdxY);

    //free grid index between start pt and end pt
    int startLineIdx = gridIndex2lineIndex(startPtIdx);
    std::vector<int> occLineIdx;
    std::vector<int> freeLineIdx;
    freeLineIdx.push_back(startLineIdx);
    for (auto i : endPtIdx) {
        std::vector<gridIndex> freePtIdx = Bresenham(startPtIdx.x,startPtIdx.y,i.x,i.y);
        int endLineIdx = gridIndex2lineIndex(i);
        occLineIdx.push_back(endLineIdx);
        for (auto j : freePtIdx) {
            int fIdx = gridIndex2lineIndex(j);
            freeLineIdx.push_back(fIdx);
        }
    }

    for (auto i : occLineIdx) {
        if (isValidLineIndex(i)) {
            if (mapLineIdx.at(i) == -1) {
                if (50 + mapInfo.logOcc > mapInfo.logMax) {
                    mapLineIdx.at(i) = 100;
                } else {
                    mapLineIdx.at(i) = 50 + mapInfo.logOcc;
                }
            } else {
                if (mapLineIdx.at(i) + mapInfo.logOcc > mapInfo.logMax) {
                    mapLineIdx.at(i) = 100;
                } else {
                    mapLineIdx.at(i) = mapLineIdx.at(i) + mapInfo.logOcc;
                }
            }

        }
    }

    for (auto i : freeLineIdx) {
        if (isValidLineIndex(i)) {
            if (mapLineIdx.at(i) == -1) {
                if (50 + mapInfo.logFree < mapInfo.logMin) {
                    mapLineIdx.at(i) = 0;
                } else {
                    mapLineIdx.at(i) = 50 + mapInfo.logFree;
                }
            } else {
                if (mapLineIdx.at(i) + mapInfo.logFree < mapInfo.logMin) {
                    mapLineIdx.at(i) = 0;
                } else {
                    mapLineIdx.at(i) = mapLineIdx.at(i) + mapInfo.logFree;
                }
            }
        }
    }

    for (int i = 0; i < mapInfo.width * mapInfo.height; ++i) {
        if (mapLineIdx.at(i) == -1) {
            gridValue.data[i] = -1;
        } else {
            gridValue.data[i] = mapLineIdx.at(i);
        }
    }

    /*gridValue.header.stamp = o->header.stamp;
    gridValue.header.frame_id = "map";
    mapPub.publish(gridValue);*/

}

int main(int argc, char **argv) {
    ros::init(argc,argv,"gridmap");
    ros::NodeHandle nh;
    ros::Publisher mapPub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map",5, true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl04full.pcd",*cloud01);

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.pose.pose.position.x = 0,
    odometry.pose.pose.position.y = 0;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.x = 0;
    odometry.pose.pose.orientation.y = 0;
    odometry.pose.pose.orientation.z = 0;
    odometry.pose.pose.orientation.w = 1;
    initMapParams();
    gridMap(*cloud01,odometry);

    gridValue.header.stamp = ros::Time::now();
    gridValue.header.frame_id = "map";
    std::cout<<"1"<<std::endl;
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        mapPub.publish(gridValue);
        loop_rate.sleep();
        ros::spinOnce();
    }
    std::cout<<"2"<<std::endl;
    return 0;
}