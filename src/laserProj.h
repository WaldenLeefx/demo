//
// Created by lee on 2021/7/23.
//

#ifndef SAVEPCD_LASERPROJ_H
#define SAVEPCD_LASERPROJ_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserEcho.h>
#include <iostream>

class laserProj {
public:
    sensor_msgs::LaserScanPtr getFirstScan(const sensor_msgs::MultiEchoLaserScan& msg);
    sensor_msgs::LaserScanPtr getLastScan(const sensor_msgs::MultiEchoLaserScan& msg);
    sensor_msgs::LaserScanPtr getMostIntenseScan(const sensor_msgs::MultiEchoLaserScan& msg);

private:
    static void fillLaserScan(const sensor_msgs::MultiEchoLaserScan& msg, sensor_msgs::LaserScan& out);
    static size_t getFirstValue(const sensor_msgs::LaserEcho& ranges, float& range);
    static size_t getLastValue(const sensor_msgs::LaserEcho& ranges, float& range);
    static void getMostIntenseValue(const sensor_msgs::LaserEcho& ranges,const sensor_msgs::LaserEcho& intensities,float& range,float& intensity);
};


#endif //SAVEPCD_LASERPROJ_H
