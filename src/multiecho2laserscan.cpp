//
// Created by lee on 2021/7/23.
//

#include "../include/utility.h"
//#include "laserProj.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserEcho.h>
#include <iostream>
class cloudHandler{
public:
    cloudHandler() {
        laserScanSub = nhh.subscribe("/horizontal_laser_2d",100,&cloudHandler::cloudCB,this);
        laserScanPub01 = nhh.advertise<sensor_msgs::LaserScan>("laserScan01",100);
        laserScanPub02 = nhh.advertise<sensor_msgs::LaserScan>("laserScan02",100);
        laserScanPub03 = nhh.advertise<sensor_msgs::LaserScan>("laserScan03",100);
    }

    void cloudCB(const sensor_msgs::MultiEchoLaserScan& multiScan) {
        sensor_msgs::LaserScanPtr scanPtr01;
        sensor_msgs::LaserScanPtr scanPtr02;
        sensor_msgs::LaserScanPtr scanPtr03;
        scanPtr01 = getFirstScan(multiScan);
        laserScanPub01.publish(scanPtr01);
        scanPtr02 = getLastScan(multiScan);
        laserScanPub02.publish(scanPtr02);
        scanPtr03 = getMostIntenseScan(multiScan);
        laserScanPub03.publish(scanPtr03);
    }

    static sensor_msgs::LaserScanPtr getFirstScan(const sensor_msgs::MultiEchoLaserScan &msg) {
        sensor_msgs::LaserScanPtr out(new sensor_msgs::LaserScan());
        fillLaserScan(msg,*out);
        out->ranges.resize(msg.ranges.size());
        if (msg.ranges.size() == msg.intensities.size()) {
            out->intensities.resize(msg.intensities.size());
        }
        for (int i = 0; i < out->ranges.size(); ++i) {
            size_t index = getFirstValue(msg.ranges[i],out->ranges[i]);
            if (out->intensities.size() > 0) {
                if (msg.intensities[i].echoes.size() > 0) {
                    out->intensities[i] = msg.intensities[i].echoes[index];
                } else {
                    out->intensities[i] = 0.0;
                }
            }
        }
        return out;
    }

    sensor_msgs::LaserScanPtr getLastScan(const sensor_msgs::MultiEchoLaserScan &msg) {
        sensor_msgs::LaserScanPtr out(new sensor_msgs::LaserScan());
        fillLaserScan(msg,*out);
        out->ranges.resize(msg.ranges.size());
        if (msg.ranges.size() == msg.intensities.size()) {
            out->intensities.resize(msg.intensities.size());
        }

        for (int i = 0; i < out->ranges.size(); ++i) {
            size_t index = getLastValue(msg.ranges[i],out->ranges[i]);
            if (out->intensities.size() > 0) {
                if (msg.intensities[i].echoes.size() > 0) {
                    out->intensities[i] = msg.intensities[i].echoes[index];
                } else {
                    out->intensities[i] = 0.0;
                }
            }
        }
        return out;
    }

    sensor_msgs::LaserScanPtr getMostIntenseScan(const sensor_msgs::MultiEchoLaserScan &msg) {
        sensor_msgs::LaserScanPtr out(new sensor_msgs::LaserScan());
        fillLaserScan(msg,*out);
        if (msg.ranges.size() == msg.intensities.size()) {
            out->ranges.resize(msg.ranges.size());
            out->intensities.resize(msg.intensities.size());
        } else {
            std::stringstream ss;
            ss << "getMostIntenseScan::Size of ranges does not equal size of intensities, cannot create scan.";
            throw std::runtime_error(ss.str());
        }
        std::cout<<"max"<<std::endl;
        float max = 0;
        for (int i = 0; i < out->intensities.size(); ++i) {
            getMostIntenseValue(msg.ranges[i], msg.intensities[i], out->ranges[i], out->intensities[i]);
            max = max > out->ranges[i] ? max : out->ranges[i];
        }
        std::cout<<max<<std::endl;
        for (int i = 0; i < out->intensities.size(); ++i) {
            if (max == out->ranges[i]) {
                out->ranges[i] = 0;
            }
        }
        return out;
    }

    static void fillLaserScan(const sensor_msgs::MultiEchoLaserScan &msg, sensor_msgs::LaserScan &out) {
        out.header = msg.header;
        out.angle_min = msg.angle_min;
        out.angle_max = msg.angle_max;
        out.angle_increment = msg.angle_increment;
        out.time_increment = msg.time_increment;
        out.scan_time = msg.scan_time;
        out.range_min = msg.range_min;
        out.range_max = msg.range_max;
    }

    static size_t getFirstValue(const sensor_msgs::LaserEcho &ranges, float &range) {
        if (ranges.echoes.size() > 0) {
            size_t index = 0;
            range = ranges.echoes[index];
            return index;
        }
        range = std::numeric_limits<float>::quiet_NaN();
        return 0;
    }

    size_t getLastValue(const sensor_msgs::LaserEcho &ranges, float &range) {
        if (ranges.echoes.size() > 0) {
            size_t index = ranges.echoes.size() - 1;
            range = ranges.echoes[index];
            return index;
        }
        range = std::numeric_limits<float>::quiet_NaN();
        return 0;
    }

    void getMostIntenseValue(const sensor_msgs::LaserEcho &ranges, const sensor_msgs::LaserEcho &intensities,
                                        float &range, float &intensity) {
        if (intensities.echoes.size() == 0) {
            range = std::numeric_limits<float>::quiet_NaN();
            intensity = 0.0;
            return;
        }
        auto max_iter = std::max_element(intensities.echoes.begin(),intensities.echoes.end());
        size_t index = std::distance(intensities.echoes.begin(),max_iter);
        if (ranges.echoes.size() > 0) {
            range = ranges.echoes[index];
            intensity = *max_iter;
        } else {
            range = std::numeric_limits<float>::quiet_NaN();
            intensity = 0.0;
            return;
        }
    }


protected:
    ros::NodeHandle nhh;
    ros::Subscriber laserScanSub;
    ros::Publisher laserScanPub01;
    ros::Publisher laserScanPub02;
    ros::Publisher laserScanPub03;
};

int main(int argc,char ** argv) {
    ros::init(argc,argv,"transform");
    cloudHandler handler;
    ros::spin();
    return 0;
}
