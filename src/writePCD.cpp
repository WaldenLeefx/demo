//
// Created by lee on 2021/11/11.
//

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


int main(int argc,char** argv){
    pcl::PointCloud<pcl::PointXYZ> full01;
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl01full.pcd",full01);
    std::ofstream ofstream;

    for (int i = 0; i < full01.points.size(); ++i) {
        double x = full01.points[i].x;
        double y = full01.points[i].y;
        double dist = sqrt(x*x + y*y);
        ofstream.open("/home/lee/catkinReadPCD_ws/plicp01.txt",std::ios::app);
        if (!ofstream.is_open())
            std::cout<<"open file failed."<<std::endl;
        //ofstream<<x<<","<<y<<","<<0.1<<"\n";
        if (i==0) {
            ofstream<<"FLASER"<<" "<<"563"<<" ";
        } else {
            ofstream<<dist<<" ";
        }
        ofstream.close();
    }
    std::cout<<"Finished."<<std::endl;
    return 0;
}