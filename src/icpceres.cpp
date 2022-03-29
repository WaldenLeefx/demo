//
// Created by lee on 2021/7/19.
//
#define FMT_HEADER_ONLY
#include <ros/ros.h>
#include <iostream>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nabo/nabo.h>
#include <boost/timer.hpp>

#include "sophus/se3.hpp"

#include <ceres/ceres.h>

typedef Eigen::Matrix<double,6,1> Vector6d;
int max_iteration = 20;
double epsilon = 1e-4;
double minDist = 0.5;
double angle_increment = 0.00871451;
struct pointType {
    int id;
    Eigen::Vector2d pt;//points
    Eigen::Vector2d tan;//tangent vector
    double lp;//planarity
    double lc;//curvature
};

struct cloudType {
    int id;
    std::vector<pointType> cloud;
    Eigen::Vector4d lpStats;//planarity stats
    Eigen::Vector4d lcStats;//curvature stats
    Nabo::NNSearchD* nns = nullptr;//nnSearchD
    Eigen::MatrixXd kdTreeData;
    Eigen::Matrix3d T;//pose
};
cloudType cloudTgt;
cloudType cloudTgt000000;
cloudType cloudSrc;
cloudType cloudSrc000000;
cloudType cloudSrcfenshen;
cloudType cloudSrcfenshen000000;
cloudType cloudSrc02;

pcl::PointCloud<pcl::PointXYZ> pclt;
pcl::PointCloud<pcl::PointXYZ> pclt02;
using namespace std;
static void histogram(std::vector<double>& data,std::vector<double>& H,double min,double max,int n){
    double interval;
    interval = (max - min) / n;
    std::vector<double> N(n);
    std::vector<double> His(n);
    for (double & i : data) {
        if (!std::isinf(i)) {
            if (i > max) { i = max; }
            if (i < min) { i = min; }
            int i_;
            i_ = floor(i / interval);//round down
            if (i_ == max / interval) { i_ = i_ - 1; }
            N[i_]++;
        }
    }
    for (int i = 0; i < N.size(); ++i) {
        His[i] = N[i] / (N.size() + 1);
        ///std::cout<<"histgram: "<<i<<" "<<His[i]<<std::endl;
    }
    H = His;
}
static void min_maxNormalization(std::vector<double>& a,double oMin,double oMax,double nMin,double nMax) {
    for (double & i : a) {
        if (!std::isinf(i)) {
            i =nMin + (nMax - nMin) * (i - oMin) / (oMax - oMin);
        }
    }
}
static Eigen::Vector2d ComputeNormal(std::vector<Eigen::Vector2d> &nearPoints) {
    Eigen::Vector2d normal;
    int ptCnt = nearPoints.size();
    if (ptCnt < 5) {
        normal(0) = normal(1) = std::numeric_limits<double>::infinity();
        return normal;
    }
    //compute mean
    Eigen::Vector2d mean;
    mean.setZero();
    for (const auto & nearPoint : nearPoints) {
        mean += nearPoint;
    }
    mean = mean / nearPoints.size();
    //compute covariance matrix
    Eigen::Matrix2d covariance;
    covariance.setZero();
    for (auto & nearPoint : nearPoints) {
        Eigen::Vector2d pt = nearPoint - mean;
        covariance(0,0) += pt(0) * pt(0);
        covariance(1,1) += pt(1) * pt(1);
        covariance(0,1) += pt(0) * pt(1);
        covariance(1,0) += pt(1) * pt(0);
    }
    covariance(0,0) /= ptCnt;
    covariance(1,1) /= ptCnt;
    covariance(0,1) /= ptCnt;
    covariance(1,0) /= ptCnt;
    //compute normals
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigenSolver(covariance);
    //eigenValues ascending sort
    Eigen::Vector2d eigenValues = eigenSolver.eigenvalues();
    //the first eigenVector is refer to min-eigenValue
    Eigen::Matrix2d eigenVectors = eigenSolver.eigenvectors();
    normal = eigenVectors.col(1);
    return normal;
}

void computeTangentCurvature(cloudType& cloudIn){
    std::vector<double> vecLC;
    std::vector<double> vecLP;

    for (int i = 0; i < cloudIn.cloud.size(); ++i) {
        if (i < cloudIn.cloud.size() - 1) {
            double angle1 = atan2(cloudIn.cloud.at(i).pt(1),cloudIn.cloud.at(i).pt(0));
            double angel2 = atan2(cloudIn.cloud.at(i+1).pt(1),cloudIn.cloud.at(i+1).pt(0));
            double delta1 = abs(angle1 - angel2);
            double p1 = sqrt(pow(cloudIn.cloud.at(i).pt(0),2) + pow(cloudIn.cloud.at(i).pt(1),2));
            double p2 = sqrt(pow(cloudIn.cloud.at(i+1).pt(0),2) + pow(cloudIn.cloud.at(i+1).pt(1),2));
            double delta2 = abs(p1 - p2);
            if (delta1 < 5 * angle_increment && delta2 < 0.11) {
                Eigen::Vector2d t1 = cloudIn.cloud.at(i+1).pt - cloudIn.cloud.at(i).pt;
                cloudIn.cloud.at(i).tan = t1.normalized();
            } else {
                cloudIn.cloud.at(i).tan(0) = cloudIn.cloud.at(i).tan(1) = std::numeric_limits<double>::infinity();
            }
        } else {
            cloudIn.cloud.at(i).tan(0) = cloudIn.cloud.at(i).tan(1) = std::numeric_limits<double>::infinity();
        }

        if (i == 0 || i == cloudIn.cloud.size() - 1) {
            cloudIn.cloud.at(i).lc = std::numeric_limits<double>::infinity();
            vecLC.push_back(std::numeric_limits<double>::infinity());
        } else if (!std::isinf(cloudIn.cloud.at(i-1).tan(0)) && !std::isinf(cloudIn.cloud.at(i).tan(0))) {
            Eigen::Vector2d t1 = cloudIn.cloud.at(i-1).tan;
            Eigen::Vector2d t2 = cloudIn.cloud.at(i).tan;
            double phi = acos(t1.dot(t2));
            Eigen::Vector2d p1 = (cloudIn.cloud.at(i).pt + cloudIn.cloud.at(i-1).pt) / 2;
            Eigen::Vector2d p2 = (cloudIn.cloud.at(i+1).pt + cloudIn.cloud.at(i).pt) / 2;
            cloudIn.cloud.at(i).lc = phi / (p1.norm() + p2.norm());
            vecLC.push_back(phi / (p1.norm() + p2.norm()));
        } else {
            cloudIn.cloud.at(i).lc = std::numeric_limits<double>::infinity();
            vecLC.push_back(std::numeric_limits<double>::infinity());
        }

        if (i < 5 || i > cloudIn.cloud.size() - 6) {
            cloudIn.cloud.at(i).lp = std::numeric_limits<double>::infinity();
            vecLP.push_back(std::numeric_limits<double>::infinity());
        } else if (!std::isinf(cloudIn.cloud.at(i).tan(0))) {
            Eigen::Vector2d lp = cloudIn.cloud.at(i-5).pt + cloudIn.cloud.at(i-4).pt + cloudIn.cloud.at(i-3).pt +
                                 cloudIn.cloud.at(i-2).pt + cloudIn.cloud.at(i-1).pt - 10 * cloudIn.cloud.at(i).pt +
                                 cloudIn.cloud.at(i+1).pt + cloudIn.cloud.at(i+2).pt + cloudIn.cloud.at(i+3).pt +
                                 cloudIn.cloud.at(i+4).pt + cloudIn.cloud.at(i+5).pt;
            cloudIn.cloud.at(i).lp = cloudIn.cloud.at(i).tan.dot(lp / 10);
            vecLP.push_back(cloudIn.cloud.at(i).tan.dot(lp / 10));
        } else {
            cloudIn.cloud.at(i).lp = std::numeric_limits<double>::infinity();
            vecLP.push_back(std::numeric_limits<double>::infinity());
        }
    }

    for (int i = 0; i < cloudIn.cloud.size(); i++) {
        double dist = 0;
        if (i < cloudIn.cloud.size()-1) {
            dist = sqrt(pow(cloudIn.cloud.at(i).pt(0),2) + pow(cloudIn.cloud.at(i).pt(1),2)) -
                          sqrt(pow(cloudIn.cloud.at(i+1).pt(0),2) + pow(cloudIn.cloud.at(i+1).pt(1),2));
        } else {dist = -1;}

        cout<<i
            <<" tan: "<<cloudIn.cloud.at(i).tan(0)<<" "<<cloudIn.cloud.at(i).tan(1)
            <<" lc: "<<cloudIn.cloud.at(i).lc
            <<" lp: "<<cloudIn.cloud.at(i).lp
            //<<" x: "<<cloudIn.cloud.at(i).pt(0)
            //<<" y: "<<cloudIn.cloud.at(i).pt(1)
            <<" dist: "<<abs(dist)
            <<endl;
        //dist>0.1 abandon
        //lc < 0.1 flat
        //lc > 0.1 sharp
    }


/*
    std::vector<double> hisLC;
    min_maxNormalization(vecLC,0,0.25,0,1);
    histogram(vecLC,hisLC,0,1,40);

    std::vector<double> hisLP;
    min_maxNormalization(vecLP,-6,8,0,1);
    histogram(vecLP,hisLP,0,1,40);

    ///compute mean/variance/energy/entropy
    //for planarity
    double LP_mean = 0,LP_var = 0,LP_e = 0,LP_entropy = 0;
    double lp_m = 0,lp_v = 0;

    //for curvature
    double LC_mean = 0,LC_var = 0,LC_e = 0,LC_entropy = 0;
    double lc_m = 0,lc_v = 0;

    for (int i = 0; i < cloudIn.cloud.size(); ++i) {
        if (!std::isinf(vecLP[i])) {
            lp_m += vecLP[i];
        }
        if (!std::isinf(vecLC[i])) {
            lc_m += vecLC[i];
        }
    }
    LP_mean = lp_m / (cloudIn.cloud.size() + 1);
    LC_mean = lc_m / (cloudIn.cloud.size() + 1);

    for (int i = 0; i < cloudIn.cloud.size(); ++i) {
        if (!std::isinf(vecLP[i])) {
            lp_v += pow(vecLP[i] - LP_mean,2);
        }
        if (!std::isinf(vecLC[i])) {
            lc_v += pow(vecLC[i] - LC_mean,2);
        }
    }
    LP_var = lp_v / (cloudIn.cloud.size() + 1);
    LC_var = lc_v / (cloudIn.cloud.size() + 1);

    for (int i = 0; i < hisLC.size(); ++i) {
        if (!std::isinf(hisLP[i])) {
            LP_e += pow(hisLP[i],2);
            if (hisLP[i] > 0) {
                LP_entropy += hisLP[i] * log2(hisLP[i]);
            }
        }
        if (!std::isinf(hisLC[i])) {
            LC_e += pow(hisLC[i],2);
            if (hisLC[i] > 0) {
                LC_entropy += hisLC[i] * log2(hisLC[i]);
            }
        }
    }

    cloudIn.lpStats << LP_mean,LP_var,LP_e,LP_entropy;
    cloudIn.lcStats << LC_mean,LC_var,LC_e,LC_entropy;
    //std::cout<<"LP_mean: "<<LP_mean<<" LP_var: "<<LP_var<<" LP_e: "<<LP_e<<" LP_entropy: "<<LP_entropy<<std::endl;
    //std::cout<<"LC_mean: "<<LC_mean<<" LC_var: "<<LC_var<<" LC_e: "<<LC_e<<" LC_entropy: "<<LC_entropy<<std::endl;
    */
}

void statsCharacteristic(cloudType& cloud) {
    int searchNum = 15;
    Eigen::VectorXi searchIndex(searchNum);
    Eigen::VectorXd searchDist(searchNum);

    int num = 10;
    Eigen::VectorXi index(num);
    Eigen::VectorXd dist(num);

    std::vector<double> vecLC;
    std::vector<double> vecLP;
    std::vector<double> vecLN;

    for (int i = 0; i < cloud.cloud.size(); ++i) {
        std::vector<Eigen::Vector2d> neighborNormal;//for pts surround point i
        //std::cout<<"i: "<<i<<std::endl;
        cloud.nns->knn(cloud.cloud.at(i).pt,searchIndex,searchDist,searchNum,0,
                       Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::SORT_RESULTS,2);
        std::vector<Eigen::Matrix<double,6,1>> s;//x,y,xn,yn,lp,lc
        std::vector<Eigen::Vector2d> selectedPts;
        for (int j = 0; j < searchNum; ++j) {
            //remove useless points
            if (searchDist[j] < std::numeric_limits<double>::infinity() && !isinf(searchDist(j))) {
                s.emplace_back(cloud.kdTreeData.col(searchIndex[j]));
            }
        }

        double LPlanarity = 0;
        double LCurvature = 0;
        double LNormalDiff = 0;
        if (s.size() == 0) {
            LPlanarity = std::numeric_limits<double>::infinity();
            LCurvature = std::numeric_limits<double>::infinity();
            vecLP.push_back(LPlanarity);
            vecLC.push_back(LCurvature);
            cloud.cloud.at(i).lp = LPlanarity;
            cloud.cloud.at(i).lc = LCurvature;
            continue;
        }
        for (auto & j : s) {
            selectedPts.emplace_back(j(0),j(1));
        }
        //remove points that are not adjacent
        double b = atan2(selectedPts[0](1),selectedPts[0](0));
        for (int j = selectedPts.size() - 1; j > 1; --j) {
            double a = atan2(selectedPts[j](1),selectedPts[j](0));
            if (abs(a - b) > 3*(j-1)*angle_increment) {
                selectedPts.erase(selectedPts.begin() + j);
            }
        }

        ///compute curvature
        std::deque<Eigen::Vector2d> sortedPts;
        sortedPts.push_back(selectedPts[0]);
        double angle0 = atan2(selectedPts[0](1),selectedPts[0](0));
        int count1_ = 0;
        int count2_ = 0;
        int count3_ = 0;
        for (auto & selectedPt : selectedPts) {
            double angle1 = atan2(selectedPt(1),selectedPt(0));
            if (angle1 >= 0) {count1_++;}
            if (angle1 < 0) {count2_++;}
            if (abs(angle1) < M_PI / 4) {count3_++;}
        }

        if (count1_ == selectedPts.size() ||
            count2_ == selectedPts.size() ||
            count3_ == selectedPts.size()) {
            for (int j = 1; j < selectedPts.size(); ++j) {
                double angle1 = atan2(selectedPts[j](1),selectedPts[j](0));
                double angle = angle1 - angle0;
                if (angle > 0) {
                    sortedPts.push_back(selectedPts[j]);
                } else {
                    sortedPts.push_front(selectedPts[j]);
                }
            }
        } else {
            for (int j = 1; j < selectedPts.size(); ++j) {
                double angle1 = atan2(selectedPts[j](1),selectedPts[j](0));
                double angle = angle1 - angle0;
                if (angle < 0 && angle1 > 0) {
                    sortedPts.push_back(selectedPts[j]);
                } else if (angle > 0 && angle1 > 0) {
                    sortedPts.push_front(selectedPts[j]);
                } else if (angle1 < 0) {
                    sortedPts.push_front(selectedPts[j]);
                }
            }
        }

        for (int j = 0; j < sortedPts.size(); ++j) {
            if (cloud.cloud.at(i).pt(0) == sortedPts[j](0)) {
                if (j >= 5 && j < sortedPts.size() - 5) {
                    double diffX = sortedPts[j-5](0) + sortedPts[j-4](0) + sortedPts[j-3](0)
                                   + sortedPts[j-2](0) + sortedPts[j-1](0) - 10 * sortedPts[j](0)
                                   + sortedPts[j+1](0) + sortedPts[j+2](0) + sortedPts[j+3](0)
                                   + sortedPts[j+4](0) + sortedPts[j+5](0);
                    double diffY = sortedPts[j-5](1) + sortedPts[j-4](1) + sortedPts[j-3](1)
                                   + sortedPts[j-2](1) + sortedPts[j-1](1) - 10 * sortedPts[j](1)
                                   + sortedPts[j+1](1) + sortedPts[j+2](1) + sortedPts[j+3](1)
                                   + sortedPts[j+4](1) + sortedPts[j+5](1);
                    LCurvature = diffX * diffX + diffY * diffY;
                    vecLC.push_back(LCurvature);
                    break;
                } else if (j >= 4 && j < sortedPts.size() - 4) {
                    double diffX = sortedPts[j-4](0) + sortedPts[j-3](0)
                                   + sortedPts[j-2](0) + sortedPts[j-1](0) - 8 * sortedPts[j](0)
                                   + sortedPts[j+1](0) + sortedPts[j+2](0) + sortedPts[j+3](0)
                                   + sortedPts[j+4](0);
                    double diffY = sortedPts[j-4](1) + sortedPts[j-3](1)
                                   + sortedPts[j-2](1) + sortedPts[j-1](1) - 8 * sortedPts[j](1)
                                   + sortedPts[j+1](1) + sortedPts[j+2](1) + sortedPts[j+3](1)
                                   + sortedPts[j+4](1);
                    LCurvature = diffX * diffX + diffY * diffY;
                    vecLC.push_back(LCurvature);
                    break;
                } else {
                    LCurvature = std::numeric_limits<double>::infinity();
                    vecLC.push_back(LCurvature);
                    break;
                }
            }
        }

        ///compute normal for every selected points, or assume all the selected points have same normal


        Eigen::Vector2d normal;
        if (selectedPts.size() >= 10) {
            normal = ComputeNormal(selectedPts);
        } else {
            normal(0) = normal(1) = std::numeric_limits<double>::infinity();
        }
        cloud.cloud.at(i).tan = normal;
        //ptsNormal.push_back(normal);

        ///compute normal difference (or assume all the selected points have same normal)
        ///compute planarity

        Eigen::Vector2d centerPt;
        if (selectedPts.size() >= 10 && !std::isinf(normal(0))){
            for (int j = 1; j < selectedPts.size(); ++j) {
                centerPt.setZero();
                centerPt += selectedPts[j] - selectedPts[0];
            }
            LPlanarity = normal.dot(centerPt / (selectedPts.size()-1));
            vecLP.push_back(LPlanarity);
        } else {
            LPlanarity = std::numeric_limits<double>::infinity();
            vecLP.push_back(LPlanarity);
        }
        cloud.cloud.at(i).lp = LPlanarity;
        cloud.cloud.at(i).lc = LCurvature;
        //lp_lc.emplace_back(LPlanarity,LCurvature);
        //std::cout<<"lp: "<<LPlanarity<<" lc: "<<LCurvature<<std::endl;
    }
    ///histogram of planarity,normal difference, curvature

    std::vector<double> hisLC;
    min_maxNormalization(vecLC,0,50,0,1);
    //std::cout<<"hisLC"<<std::endl;
    histogram(vecLC,hisLC,0,1,40);


    std::vector<double> hisLP;
    min_maxNormalization(vecLP,-0.1,0.1,0,1);
    //std::cout<<"hisLP"<<std::endl;
    histogram(vecLP,hisLP,0,1,40);

    std::vector<double> hisLN;
    //min_maxNormalization(vecLN,-0.0001,0.0009,0,1);
    //histogram(vecLN,hisLN,0,1,40);

    ///compute mean/variance/energy/entropy
    //for planarity
    double LP_mean = 0,LP_var = 0,LP_e = 0,LP_entropy = 0;
    double lp_m = 0,lp_v = 0;

    //for curvature
    double LC_mean = 0,LC_var = 0,LC_e = 0,LC_entropy = 0;
    double lc_m = 0,lc_v = 0;

    for (int i = 0; i < cloud.cloud.size(); ++i) {
        if (!std::isinf(vecLP[i])) {
            lp_m += vecLP[i];
        }
        if (!std::isinf(vecLC[i])) {
            lc_m += vecLC[i];
        }
    }
    LP_mean = lp_m / (cloud.cloud.size() + 1);
    LC_mean = lc_m / (cloud.cloud.size() + 1);

    for (int i = 0; i < cloud.cloud.size(); ++i) {
        if (!std::isinf(vecLP[i])) {
            lp_v += pow(vecLP[i] - LP_mean,2);
        }
        if (!std::isinf(vecLC[i])) {
            lc_v += pow(vecLC[i] - LC_mean,2);
        }
    }
    LP_var = lp_v / (cloud.cloud.size() + 1);
    LC_var = lc_v / (cloud.cloud.size() + 1);

    for (int i = 0; i < hisLC.size(); ++i) {
        if (!std::isinf(hisLP[i])) {
            LP_e += pow(hisLP[i],2);
            if (hisLP[i] > 0) {
                LP_entropy += hisLP[i] * log2(hisLP[i]);
            }
        }
        if (!std::isinf(hisLC[i])) {
            LC_e += pow(hisLC[i],2);
            if (hisLC[i] > 0) {
                LC_entropy += hisLC[i] * log2(hisLC[i]);
            }
        }
    }

    cloud.lpStats << LP_mean,LP_var,LP_e,LP_entropy;
    cloud.lcStats << LC_mean,LC_var,LC_e,LC_entropy;
    //std::cout<<"LP_mean: "<<LP_mean<<" LP_var: "<<LP_var<<" LP_e: "<<LP_e<<" LP_entropy: "<<LP_entropy<<std::endl;
    //std::cout<<"LC_mean: "<<LC_mean<<" LC_var: "<<LC_var<<" LC_e: "<<LC_e<<" LC_entropy: "<<LC_entropy<<std::endl;
}

void setSourcePointCloud(std::vector<pointType>& _source_cloud, cloudType& source) {
    source.cloud = _source_cloud;
    source.id = _source_cloud.at(0).id;
}

void setTargetPointCloud(std::vector<pointType>& _target_cloud, cloudType& target) {
    target.cloud = _target_cloud;
    if (target.nns != nullptr) {
        delete target.nns;
        target.nns = nullptr;
    }
    //construct kd tree
    if (target.nns == nullptr) {
        target.kdTreeData.resize(6,target.cloud.size());
        for (int i = 0; i < target.cloud.size(); ++i) {
            target.kdTreeData(0,i) = target.cloud.at(i).pt(0);
            target.kdTreeData(1,i) = target.cloud.at(i).pt(1);
            target.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
            target.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
            target.kdTreeData(4,i) = std::numeric_limits<double>::infinity();
            target.kdTreeData(5,i) = std::numeric_limits<double>::infinity();
        }
        target.nns = Nabo::NNSearchD::createKDTreeLinearHeap(target.kdTreeData,2);
    }
    ///statsCharacteristic(target);
    computeTangentCurvature(target);
    for (int i = 0; i < target.cloud.size(); ++i) {
        target.kdTreeData(2,i) = target.cloud.at(i).tan(0);
        target.kdTreeData(3,i) = target.cloud.at(i).tan(1);
        target.kdTreeData(4,i) = target.cloud.at(i).lp;
        target.kdTreeData(5,i) = target.cloud.at(i).lc;
    }
    target.id = _target_cloud.at(0).id;
}


bool icpAlgorithm2DST ( const cloudType& pts_src,
                        const cloudType& pts_tgt,
                        Eigen::Matrix3d& T_st)
{
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    int k = 20;
    Eigen::VectorXi si(k);
    Eigen::VectorXd sd(k);
    std::vector<Eigen::Vector2d> norVec;
    double time = 0;
    cloudType src;
    for (int n = 0; n < max_iteration; ++n) {
        //std::cout<<"n: "<<n<<std::endl;
        std::vector<Eigen::Vector2d> match_src;
        std::vector<Eigen::Vector2d> match_tgt;
        T_st = T * T_st;
        //std::cout<<"T_st:\n"<<T_st<<std::endl;
        //std::cout<<"T_st_inv:\n"<<T_st.inverse()<<std::endl;
        size_t s1 = pts_src.cloud.size();
        std::vector<pointType> p3(s1);
        for (int i = 0; i < s1; ++i) {
            Eigen::Vector3d p0;
            p0 << pts_src.cloud.at(i).pt(0),
                    pts_src.cloud.at(i).pt(1),
                    1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p3.at(i).pt = {p1[0],p1[1]};
        }

        src.cloud = p3;
        src.kdTreeData.resize(6,src.cloud.size());
        boost::timer timer01;
        if (n < 2) {
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(0,i) = src.cloud.at(i).pt(0);
                src.kdTreeData(1,i) = src.cloud.at(i).pt(1);
                src.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(4,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(5,i) = std::numeric_limits<double>::infinity();
            }
            src.nns = Nabo::NNSearchD::createKDTreeLinearHeap(src.kdTreeData,2);
            //std::cout<<"id: "<<src.id<<std::endl;
            statsCharacteristic(src);
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(2,i) = src.cloud.at(i).tan(0);
                src.kdTreeData(3,i) = src.cloud.at(i).tan(1);
                src.kdTreeData(4,i) = src.cloud.at(i).lp;
                src.kdTreeData(5,i) = src.cloud.at(i).lc;
                norVec.emplace_back(src.cloud.at(i).tan(0),src.cloud.at(i).tan(1));
            }
        } else {
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                if (!std::isinf(norVec[i](0))) {
                    Eigen::Vector2d nor_;
                    Eigen::Matrix2d r_;
                    r_ << T_st(0,0),T_st(0,1),
                            T_st(1,0),T_st(1,1);
                    nor_ = r_ * norVec[i];
                    src.kdTreeData(2,i) = nor_(0);
                    src.kdTreeData(3,i) = nor_(1);
                } else {
                    src.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                    src.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                }
            }
        }
        time += timer01.elapsed();
        //std::cout<<"time1: "<<time<<"(s)"<<std::endl;
        if (n > 0) {
            for (auto & i : src.cloud) {
                pclt.points.emplace_back(i.pt(0),i.pt(1),0);
            }
            if (n == -1) {
                for (int i = 0; i < pclt.size(); ++i) {
                    pclt02.points.emplace_back(pclt.points[i]);
                }
                pclt02.height = 1;
                pclt02.width = 470;
                pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/sortpts01_01iteration.pcd",pclt02);
            }
        }

        std::vector<Eigen::Matrix<double,6,1>> tgt;//x,y,xn,yn,lp,lc
        for (int i = 15; i < src.cloud.size() - 15; ++i) {
            double b = sqrt(pow(src.cloud.at(i).tan(0),2) + pow(src.cloud.at(i).tan(1),2));
            if (!std::isinf(src.cloud.at(i).tan(0)) &&
                !std::isinf(src.cloud.at(i).lp) &&
                !std::isinf(src.cloud.at(i).lc)) {
                pts_tgt.nns->knn(src.cloud.at(i).pt, si, sd, k, 0,
                                 Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::SORT_RESULTS, 2);
                for (int j = 0; j < k; ++j) {
                    if (sd[j] < std::numeric_limits<double>::infinity() && !std::isinf(sd[j])) {
                        tgt.emplace_back(pts_tgt.kdTreeData.col(si[j]));
                    }
                }

                int count = 1;
                while (tgt.size() > 1) {
                    double ep1 = 1e-2;
                    double ep2 = 1e-1;
                    ep1 = ep1 / pow(10,count);
                    ep2 = ep2 / pow(10,count);
                    for (int j = tgt.size() - 1; j >= 0; --j) {
                        if (!std::isinf(tgt[j](2)) &&
                            !std::isinf(tgt[j](4)) &&
                            !std::isinf(tgt[j](5))) {
                            Eigen::Vector2d norTgt;
                            norTgt = {tgt[j](2), tgt[j](3)};
                            double b_ = sqrt(pow(norTgt(0), 2) + pow(norTgt(1), 2));
                            double a = src.cloud.at(i).tan.dot(norTgt);
                            double c = a / (b * b_);
                            double d = tgt[j](4) - src.cloud.at(i).lp;
                            double e = tgt[j](5) - src.cloud.at(i).lc;
                            if (c < 0.5 || abs(d) > ep1 || abs(e) > ep2) {
                                if (tgt.size() == 1) { break; }
                                tgt.erase(tgt.begin() + j);
                            }
                        } else {
                            tgt.erase(tgt.begin() + j);
                        }
                    }
                    count++;
                }
            }
            if (!tgt.empty()) {
                match_src.emplace_back(src.cloud.at(i).pt);//pt
                match_tgt.emplace_back(tgt[0](0),tgt[0](1));
                tgt.clear();
            }
        }

        //求质心
        Eigen::Vector2d cTgt;
        Eigen::Vector2d cSrc;
        size_t sSrc = match_src.size();
        for (int i = 0; i < sSrc; ++i) {
            cTgt += match_tgt[i];
            cSrc += match_src[i];
        }
        cTgt = cTgt / sSrc;
        cSrc = cSrc / sSrc;
        //求去质心后的点
        std::vector<Eigen::Vector2d> rTgt(sSrc);
        std::vector<Eigen::Vector2d> rSrc(sSrc);
        for (int i = 0; i < sSrc; ++i) {
            rTgt[i] = match_tgt[i] - cTgt;
            rSrc[i] = match_src[i] - cSrc;
        }
        //svd
        Eigen::Matrix2d w = Eigen::Matrix2d::Zero();
        for (int i = 0; i < sSrc; ++i) {
            w += Eigen::Vector2d(rTgt[i](0),rTgt[i](1)) * Eigen::Vector2d(rSrc[i](0),rSrc[i](1)).transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(w,Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d u = svd.matrixU();
        Eigen::Matrix2d v = svd.matrixV();
        Eigen::Matrix2d r = u * (v.transpose());
        Eigen::Vector2d t = cTgt - r * cSrc;
        T << r(0,0),r(0,1),t(0,0),
                r(1,0),r(1,1),t(1,0),
                0,0,1;
        double sum;
        sum = abs(T(0,0)) + abs(T(0,1)) + abs(T(0,2)) +
              abs(T(1,0)) + abs(T(1,1)) + abs(T(1,2)) +
              abs(T(2,0)) + abs(T(2,1)) + abs(T(2,2));
        if (n > 0 && sum < 3.00000001 || n == max_iteration - 1) {
            T_st = T * T_st;
            std::cout<<"\033[32m"<<"Final Result!"<<"\033[0m"<<std::endl;
            std::cout<<"T:\n"<<T<<std::endl;
            std::cout<<"T_inv:\n"<<T.inverse()<<std::endl;
            std::cout<<"T_st:\n"<<std::setiosflags(std::ios::fixed)<<std::setprecision(10)<<T_st<<std::endl;
            std::cout<<"T_st_inv:\n"<<std::setiosflags(std::ios::fixed)<<std::setprecision(10)<<T_st.inverse()<<std::endl;
            std::cout<<"time: "<<time<<std::endl;
            if (n == max_iteration) {
                std::cout<<"iteration"<<n<<std::endl;
            }
            break;
        }
        //std::cout<<"T:\n"<<T<<std::endl;
        //std::cout<<"T_inv:\n"<<T.inverse()<<std::endl;
        //std::cout<<"T_st:\n"<<T * T_st<<std::endl;
        //std::cout<<"T_st_inv:\n"<<(T * T_st).inverse()<<std::endl;
    }
    return true;
}

bool icpAlgorithm2STR(const cloudType& pts_src,
                      const cloudType& pts_tgt,
                      Eigen::Matrix3d& T_st) {
    std::vector<Eigen::Vector3d> norTgt;
    std::vector<Eigen::Vector3d> norSrc;
    double cost = 0;
    double lastCost = 0;
    Sophus::SO3d R;
    for (int i = 0; i < 10; ++i) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();

        cost = 0;
        for (int j = 0; j < norSrc.size(); ++j) {
            Eigen::Vector3d _nor = R * norSrc[j];
            Eigen::Vector3d err = norTgt[i] - _nor;
            Eigen::Matrix3d J = -Sophus::SO3d::hat(_nor);
            cost += err.squaredNorm();
            H += J * J.transpose();
            b += -J * err;
        }
        Eigen::Vector3d dx;
        dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            std::cout<<"result is nan!"<<std::endl;
            break;
        }
        if (i > 0 && cost >= lastCost) {
            std::cout<<"cost: "<<cost<<", last cost: "<<lastCost<<std::endl;
            break;
        }
        R = Sophus::SO3d::exp(dx) * R;
        lastCost = cost;

        std::cout<<"iteration: "<<i<<" cost="<<std::cout.precision(12)<<cost<<std::endl;
        if (dx.norm() < 1e-6) {break;}
    }
    std::cout<<"pose by GN:\n"<<R.matrix()<<std::endl;
}

class ICPCeres : public ceres::SizedCostFunction<3,6> {
public:
    ICPCeres(Eigen::Vector3d &tgt,Eigen::Vector3d &src) : _tgt(tgt), _src(src) {};
    virtual ~ICPCeres(){}
    bool Evaluate(double const* const* param, double *residuals, double **jacobi) const override {
        Eigen::Map<const Eigen::Matrix<double,6,1>> se3(param[0]);
        Sophus::SE3d T = Sophus::SE3d::exp(se3);
        auto src_ = T * _src;
        Eigen::Vector3d res = _tgt - src_;
        residuals[0] = res[0];
        residuals[1] = res[1];
        residuals[2] = res[2];

        Eigen::Matrix<double,3,6> jac = Eigen::Matrix<double,3,6>::Zero();
        jac.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
        jac.block<3,3>(0,3) = Sophus::SO3d::hat(src_);
        Eigen::Matrix<double,6,3> jac_transpose = jac.transpose();
        if (jacobi != nullptr && jacobi[0] != nullptr) {
            for (int i = 0; i < 18; ++i) {
                jacobi[0][i] = jac_transpose(i);
            }
        }
        return true;
    };

private:
    Eigen::Vector3d _tgt;
    Eigen::Vector3d _src;
};



class ICPCeresTan : public ceres::SizedCostFunction<3,3> {
public:
    ICPCeresTan(Eigen::Vector3d &tgtTan,Eigen::Vector3d &srcTan) : _tgtTan(tgtTan), _srcTan(srcTan) {};
    virtual ~ICPCeresTan() {}
    bool Evaluate(double const* const* param,double *residuals, double **jacobi) const override{
        Eigen::Map<const Eigen::Vector3d> so3(param[0]);
        Sophus::SO3d R = Sophus::SO3d::exp(so3);
        auto srcNor_ = R * _srcTan;
        Eigen::Vector3d res = _tgtTan - srcNor_;
        residuals[0] = res[0];
        residuals[1] = res[1];
        residuals[2] = res[2];

        Eigen::Matrix3d jac = Sophus::SO3d::hat(srcNor_);
        Eigen::Matrix3d jac_transpose = jac.transpose();
        if (jacobi != nullptr && jacobi[0] != nullptr) {
            for (int i = 0; i < 9; ++i) {
                jacobi[0][i] = jac_transpose(i);
            }
        }
        return true;
    };

private:
    Eigen::Vector3d _tgtTan;
    Eigen::Vector3d _srcTan;
};

static Vector6d Mat33toVec61(Eigen::Matrix3d &T) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R << T(0,0),T(0,1),0,
         T(1,0),T(1,1),0,
         0,0,1;
    Eigen::Vector3d t = {0,0,0};
    t << T(0,2),T(1,2),0;
    Sophus::SE3d SE3_Rt(R,t);
    Vector6d se3 = SE3_Rt.log();
    return se3;
}

bool icpAlgorithm2DSTCeres ( const cloudType& pts_src,
                          const cloudType& pts_tgt,
                          Eigen::Matrix3d& T_st)
{
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    int k = 20;
    Eigen::VectorXi si(k);
    Eigen::VectorXd sd(k);
    std::vector<Eigen::Vector2d> norVec;
    double time = 0;
    cloudType src;
    Sophus::SE3d se3D;
    double cost = 0;
    double lastCost = 0;
    for (int n = 0; n < max_iteration; ++n) {
        //std::cout<<"n: "<<n<<std::endl;
        std::vector<Eigen::Vector2d> match_src;
        std::vector<Eigen::Vector2d> match_tgt;
        std::vector<Eigen::Vector3d> match_src3d;
        std::vector<Eigen::Vector3d> match_tgt3d;
        T_st = T * T_st;
        //std::cout<<"T_st:\n"<<T_st<<std::endl;
        //std::cout<<"T_st_inv:\n"<<T_st.inverse()<<std::endl;
        size_t s1 = pts_src.cloud.size();
        std::vector<pointType> p3(s1);
        for (int i = 0; i < s1; ++i) {
            Eigen::Vector3d p0;
            p0 << pts_src.cloud.at(i).pt(0),
                    pts_src.cloud.at(i).pt(1),
                    1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p3.at(i).pt = {p1[0],p1[1]};
        }

        src.cloud = p3;
        src.kdTreeData.resize(6,src.cloud.size());
        boost::timer timer01;
        if (n < 1) {
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(0,i) = src.cloud.at(i).pt(0);
                src.kdTreeData(1,i) = src.cloud.at(i).pt(1);
                src.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(4,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(5,i) = std::numeric_limits<double>::infinity();
            }
            src.nns = Nabo::NNSearchD::createKDTreeLinearHeap(src.kdTreeData,2);
            ///std::cout<<"id: "<<src.id<<std::endl;
            ///statsCharacteristic(src);
            computeTangentCurvature(src);
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(2,i) = src.cloud.at(i).tan(0);
                src.kdTreeData(3,i) = src.cloud.at(i).tan(1);
                src.kdTreeData(4,i) = src.cloud.at(i).lp;
                src.kdTreeData(5,i) = src.cloud.at(i).lc;
                norVec.emplace_back(src.cloud.at(i).tan(0),src.cloud.at(i).tan(1));
            }
        } else {
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                if (!std::isinf(norVec[i](0))) {
                    Eigen::Vector2d nor_;
                    Eigen::Matrix2d r_;
                    r_ << T_st(0,0),T_st(0,1),
                            T_st(1,0),T_st(1,1);
                    nor_ = r_ * norVec[i];
                    src.kdTreeData(2,i) = nor_(0);
                    src.kdTreeData(3,i) = nor_(1);
                } else {
                    src.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                    src.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                }
            }
        }
        time += timer01.elapsed();
        //std::cout<<"time1: "<<time<<"(s)"<<std::endl;
        if (n > 0) {
            for (auto & i : src.cloud) {
                pclt.points.emplace_back(i.pt(0),i.pt(1),0);
            }
            if (n == -1) {
                for (int i = 0; i < pclt.size(); ++i) {
                    pclt02.points.emplace_back(pclt.points[i]);
                }
                pclt02.height = 1;
                pclt02.width = 470;
                pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/sortpts01_01iteration.pcd",pclt02);
            }
        }

        std::vector<Eigen::Matrix<double,6,1>> tgt;//x,y,xn,yn,lp,lc
        for (int i = 15; i < src.cloud.size() - 15; ++i) {
            double b = sqrt(pow(src.cloud.at(i).tan(0),2) + pow(src.cloud.at(i).tan(1),2));
            if (!std::isinf(src.cloud.at(i).tan(0)) &&
                !std::isinf(src.cloud.at(i).lp) &&
                !std::isinf(src.cloud.at(i).lc)) {
                pts_tgt.nns->knn(src.cloud.at(i).pt, si, sd, k, 0,
                                 Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::SORT_RESULTS, 2);
                for (int j = 0; j < k; ++j) {
                    if (sd[j] < std::numeric_limits<double>::infinity() && !std::isinf(sd[j])) {
                        tgt.emplace_back(pts_tgt.kdTreeData.col(si[j]));
                    }
                }

                int count = 1;
                while (tgt.size() > 1) {
                    double ep1 = 1e-2;
                    double ep2 = 1e-1;
                    ep1 = ep1 / pow(10,count);
                    ep2 = ep2 / pow(10,count);
                    for (int j = tgt.size() - 1; j >= 0; --j) {
                        if (!std::isinf(tgt[j](2)) &&
                            !std::isinf(tgt[j](4)) &&
                            !std::isinf(tgt[j](5))) {
                            Eigen::Vector2d norTgt;
                            norTgt = {tgt[j](2), tgt[j](3)};
                            double b_ = sqrt(pow(norTgt(0), 2) + pow(norTgt(1), 2));
                            double a = src.cloud.at(i).tan.dot(norTgt);
                            double c = a / (b * b_);
                            double d = tgt[j](4) - src.cloud.at(i).lp;
                            double e = tgt[j](5) - src.cloud.at(i).lc;
                            if (c < 0.5 || abs(d) > ep1 || abs(e) > ep2) {
                                if (tgt.size() == 1) { break; }
                                tgt.erase(tgt.begin() + j);
                            }
                        } else {
                            tgt.erase(tgt.begin() + j);
                        }
                    }
                    count++;
                }
            }
            if (!tgt.empty()) {
                match_src.emplace_back(src.cloud.at(i).pt);//pt
                match_tgt.emplace_back(tgt[0](0),tgt[0](1));
                match_src3d.emplace_back(src.cloud.at(i).pt[0],src.cloud.at(i).pt[1],0);
                match_tgt3d.emplace_back(tgt[0](0),tgt[0](1),0);
                tgt.clear();
            }
        }

        Vector6d se3 = Mat33toVec61(T);
        double se[6] = {se3[0],se3[1],se3[2],se3[3],se3[4],se3[5]};
        ceres::Problem problem;
        size_t N = match_tgt3d.size();
        for (int i = 0; i < N; ++i) {
            ceres::CostFunction *costFunction = new ICPCeres(match_tgt3d[i],match_src3d[i]);
            problem.AddResidualBlock(costFunction, nullptr,se);
        }
        ceres::Solver::Options options;
        std::cout<<"\033[31m"<<n<<"\033[0m"<<std::endl;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);//start to optimization
        std::cout<<summary.BriefReport()<<std::endl;
        Eigen::Map<Eigen::Matrix<double,6,1>> selie(se);
        Eigen::Matrix4d T_mid = Sophus::SE3d::exp(selie).matrix();
        std::cout<<"estimate =\n"<<T_mid<<std::endl;
        T << T_mid(0,0),T_mid(0,1),T_mid(0,3),
             T_mid(1,0),T_mid(1,1),T_mid(1,3),
             0,0,1;
        double sum;
        sum = abs(T(0,0)) + abs(T(0,1)) + abs(T(0,2)) +
              abs(T(1,0)) + abs(T(1,1)) + abs(T(1,2)) +
              abs(T(2,0)) + abs(T(2,1)) + abs(T(2,2));
        if (n > 0 && sum < 3.000001 || n == max_iteration - 1) {
            T_st = T * T_st;
            std::cout<<"\033[32m"<<"Final Result!"<<"\033[0m"<<std::endl;
            std::cout<<"T:\n"<<T<<std::endl;
            std::cout<<"T_inv:\n"<<T.inverse()<<std::endl;
            std::cout<<"T_st:\n"<<std::setiosflags(std::ios::fixed)<<std::setprecision(10)<<T_st<<std::endl;
            std::cout<<"T_st_inv:\n"<<std::setiosflags(std::ios::fixed)<<std::setprecision(10)<<T_st.inverse()<<std::endl;
            //std::cout<<"time: "<<time<<std::endl;
            if (n == max_iteration) {
                std::cout<<"iteration"<<n<<std::endl;
            }
            break;
        }
    }
    return true;
}

bool icpAlgorithm2DSTCeresTan ( const cloudType& pts_src,
                             const cloudType& pts_tgt,
                             Eigen::Matrix3d& T_st)
{
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    int k = 20;
    Eigen::VectorXi si(k);
    Eigen::VectorXd sd(k);
    std::vector<Eigen::Vector2d> norVec;
    double time = 0;
    cloudType src;
    double cost = 0;
    double lastCost = 0;
    for (int n = 0; n < max_iteration; ++n) {
        //std::cout<<"n: "<<n<<std::endl;
        std::vector<Eigen::Vector2d> match_src;
        std::vector<Eigen::Vector2d> match_tgt;
        std::vector<Eigen::Vector3d> match_src3d;
        std::vector<Eigen::Vector3d> match_tgt3d;
        std::vector<Eigen::Vector3d> match_NSrc3d;
        std::vector<Eigen::Vector3d> match_NTgt3d;
        T_st = T * T_st;
        //std::cout<<"T_st:\n"<<T_st<<std::endl;
        //std::cout<<"T_st_inv:\n"<<T_st.inverse()<<std::endl;
        size_t s1 = pts_src.cloud.size();
        std::vector<pointType> p3(s1);
        for (int i = 0; i < s1; ++i) {
            Eigen::Vector3d p0;
            p0 << pts_src.cloud.at(i).pt(0),
                    pts_src.cloud.at(i).pt(1),
                    1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p3.at(i).pt = {p1[0],p1[1]};
        }

        src.cloud = p3;
        src.kdTreeData.resize(6,src.cloud.size());
        boost::timer timer01;
        if (n < 1) {
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(0,i) = src.cloud.at(i).pt(0);
                src.kdTreeData(1,i) = src.cloud.at(i).pt(1);
                src.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(4,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(5,i) = std::numeric_limits<double>::infinity();
            }
            src.nns = Nabo::NNSearchD::createKDTreeLinearHeap(src.kdTreeData,2);
            ///std::cout<<"id: "<<src.id<<std::endl;
            ///statsCharacteristic(src);
            computeTangentCurvature(src);
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(2,i) = src.cloud.at(i).tan(0);
                src.kdTreeData(3,i) = src.cloud.at(i).tan(1);
                ///std::cout<<"src_tv1: "<<src.kdTreeData(2,i)<<" "<<src.kdTreeData(3,i)<<std::endl;
                src.kdTreeData(4,i) = src.cloud.at(i).lp;
                src.kdTreeData(5,i) = src.cloud.at(i).lc;
                norVec.emplace_back(src.cloud.at(i).tan(0),src.cloud.at(i).tan(1));
            }
        } else {
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                if (!std::isinf(norVec[i](0))) {
                    Eigen::Vector2d nor_;
                    Eigen::Matrix2d r_;
                    r_ << T_st(0,0),T_st(0,1),
                            T_st(1,0),T_st(1,1);
                    nor_ = r_ * norVec[i];
                    src.kdTreeData(2,i) = nor_(0);
                    src.kdTreeData(3,i) = nor_(1);
                    ///std::cout<<"src_tv2: "<<src.kdTreeData(2,i)<<" "<<src.kdTreeData(3,i)<<std::endl;
                } else {
                    src.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                    src.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                    ///std::cout<<"src_tv3: "<<src.kdTreeData(2,i)<<" "<<src.kdTreeData(3,i)<<std::endl;
                }
            }
        }
        time += timer01.elapsed();
        //std::cout<<"time1: "<<time<<"(s)"<<std::endl;
        if (n > 0) {
            for (auto & i : src.cloud) {
                pclt.points.emplace_back(i.pt(0),i.pt(1),0);
            }
            if (n == -1) {
                for (int i = 0; i < pclt.size(); ++i) {
                    pclt02.points.emplace_back(pclt.points[i]);
                }
                pclt02.height = 1;
                pclt02.width = 470;
                pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/sortpts01_01iteration.pcd",pclt02);
            }
        }

        std::vector<Eigen::Matrix<double,6,1>> tgt;//x,y,xn,yn,lp,lc
        for (int i = 15; i < src.cloud.size() - 15; ++i) {
            double b = sqrt(pow(src.cloud.at(i).tan(0),2) + pow(src.cloud.at(i).tan(1),2));
            if (!std::isinf(src.kdTreeData(2,i)) && src.kdTreeData(2,i) != 0 &&
                !std::isinf(src.cloud.at(i).lp) &&
                !std::isinf(src.cloud.at(i).lc)) {
                pts_tgt.nns->knn(src.cloud.at(i).pt, si, sd, k, 0,
                                 Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::SORT_RESULTS, 2);
                for (int j = 0; j < k; ++j) {
                    if (sd[j] < std::numeric_limits<double>::infinity() && !std::isinf(sd[j])) {
                        tgt.emplace_back(pts_tgt.kdTreeData.col(si[j]));
                    }
                }

                int count = 1;
                while (tgt.size() > 1) {
                    double ep1 = 1e-2;
                    double ep2 = 1e-1;
                    ep1 = ep1 / pow(10,count);
                    ep2 = ep2 / pow(10,count);
                    for (int j = tgt.size() - 1; j >= 0; --j) {
                        if (!std::isinf(tgt[j](2)) &&
                            !std::isinf(tgt[j](4)) &&
                            !std::isinf(tgt[j](5))) {
                            Eigen::Vector2d norTgt;
                            norTgt = {tgt[j](2), tgt[j](3)};
                            double b_ = sqrt(pow(norTgt(0), 2) + pow(norTgt(1), 2));
                            double a = src.cloud.at(i).tan.dot(norTgt);
                            double c = a / (b * b_);
                            double d = tgt[j](4) - src.cloud.at(i).lp;
                            double e = tgt[j](5) - src.cloud.at(i).lc;
                            if (c < 0.5 || abs(d) > ep1 || abs(e) > ep2) {
                                if (tgt.size() == 1) { break; }
                                tgt.erase(tgt.begin() + j);
                            }
                        } else {
                            tgt.erase(tgt.begin() + j);
                        }
                    }
                    count++;
                }
            }
            if (!tgt.empty()) {
                match_src.emplace_back(src.cloud.at(i).pt);//pt
                match_tgt.emplace_back(tgt[0](0),tgt[0](1));
                match_src3d.emplace_back(src.cloud.at(i).pt[0],src.cloud.at(i).pt[1],0);
                match_tgt3d.emplace_back(tgt[0](0),tgt[0](1),0);
                match_NSrc3d.emplace_back(src.kdTreeData(2,i),src.kdTreeData(3,i),0);
                match_NTgt3d.emplace_back(tgt[0](2),tgt[0](3),0);
                ///std::cout<<"src_tv: "<<src.kdTreeData(2,i)<<" "<<src.kdTreeData(3,i)<<std::endl;
                ///std::cout<<"tgt_tv: "<<tgt[0](2)<<" "<<tgt[0](2)<<std::endl;
                tgt.clear();
            }
        }


        Vector6d se3 = Mat33toVec61(T);
        double so[3] = {se3[3],se3[4],se3[5]};///exp(Phi^) = R
        ceres::Problem problem;
        ceres::LossFunction *lossFunction;
        lossFunction = new ceres::CauchyLoss(0.01);
        size_t N = match_NTgt3d.size();
        for (int i = 0; i < N; ++i) {
            ceres::CostFunction *costFunction = new ICPCeresTan(match_NTgt3d[i],match_NSrc3d[i]);
            problem.AddResidualBlock(costFunction, lossFunction,so);
        }
        ceres::Solver::Options options;
        std::cout<<"\033[31m"<<n<<"\033[0m"<<std::endl;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);//start to optimization
        std::cout<<summary.BriefReport()<<std::endl;
        Eigen::Map<Eigen::Vector3d> solie(so);
        Eigen::Matrix3d T_mid = Sophus::SO3d::exp(solie).matrix();
        std::cout<<"estimate =\n"<<T_mid<<std::endl;
        T << T_mid(0,0),T_mid(0,1),0,
                T_mid(1,0),T_mid(1,1),0,
                0,0,1;
        double sum;
        sum = abs(T(0,0)) + abs(T(0,1)) + abs(T(0,2)) +
              abs(T(1,0)) + abs(T(1,1)) + abs(T(1,2)) +
              abs(T(2,0)) + abs(T(2,1)) + abs(T(2,2));
        if (n > 0 && sum < 3.0001 || n == max_iteration - 1) {
            T_st = T * T_st;
            std::cout<<"\033[32m"<<"Final Result!"<<"\033[0m"<<std::endl;
            std::cout<<"T:\n"<<T<<std::endl;
            std::cout<<"T_inv:\n"<<T.inverse()<<std::endl;
            std::cout<<"T_st:\n"<<std::setiosflags(std::ios::fixed)<<std::setprecision(10)<<T_st<<std::endl;
            std::cout<<"T_st_inv:\n"<<std::setiosflags(std::ios::fixed)<<std::setprecision(10)<<T_st.inverse()<<std::endl;
            ///std::cout<<"time: "<<time<<std::endl;
            if (n == max_iteration) {
                std::cout<<"iteration"<<n<<std::endl;
            }
            break;
        }
    }
    return true;
}


bool icpTan2DST ( const cloudType& pts_src,
                             const cloudType& pts_tgt,
                             Eigen::Matrix3d& T_st)
{
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    int k = 20;
    Eigen::VectorXi si(k);
    Eigen::VectorXd sd(k);
    std::vector<Eigen::Vector2d> tanVec;
    cloudType src;
    size_t minSize = pts_src.cloud.size() < pts_tgt.cloud.size() ? pts_src.cloud.size() : pts_tgt.cloud.size();
    for (int n = 0; n < max_iteration; ++n) {
        std::vector<Eigen::Vector2d> match_src;
        std::vector<Eigen::Vector2d> match_tgt;
        std::vector<Eigen::Vector3d> match_src3d;
        std::vector<Eigen::Vector3d> match_tgt3d;
        std::vector<Eigen::Vector3d> match_TSrc3d;
        std::vector<Eigen::Vector3d> match_TTgt3d;
        T_st = T * T_st;
        size_t s1 = pts_src.cloud.size();
        std::vector<pointType> p3(s1);
        for (int i = 0; i < s1; ++i) {
            Eigen::Vector3d p0;
            p0 << pts_src.cloud.at(i).pt(0),
                    pts_src.cloud.at(i).pt(1),
                    1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p3.at(i).pt = {p1[0],p1[1]};
        }

        src.cloud = p3;
        src.kdTreeData.resize(6,src.cloud.size());
        boost::timer timer01;
        if (n < 1) {
/*
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(0,i) = src.cloud.at(i).pt(0);
                src.kdTreeData(1,i) = src.cloud.at(i).pt(1);
                src.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(4,i) = std::numeric_limits<double>::infinity();
                src.kdTreeData(5,i) = std::numeric_limits<double>::infinity();
            }
            */
            ///src.nns = Nabo::NNSearchD::createKDTreeLinearHeap(src.kdTreeData,2);
            ///std::cout<<"id: "<<src.id<<std::endl;
            ///statsCharacteristic(src);
            boost::timer timer00;
            computeTangentCurvature(src);
            cout<<"time00: "<<timer00.elapsed()<<endl;
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(2,i) = src.cloud.at(i).tan(0);
                src.kdTreeData(3,i) = src.cloud.at(i).tan(1);
                src.kdTreeData(4,i) = src.cloud.at(i).lp;
                src.kdTreeData(5,i) = src.cloud.at(i).lc;
                tanVec.emplace_back(src.cloud.at(i).tan(0),src.cloud.at(i).tan(1));
            }
        } else {
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                if (!std::isinf(tanVec[i](0))) {
                    Eigen::Vector2d tan_;
                    Eigen::Matrix2d r_;
                    r_ << T_st(0,0),T_st(0,1),
                            T_st(1,0),T_st(1,1);
                    tan_ = r_ * tanVec[i];
                    src.kdTreeData(2,i) = tan_(0);
                    src.kdTreeData(3,i) = tan_(1);
                } else {
                    src.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                    src.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                }
            }
        }

        if (n == 0) {
            cout<<"time: "<<timer01.elapsed()<<endl;
        }
        if (n > 0) {
            for (auto & i : src.cloud) {
                pclt.points.emplace_back(i.pt(0),i.pt(1),0);
            }
            if (n == -1) {
                for (int i = 0; i < pclt.size(); ++i) {
                    pclt02.points.emplace_back(pclt.points[i]);
                }
                pclt02.height = 1;
                pclt02.width = 470;
                pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/sortpts01_01iteration.pcd",pclt02);
            }
        }

        boost::timer timer;
        std::vector<Eigen::Matrix<double,6,1>> tgt;//x,y,xt,yt,lp,lc
        for (int i = 15; i < minSize - 15; ++i) {
            double b = sqrt(pow(src.cloud.at(i).tan(0),2) + pow(src.cloud.at(i).tan(1),2));
            if (!std::isinf(src.kdTreeData(2,i)) && src.kdTreeData(2,i) != 0 &&
                !std::isinf(src.cloud.at(i).lp) &&
                !std::isinf(src.cloud.at(i).lc)) {
                pts_tgt.nns->knn(src.cloud.at(i).pt, si, sd, k, 0,0,2);////
                for (int j = 0; j < k; ++j) {
                    if (sd[j] < std::numeric_limits<double>::infinity() && !std::isinf(sd[j])) {
                        tgt.emplace_back(pts_tgt.kdTreeData.col(si[j]));
                    }
                }

                int cc = 1;
                while (tgt.size() > 1) {
                    double ep1 = 1e-2;
                    double ep2 = 1e-1;
                    ep1 = ep1 / pow(10,cc);
                    ep2 = ep2 / pow(10,cc);
                    for (int j = tgt.size() - 1; j >= 0; --j) {
                        if (!std::isinf(tgt[j](2)) &&
                            !std::isinf(tgt[j](4)) &&
                            !std::isinf(tgt[j](5))) {
                            Eigen::Vector2d tanTgt;
                            tanTgt = {tgt[j](2), tgt[j](3)};
                            double b_ = sqrt(pow(tanTgt(0), 2) + pow(tanTgt(1), 2));
                            double a = src.cloud.at(i).tan.dot(tanTgt);
                            double c = a / (b * b_);
                            double d = tgt[j](4) - src.cloud.at(i).lp;
                            double e = tgt[j](5) - src.cloud.at(i).lc;
                            if (abs(c) < 0.9 || abs(d) > ep1 || abs(e) > ep2) {
                                if (tgt.size() == 1 && n > 5) { break; }
                                tgt.erase(tgt.begin() + j);
                            }
                        } else {
                            tgt.erase(tgt.begin() + j);
                        }
                    }
                    cc++;
                }
            }
            if (!tgt.empty()) {
                match_src.emplace_back(src.cloud.at(i).pt);//pt
                match_tgt.emplace_back(tgt[0](0),tgt[0](1));
                match_src3d.emplace_back(src.cloud.at(i).pt[0],src.cloud.at(i).pt[1],0);
                match_tgt3d.emplace_back(tgt[0](0),tgt[0](1),0);
                match_TSrc3d.emplace_back(src.kdTreeData(2,i),src.kdTreeData(3,i),0);
                match_TTgt3d.emplace_back(tgt[0](2),tgt[0](3),0);
                tgt.clear();
            }
        }
        cout<<"match size: "<<match_tgt3d.size()<<endl;
        if (n == 0){cout<<"time_matchpts: "<<timer.elapsed()<<endl;}

        Vector6d se3 = Mat33toVec61(T);
        double se[6] = {se3[0],se3[1],se3[2],se3[3],se3[4],se3[5]};///exp(Cauchy^) = T
        double so[3] = {se3[3],se3[4],se3[5]};///exp(Phi^) = R
        ceres::LossFunction *lossFunction;
        lossFunction = new ceres::CauchyLoss(0.01);

        boost::timer timer1;
        ceres::Problem problem;
        size_t N = match_tgt3d.size();
        for (int i = 0; i < N; ++i) {
            ceres::CostFunction *costFunctionPt = new ICPCeres(match_tgt3d[i],match_src3d[i]);
            problem.AddResidualBlock(costFunctionPt, nullptr,se);
            ceres::CostFunction *costFunctionTan = new ICPCeresTan(match_TTgt3d[i],match_TSrc3d[i]);
            problem.AddResidualBlock(costFunctionTan,lossFunction,so);
        }
        ceres::Solver::Options options;
        std::cout<<"\033[31m"<<n<<"\033[0m"<<std::endl;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = 4;
        options.max_solver_time_in_seconds = 0.08;
        options.num_threads = 4;
        //options.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);//start to optimization
        std::cout<<summary.BriefReport()<<std::endl;
        Eigen::Map<Eigen::Matrix<double,6,1>> selie(se);
        Eigen::Matrix4d T_m1 = Sophus::SE3d::exp(selie).matrix();
        cout<<"opti-time: "<<timer1.elapsed()<<endl;
        std::cout<<"estimateT =\n"<<T_m1<<std::endl;

        Eigen::Map<Eigen::Vector3d> solie(so);
        Eigen::Matrix3d T_m2 = Sophus::SO3d::exp(solie).matrix();
        std::cout<<"estimateR =\n"<<T_m2<<std::endl;
        Eigen::Matrix4d T_m2m = Eigen::Matrix4d::Identity();
        T_m2m << T_m2(0,0),T_m2(0,1),T_m2(0,2),0,
                 T_m2(1,0),T_m2(1,1),T_m2(1,2),0,
                 T_m2(2,0),T_m2(2,1),T_m2(2,2),0,
                 0,0,0,1;

        Eigen::Matrix4d T_m = T_m1 * T_m2m;

        if (n < 1) {
            T << T_m(0,0),T_m(0,1),T_m(0,3),
                    T_m(1,0),T_m(1,1),T_m(1,3),
                    0,0,1;
        } else {
            T << T_m1(0,0),T_m1(0,1),T_m1(0,3),
                    T_m1(1,0),T_m1(1,1),T_m1(1,3),
                    0,0,1;
        }

        double a = abs(T_m1(0,3));
        double b = abs(T_m1(1,3));
        double sum = 0;
        sum = abs(T(0,0)) + abs(T(0,1)) + abs(T(0,2)) +
              abs(T(1,0)) + abs(T(1,1)) + abs(T(1,2)) +
              abs(T(2,0)) + abs(T(2,1)) + abs(T(2,2)) - 3;
        if ((n > 0 && sum < 0.0001) || (a < 0.0002 && b < 0.0002) || n == max_iteration-1) {
            T_st = T * T_st;
            std::cout<<"\033[32m"<<"Final Result!"<<"\033[0m"<<std::endl;
            std::cout<<"T:\n"<<T<<std::endl;
            std::cout<<"T_inv:\n"<<T.inverse()<<std::endl;
            std::cout<<"T_st:\n"<<std::setiosflags(std::ios::fixed)<<std::setprecision(10)<<T_st<<std::endl;
            std::cout<<"T_st_inv:\n"<<std::setiosflags(std::ios::fixed)<<std::setprecision(10)<<T_st.inverse()<<std::endl;
            //std::cout<<"time: "<<time<<"(s)"<<std::endl;
            if (n == max_iteration-1) {
                std::cout<<"iteration "<<n<<std::endl;
            }
            break;
        }
    }
    return true;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"newicp");

    ros::NodeHandle nh;
    ros::Publisher readPub01 = nh.advertise<sensor_msgs::PointCloud2>("pclOutTgt",10);
    ros::Publisher readPub02 = nh.advertise<sensor_msgs::PointCloud2>("pclOutSrc",10);
    ros::Publisher readPub0301 = nh.advertise<sensor_msgs::PointCloud2>("pclOutMid01",10);
    ros::Publisher readPub0302 = nh.advertise<sensor_msgs::PointCloud2>("pclOutMid02",10);
    ros::Publisher readPub0201 = nh.advertise<sensor_msgs::PointCloud2>("pclOutSrc02",10);

    sensor_msgs::PointCloud2 outputTgt;
    sensor_msgs::PointCloud2 outputSrc;
    sensor_msgs::PointCloud2 outputMid01;
    sensor_msgs::PointCloud2 outputMid02;
    sensor_msgs::PointCloud2 outputSrc02;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTgt00(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl01.pcd",*cloudTgt00);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc00(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl02.pcd",*cloudSrc00);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc0001(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl03.pcd",*cloudSrc0001);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTgt01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc0102(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<Eigen::Vector3d> cloud_tgt;
    std::vector<Eigen::Vector3d> cloud_src;
    std::vector<Eigen::Vector3d> cloud_src02;

    std::vector<Eigen::Vector2d> tgtPts;
    std::vector<Eigen::Vector2d> srcPts;


    int n = cloudTgt00->size();
    std::vector<pointType> p(n);
    double a = 0;
    Eigen::Matrix3d T00;
    T00 << cos(a),-sin(a),0,
            sin(a), cos(a),0,
            0,0,1;
    for (int i = 0; i < cloudTgt00->size(); ++i) {
        float x = cloudTgt00->points[i].x;
        float y = cloudTgt00->points[i].y;
        float z = cloudTgt00->points[i].z;
        Eigen::Vector3d v = {x,y,1};
        v = T00 * v;
        cloud_tgt.emplace_back(v[0],v[1],v[2]);
        tgtPts.emplace_back(v[0],v[1]);
        p.at(i).pt = {v[0],v[1]};
        cloudTgt01->points.emplace_back(v[0],v[1],v[2]);
    }
    cloudTgt.cloud = p;

    int nn = cloudSrc00->size();
    std::vector<pointType> pp(nn);
    double b = 0;
    Eigen::Matrix3d T01;
    T01 << cos(b),-sin(b),0,
            sin(b), cos(b),0,
            0,0,1;
    for (int i = 0; i < cloudSrc00->size(); ++i) {
        float x = cloudSrc00->points[i].x;
        float y = cloudSrc00->points[i].y;
        float z = cloudSrc00->points[i].z;
        Eigen::Vector3d v = {x,y,1};
        v = T01 * v;
        pp.at(i).pt = {v[0],v[1]};
        cloud_src.emplace_back(v);
        srcPts.emplace_back(v[0],v[1]);
        cloudSrc01->points.emplace_back(v[0],v[1],v[2]);
    }
    cloudSrc.cloud = pp;
    cloudSrcfenshen.cloud = pp;

    //int nnn = cloudSrc01->size();
    int nnn = cloudSrc0001->size();
    std::vector<pointType> ppp(nnn);
    double c = 0;
    Eigen::Matrix3d T02;
    T02 << cos(c),-sin(c),0,
            sin(c), cos(c),0,
            0,0,1;
    for (int i = 0; i < cloudSrc0001->size(); ++i) {
        float x = cloudSrc0001->points[i].x;
        float y = cloudSrc0001->points[i].y;
        float z = cloudSrc0001->points[i].z;
        Eigen::Vector3d v = {x,y,1};
        v = T02 * v;
        ppp.at(i).pt = {v[0],v[1]};
        cloudSrc0102->points.emplace_back(v[0],v[1],v[2]);
    }
    cloudSrc02.cloud = ppp;

    //computeTangentCurvature(cloudSrc);

    ///setTargetPointCloud(cloudTgt.cloud,cloudTgt000000);
    Eigen::Matrix3d T_st = Eigen::Matrix3d::Identity();
    ///icpAlgorithm2DST(cloudSrc,cloudTgt000000,T_st);///T21

    ///ceres-icp
    boost::timer timer;
    Eigen::Matrix3d T_st_ceres = Eigen::Matrix3d::Identity();
    ///icpAlgorithm2DSTCeres(cloudSrc,cloudTgt000000,T_st_ceres);
    cout<<"time_cost: "<<timer.elapsed()<<"(s)"<<endl;

    ///ceres-icp-tangent-vector
    boost::timer timer1;
    Eigen::Matrix3d T_st_ceres_tv = Eigen::Matrix3d::Identity();
    ///icpAlgorithm2DSTCeresTan(cloudSrc,cloudTgt000000,T_st_ceres_tv);
    cout<<"time_cost1: "<<timer1.elapsed()<<"(s)"<<endl;

    ///ceres-icp-tan
    boost::timer timer2;
    Eigen::Matrix3d T_st_icp_tan = Eigen::Matrix3d::Identity();
    ///icpTan2DST(cloudSrc,cloudTgt000000,T_st_icp_tan);
    cout<<"time_cost2: "<<timer2.elapsed()<<"(s)"<<endl;

    //setTargetPointCloud(cloudSrcfenshen.cloud,cloudSrcfenshen000000);
    Eigen::Matrix3d T_st02 = Eigen::Matrix3d::Identity();
    ///icpAlgorithm2DST(cloudSrc02,cloudSrcfenshen000000,T_st02);///T32

    cloudType ct01;
    cloudType ct02;
    ct01.cloud = cloudTgt000000.cloud;
    //setTargetPointCloud(ct01.cloud,ct02);
    Eigen::Matrix3d T_st03 = Eigen::Matrix3d::Identity();///T31
    ///icpAlgorithm2DST(cloudSrc02,ct02,T_st03);

    ///std::cout<<"T_3_1:\n"<<T_st02.inverse() * T_st.inverse()<<std::endl;

















    pcl::toROSMsg(*cloudTgt01,outputTgt);//tgt,red
    pcl::toROSMsg(*cloudSrc01,outputSrc);//src,green
    pcl::toROSMsg(pclt,outputMid01);//mid,white
    pcl::toROSMsg(*cloudSrc0102,outputSrc02);
    outputTgt.header.frame_id = "map";
    outputSrc.header.frame_id = "map";
    outputMid01.header.frame_id = "map";
    outputSrc02.header.frame_id = "map";
    ros::Rate loop_rate(1);
    while (ros::ok()){
        readPub01.publish(outputTgt);//tgt,red
        readPub02.publish(outputSrc);//src,green
        readPub0301.publish(outputMid01);//middle,white
        readPub0201.publish(outputSrc02);//src02,green;
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
