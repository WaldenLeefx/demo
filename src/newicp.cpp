//
// Created by lee on 2021/6/8.
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


#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/sparse_optimizer.h>
#include "sophus/se3.hpp"
#include <g2o/core/optimizable_graph.h>


typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> BlockSolverType;
typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

int max_iteration = 100;
double epsilon = 1e-4;
double minDist = 0.5;
double angle_increment = 0.00871451;
struct pointType {
    int id;
    Eigen::Vector2d pt;//points
    Eigen::Vector2d nor;//normal
    double lp;//planarity
    double lc;//curvature
};

struct cloudParams {
    int id;
    std::vector<pointType> cloud;
    Eigen::Vector4d lpStats;//planarity stats
    Eigen::Vector4d lcStats;//curvature stats
    Nabo::NNSearchD* nns = nullptr;//nnSearchD
    Eigen::MatrixXd kdTreeData;
    Eigen::Matrix3d T;//pose
};
cloudParams cloudTgt;
cloudParams cloudTgt000000;
cloudParams cloudSrc;
cloudParams cloudSrc000000;
cloudParams cloudSrcfenshen;
cloudParams cloudSrcfenshen000000;
cloudParams cloudSrc02;

pcl::PointCloud<pcl::PointXYZ> pclt;
pcl::PointCloud<pcl::PointXYZ> pclt02;
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
        std::cout<<"histgram: "<<i<<" "<<His[i]<<std::endl;
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

void statsCharacteristic(cloudParams& cloud) {
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
        cloud.cloud.at(i).nor = normal;
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
    std::cout<<"hisLC"<<std::endl;
    histogram(vecLC,hisLC,0,1,40);


    std::vector<double> hisLP;
    min_maxNormalization(vecLP,-0.1,0.1,0,1);
    std::cout<<"hisLP"<<std::endl;
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
    std::cout<<"LP_mean: "<<LP_mean<<" LP_var: "<<LP_var<<" LP_e: "<<LP_e<<" LP_entropy: "<<LP_entropy<<std::endl;
    std::cout<<"LC_mean: "<<LC_mean<<" LC_var: "<<LC_var<<" LC_e: "<<LC_e<<" LC_entropy: "<<LC_entropy<<std::endl;
}

void setSourcePointCloud(std::vector<pointType>& _source_cloud, cloudParams& source) {
    source.cloud = _source_cloud;
    source.id = _source_cloud.at(0).id;
}

void setTargetPointCloud(std::vector<pointType>& _target_cloud, cloudParams& target) {
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
    std::cout<<"id: "<<target.id<<std::endl;
    statsCharacteristic(target);
    for (int i = 0; i < target.cloud.size(); ++i) {
        target.kdTreeData(2,i) = target.cloud.at(i).nor(0);
        target.kdTreeData(3,i) = target.cloud.at(i).nor(1);
        target.kdTreeData(4,i) = target.cloud.at(i).lp;
        target.kdTreeData(5,i) = target.cloud.at(i).lc;
    }
    target.id = _target_cloud.at(0).id;
}

bool icpAlgorithm2D ( const cloudParams& pts_tgt,
                      const cloudParams& pts_src,
                      Eigen::Matrix3d& T_ts)
{
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();//中间变换
    //Eigen::Matrix3d T_ts = Eigen::Matrix3d::Identity();//最终变换

    Eigen::Vector2d center_tgt = {0,0};
    Eigen::Vector2d center_src = {0,0};

    std::vector<Eigen::Vector2d> closestPts_tgt;
    std::vector<Eigen::Vector2d> closestPts_src;
    std::vector<Eigen::Matrix<double,6,1>> tgt;//x,y,xn,yn,lp,lc
    int k = 20;
    Eigen::VectorXi index(k);
    Eigen::VectorXd squareDist(k);

    Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t_ = {0,0};


    int count = 0;
    for (int n = 0;n < max_iteration; ++n) {
        std::cout<<"n: "<<n<<std::endl;
        tgt.clear();
        closestPts_src.clear();
        cloudParams src_;
        //wr = R_ * wr;
        //wt = wr * wt + t_;
        T_ts = T * T_ts;///
        std::cout<<"T_ts:\n"<<T_ts<<std::endl;
        //std::cout<<"wr:\n"<<wr<<std::endl;
        //std::cout<<"wt:\n"<<wt<<std::endl;
        ///
        size_t size1 = pts_src.cloud.size();
        size_t size2 = src_.cloud.size();
        std::vector<pointType> ps(size1);
        for (size_t i = 0; i < size1; ++i) {
            Eigen::Vector3d p0;
            Eigen::Vector3d p1;
            p0 << pts_src.cloud.at(i).pt(0),
                  pts_src.cloud.at(i).pt(1),
                  1;
            p1 = T * p0;
            //Eigen::Vector2d pp = wr * pts_src.cloud.at(i).pt + wt;
            ps.at(i).pt = {p1(0),p1(1)};
        }
        src_.cloud = ps;

        if (src_.nns != nullptr) {
            delete src_.nns;
            src_.nns = nullptr;
        }
        if (src_.nns == nullptr) {
            src_.kdTreeData.resize(6,src_.cloud.size());
            for (int i = 0; i < src_.cloud.size(); ++i) {
                src_.kdTreeData(0,i) = src_.cloud.at(i).pt(0);
                src_.kdTreeData(1,i) = src_.cloud.at(i).pt(1);
                src_.kdTreeData(2,i) = std::numeric_limits<double>::infinity();
                src_.kdTreeData(3,i) = std::numeric_limits<double>::infinity();
                src_.kdTreeData(4,i) = std::numeric_limits<double>::infinity();
                src_.kdTreeData(5,i) = std::numeric_limits<double>::infinity();
            }
            src_.nns = Nabo::NNSearchD::createKDTreeLinearHeap(src_.kdTreeData,2);
        }
        statsCharacteristic(src_);
        for (int i = 0; i < src_.cloud.size(); ++i) {
            src_.kdTreeData(2,i) = src_.cloud.at(i).nor(0);
            src_.kdTreeData(3,i) = src_.cloud.at(i).nor(1);
            src_.kdTreeData(4,i) = src_.cloud.at(i).lp;
            src_.kdTreeData(5,i) = src_.cloud.at(i).lc;
            //std::cout<<"i:"<<i<<" src_x: "<<src_.cloud.at(i).pt(0)<<" src_y: "<<src_.cloud.at(i).pt(1)<<" lc: "<<src_.cloud.at(i).lc<<std::endl;
        }
        size_t size3 = src_.cloud.size();

        for (int i = 0; i < src_.cloud.size(); ++i) {
            pclt.points.emplace_back(src_.cloud.at(i).pt(0),src_.cloud.at(i).pt(1),0);
        }
        if (n == -1) {
            for (int i = 470; i < pclt.size(); ++i) {
                pclt02.points.emplace_back(pclt.points[i]);
            }
            pclt02.height = 1;
            pclt02.width = 470;
            pcl::io::savePCDFileASCII("/home/lee/catkinReadPCD_ws/sortpts01_01iteration.pcd",pclt02);
        }
        ///




        for (size_t i = 0; i < src_.cloud.size(); ++i) {//for pts_src,find closest points in pts_tgt
            //Eigen::Vector2d pt = wr * src_.cloud.at(i).pt + wt;
            double b = sqrt(pow(src_.cloud.at(i).nor(0),2) + pow(src_.cloud.at(i).nor(1),2));
            if (!std::isinf(src_.cloud.at(i).nor(0)) &&
                !std::isinf(src_.cloud.at(i).lp) &&
                !std::isinf(src_.cloud.at(i).lc)) {
                pts_tgt.nns->knn(src_.cloud.at(i).pt,index,squareDist,k,0,Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::SORT_RESULTS,2);
                for (int j = 0; j < k; ++j) {
                    if (squareDist[j] < std::numeric_limits<double>::infinity() && !std::isinf(squareDist[j])) {
                        tgt.emplace_back(pts_tgt.kdTreeData.col(index[j]));
                    }
                }

                int count1 = 1;
                while (tgt.size() > 1) {
                    double ep1 = 1e-2;
                    double ep2 = 1e-1;
                    ep1 = ep1 / (10 * count);
                    ep2 = ep2 / (10 * count);
                    for (int j = tgt.size() - 1; j >= 0; --j) {
                        if (!std::isinf(tgt[j](2)) &&
                            !std::isinf(tgt[j](4)) &&
                            !std::isinf(tgt[j](5))) {
                            Eigen::Vector2d norTgt;
                            norTgt = {tgt[j](2),tgt[j](3)};
                            double b_ = sqrt(pow(norTgt(0),2) + pow(norTgt(1),2));
                            double a = src_.cloud.at(i).nor.dot(norTgt);
                            double c = a / (b * b_);
                            double d = tgt[j](4) - src_.cloud.at(i).lp;
                            double e = tgt[j](5) - src_.cloud.at(i).lc;
                            if (c < 0.5 || abs(d) > ep1 || abs(e) > ep2) {
                                if (tgt.size() == 1) {break;}
                                tgt.erase(tgt.begin() + j);
                            }
                        } else {
                            tgt.erase(tgt.begin() + j);
                        }
                    }
                    count1++;
                }

            }

            if (tgt.size() > 0) {
                closestPts_src.emplace_back(src_.cloud.at(i).pt);//pt
                closestPts_tgt.emplace_back(tgt[0](0),tgt[0](1));
                tgt.clear();
            }
        }

        size_t cloudSize;
        cloudSize = closestPts_src.size() < closestPts_tgt.size() ? closestPts_src.size() : closestPts_tgt.size();
        for (size_t i = 0; i < cloudSize; i++) {
            center_tgt += closestPts_tgt[i];
            center_src += closestPts_src[i];
        }
        center_tgt = center_tgt / cloudSize;
        center_src = center_src / cloudSize;
        //remove the center
        std::vector<Eigen::Vector2d> removed_tgt(cloudSize);
        std::vector<Eigen::Vector2d> removed_src(cloudSize);
        for (size_t i = 0; i < cloudSize; i++) {
            removed_tgt[i] = closestPts_tgt[i] - center_tgt;
            removed_src[i] = closestPts_src[i] - center_src;
        }
        Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < cloudSize; ++i) {
            W += Eigen::Vector2d(removed_src[i](0),removed_src[i](1)) *
                 Eigen::Vector2d(removed_tgt[i](0),removed_tgt[i](1)).transpose();////
        }
        //svd on W
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix2d& U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        Eigen::Matrix2d r0 = U * (V.transpose());
        Eigen::Vector2d t0 = center_src - r0 * center_tgt;
        T << r0(0,0),r0(0,1),t0(0,0),
             r0(1,0),r0(1,1),t0(1,0),
             0,0,1;
        std::cout<<"T:\n"<<T<<std::endl;
        //R_ = U * (V.transpose());
        //t_ = center_tgt - R_ * center_src;
        //std::cout<<"R_:\n"<<R_<<std::endl;
        //std::cout<<"t_:\n"<<t_<<std::endl;
        if (std::isnan(t_[0]) || std::isinf(t_[0]) ||
            std::isnan(t_[1]) || std::isinf(t_[1])) {
            std::cout<<"iteration:"<<n<<std::endl;
            break;
        }
        ///check if convergence
        size_t size = removed_tgt.size();
        double dist = 0.0;
        for (size_t i = 0; i < size; ++i) {
            Eigen::Vector2d check_src = R_ * closestPts_src.at(i) + t_;
            Eigen::Vector2d check_tgt = closestPts_tgt.at(i) - check_src;
            dist += sqrt(pow(check_tgt[0],2) + pow(check_tgt[1],2));
        }

        count++;

        if (epsilon > dist / double(size) || n == 2) {
            std::cout<<"total iteration: "<<n+1<<std::endl;
            T_ts = T * T_ts;
            break;
        }

    }
    if (count == max_iteration) {
        std::cout<<"Max Iteration!"<<std::endl;
    }
    return true;
}

bool icpAlgorithm2DST ( const cloudParams& pts_src,
                        const cloudParams& pts_tgt,
                        Eigen::Matrix3d& T_st)
{
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    int k = 20;
    Eigen::VectorXi si(k);
    Eigen::VectorXd sd(k);
    std::vector<Eigen::Vector2d> norVec;
    double time = 0;
    cloudParams src;
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
            std::cout<<"id: "<<src.id<<std::endl;
            statsCharacteristic(src);
            for (int i = 15; i < src.cloud.size() - 15; ++i) {
                src.kdTreeData(2,i) = src.cloud.at(i).nor(0);
                src.kdTreeData(3,i) = src.cloud.at(i).nor(1);
                src.kdTreeData(4,i) = src.cloud.at(i).lp;
                src.kdTreeData(5,i) = src.cloud.at(i).lc;
                norVec.emplace_back(src.cloud.at(i).nor(0),src.cloud.at(i).nor(1));
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
            double b = sqrt(pow(src.cloud.at(i).nor(0),2) + pow(src.cloud.at(i).nor(1),2));
            if (!std::isinf(src.cloud.at(i).nor(0)) &&
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
                            double a = src.cloud.at(i).nor.dot(norTgt);
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
        if (n > 0 && sum < 3.00000001) {
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

bool icpAlgorithm2STR(const cloudParams& pts_src,
                         const cloudParams& pts_tgt,
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


bool icpAlgorithm2DOld ( const cloudParams& pts_tgt,
                      const cloudParams& pts_src,
                      Eigen::Matrix2d& wr, Eigen::Vector2d& wt )
{
    Eigen::Vector2d center_tgt = {0,0};
    Eigen::Vector2d center_src = {0,0};

    std::vector<Eigen::Vector2d> closestPts_tgt;
    std::vector<Eigen::Vector2d> closestPts_src;
    std::vector<Eigen::Matrix<double,6,1>> tgt;//x,y,xn,yn,lp,lc
    int k = 15;
    Eigen::VectorXi index(k);
    Eigen::VectorXd squareDist(k);

    Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t_ = {0,0};

    int count = 0;
    for (int n = 0;n < max_iteration; ++n) {
        tgt.clear();
        closestPts_src.clear();
        wr = R_ * wr;
        wt = wr * wt + t_;
        for (size_t i = 0; i < pts_src.cloud.size(); ++i) {//for pts_src,find closest points in pts_tgt
            Eigen::Vector2d pt = wr * pts_src.cloud.at(i).pt + wt;
            double b = sqrt(pow(pts_src.cloud.at(i).nor(0),2) + pow(pts_src.cloud.at(i).nor(1),2));
            if (!std::isinf(pts_src.cloud.at(i).nor(0)) &&
                !std::isinf(pts_src.cloud.at(i).lp) &&
                !std::isinf(pts_src.cloud.at(i).lc)) {
                pts_tgt.nns->knn(pt,index,squareDist,k,0,Nabo::NNSearchD::SORT_RESULTS,1);
                for (int j = 0; j < k; ++j) {
                    if (squareDist[j] < std::numeric_limits<double>::infinity() && !std::isinf(squareDist[j])) {
                        tgt.emplace_back(pts_tgt.kdTreeData.col(index[j]));
                    }
                }

                for (int j = tgt.size() - 1; j > 0; --j) {
                    if (!std::isinf(tgt[j](2)) &&
                        !std::isinf(tgt[j](4)) &&
                        !std::isinf(tgt[j](5)) &&
                        !tgt.empty()) {
                        Eigen::Vector2d norTgt;
                        norTgt = {tgt[j](2),tgt[j](3)};
                        double b_ = sqrt(pow(norTgt(0),2) + pow(norTgt(1),2));
                        double a = pts_src.cloud.at(i).nor.dot(norTgt);
                        double c = a / (b * b_);
                        double d = tgt[j](4) - pts_src.cloud.at(i).lp;
                        double e = tgt[j](5) - pts_src.cloud.at(i).lc;
                        if (c < 0.5 || d > 1e-5 || e > 1e-5) {
                            tgt.erase(tgt.begin() + j);
                        }
                    }
                }

            }
            if (tgt.size() == 1) {
                closestPts_src.emplace_back(pt);
                closestPts_tgt.emplace_back(tgt[0](0),tgt[0](1));
                tgt.clear();
            }

        }

        size_t cloudSize;
        cloudSize = closestPts_src.size() < closestPts_tgt.size() ? closestPts_src.size() : closestPts_tgt.size();
        for (size_t i = 0; i < cloudSize; i++) {
            center_tgt += closestPts_tgt[i];
            center_src += closestPts_src[i];
        }
        center_tgt = center_tgt / cloudSize;
        center_src = center_src / cloudSize;
        //remove the center
        std::vector<Eigen::Vector2d> removed_tgt(cloudSize);
        std::vector<Eigen::Vector2d> removed_src(cloudSize);
        for (size_t i = 0; i < cloudSize; i++) {
            removed_tgt[i] = closestPts_tgt[i] - center_tgt;
            removed_src[i] = closestPts_src[i] - center_src;
        }
        Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < cloudSize; ++i) {
            W += Eigen::Vector2d(removed_tgt[i](0),removed_tgt[i](1)) *
                 Eigen::Vector2d(removed_src[i](0),removed_src[i](1)).transpose();
        }
        //svd on W
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix2d& U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        R_ = U * (V.transpose());
        t_ = center_tgt - R_ * center_src;
        if (std::isnan(t_[0]) || std::isinf(t_[0]) ||
            std::isnan(t_[1]) || std::isinf(t_[1])) {
            std::cout<<"iteration:"<<n<<std::endl;
            break;
        }
        ///check if convergence
        size_t size = removed_tgt.size();
        double dist = 0.0;
        for (size_t i = 0; i < size; ++i) {
            Eigen::Vector2d check_src = R_ * closestPts_src.at(i) + t_;
            Eigen::Vector2d check_tgt = closestPts_tgt.at(i) - check_src;
            dist += sqrt(pow(check_tgt[0],2) + pow(check_tgt[1],2));
        }
        count++;
        if (epsilon > dist / double(size) ) {
            std::cout<<"iteration"<<n<<std::endl;
            break;
        }
    }
    if (count == max_iteration) {
        std::cout<<"Max Iteration!"<<std::endl;
    }
    return true;
}




static Matrix6d JRInv(const Sophus::SE3d &e) {
    Matrix6d J;
    J.block(0,0,3,3) = Sophus::SO3d::hat(e.so3().log());
    J.block(0,3,3,3) = Sophus::SO3d::hat(e.translation());
    J.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
    J.block(3,3,3,3) = Sophus::SO3d::hat(e.so3().log());
    J = J * 0.5 + Matrix6d::Identity();
    return J;
}

class VertexSE3LieAlgebra : public g2o::BaseVertex<6,Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream &is) override {
        double data[7];
        for (double & i : data) {
            is >> i;
            setEstimate(Sophus::SE3d(Eigen::Quaterniond(data[6],data[3],data[4],data[5]),
                                     Eigen::Vector3d(data[0],data[1],data[2])));
        }
    }

    //**setEstimate for vertex, use read
    bool read(Eigen::Matrix3d &T) {
        Eigen::Vector3d v = {0,0,0};
        v << T(0,2),T(1,2),0;
        Eigen::Matrix3d T_;
        T_ << T(0,0),T(0,1),0,
                T(1,0),T(1,1),0,
                0,0,1;
        Eigen::Quaterniond q(T_);
        setEstimate(Sophus::SE3d(q,v));
    }

    bool write(std::ostream &os) const override {
        os << id() << " ";
        Eigen::Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
        return true;
    }

    void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    void oplusImpl(const double *update) override {
        Vector6d upd;
        upd << update[0],update[1],update[2],update[3],update[4],update[5];
        _estimate = Sophus::SE3d::exp(upd) * _estimate;
    }
};

class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6,Sophus::SE3d,VertexSE3LieAlgebra,VertexSE3LieAlgebra> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream &is) override {
        double data[7];
        for (int i = 0; i < 7; ++i) {
            is >> data[i];
        }
        Eigen::Quaterniond q(data[6],data[3],data[4],data[5]);
        q.normalize();
        setMeasurement(Sophus::SE3d(q,Eigen::Vector3d(data[0],data[1],data[2])));
        for (int i = 0; i < information().rows() && is.good(); ++i) {
            for (int j = 0; j < information().cols() && is.good(); ++j) {
                is >> information()(i,j);
                if (i != j)
                    information()(j,i) = information()(i,j);
            }
        }
        return true;
    }

    //**setMeasurement for edge, use read
    bool read(Eigen::Matrix3d &T) {
        Eigen::Vector3d v = {0,0,0};
        v << T(0,2),T(1,2),0;
        Eigen::Matrix3d T_;
        T_ << T(0,0),T(0,1),0,
                T(1,0),T(1,1),0,
                0,0,1;
        Eigen::Quaterniond q(T_);
        setMeasurement(Sophus::SE3d(q,v));
        return true;
    }

    bool write(std::ostream &os) const override {
        auto *v1 = dynamic_cast<VertexSE3LieAlgebra *>(_vertices[0]);
        auto *v2 = dynamic_cast<VertexSE3LieAlgebra *>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        Sophus::SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1]<< " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";
        for (int i = 0; i < information().rows(); ++i) {
            for (int j = 0; j < information().cols(); ++j) {
                os << information()(i,j) << " ";
            }
        }
        os << std::endl;
        return true;
    }

    // 误差计算与书中推导一致
    void computeError() override {
        Sophus::SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        Sophus::SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    void linearizeOplus() override {
        Sophus::SE3d v1 = (static_cast<VertexSE3LieAlgebra *>(_vertices[0]))->estimate();
        Sophus::SE3d v2 = (static_cast<VertexSE3LieAlgebra *>(_vertices[1]))->estimate();
        Matrix6d J = JRInv(Sophus::SE3d::exp(_error));
        _jacobianOplusXi = -J * v2.inverse().Adj();
        _jacobianOplusXj =  J * v2.inverse().Adj();
    }
};

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
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/un01.pcd",*cloudTgt00);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc00(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/un02.pcd",*cloudSrc00);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc0001(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/un03.pcd",*cloudSrc0001);

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



    setTargetPointCloud(cloudTgt.cloud,cloudTgt000000);
    Eigen::Matrix3d T_st = Eigen::Matrix3d::Identity();
    icpAlgorithm2DST(cloudSrc,cloudTgt000000,T_st);///T21

    setTargetPointCloud(cloudSrcfenshen.cloud,cloudSrcfenshen000000);
    Eigen::Matrix3d T_st02 = Eigen::Matrix3d::Identity();
    icpAlgorithm2DST(cloudSrc02,cloudSrcfenshen000000,T_st02);///T32

    cloudParams ct01;
    cloudParams ct02;
    ct01.cloud = cloudTgt000000.cloud;
    setTargetPointCloud(ct01.cloud,ct02);
    Eigen::Matrix3d T_st03 = Eigen::Matrix3d::Identity();///T31
    icpAlgorithm2DST(cloudSrc02,ct02,T_st03);

    std::cout<<"T_3_1:\n"<<T_st02.inverse() * T_st.inverse()<<std::endl;

    ///pose graph
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);
    std::vector<VertexSE3LieAlgebra *> vertices;
    std::vector<EdgeSE3LieAlgebra *> edges;


    Eigen::Matrix3d Ti = Eigen::Matrix3d::Identity();
    //vertex
    auto *vT = new VertexSE3LieAlgebra();
    vT->setId(0);
    vT->setFixed(true);
    vT->read(Ti);//v->setEstimate()
    optimizer.addVertex(vT);
    vertices.push_back(vT);

    auto *v1 = dynamic_cast<VertexSE3LieAlgebra*>(optimizer.vertices()[0]);
    Sophus::SE3d p1 = v1->estimate();
    std::cout<<"p1:\n"<<p1.matrix().inverse()<<std::endl;

    auto *vS = new VertexSE3LieAlgebra();
    vS->setId(1);
    vS->read(T_st02);
    optimizer.addVertex(vS);
    vertices.push_back(vS);

    auto *v2 = dynamic_cast<VertexSE3LieAlgebra*>(optimizer.vertices()[1]);
    Sophus::SE3d p2 = v2->estimate();
    std::cout<<"p2:\n"<<p2.matrix().inverse()<<std::endl;


    std::cout<<"vertex: "<<optimizer.vertices()[0]->id()<<" "<<optimizer.vertices()[1]->id()<<std::endl;
    //edge
    auto *e = new EdgeSE3LieAlgebra();
    e->setId(1);
    e->setVertex(0,optimizer.vertices()[0]);
    e->setVertex(1,optimizer.vertices()[1]);
    e->read(T_st03);//e->setMeasurement
    e->setInformation(Matrix6d::Identity());
    optimizer.addEdge(e);
    edges.push_back(e);

    optimizer.initializeOptimization();
    optimizer.optimize(30);


    for (int i = 0; i < optimizer.vertices().size(); ++i) {
        auto *vv = dynamic_cast<VertexSE3LieAlgebra*>(optimizer.vertices()[i]);
        Sophus::SE3d pose = vv->estimate();
        std::cout<<"pose:\n"<<pose.matrix().inverse()<<std::endl;
    }










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
