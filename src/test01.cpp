//
// Created by lee on 2021/7/31.
//
#define FMT_HEADER_ONLY
#include <ros/ros.h>
#include <iostream>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/registration/icp.h>
#include <pcl-1.8/pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <nabo/nabo.h>
#include <boost/timer.hpp>
//#include <csm/csm_all.h>
#include "sophus/se3.hpp"

#include <ceres/ceres.h>
using namespace std;
//using namespace CSM;
typedef Eigen::Matrix<double,6,1> Vector6d;
int max_iteration = 20;
double epsilon = 1e-10;
double minDist = 0.5;
double angle_increment = 0.00871451;

pcl::PointCloud<pcl::PointXYZ> show01;
pcl::PointCloud<pcl::PointXYZ> show02;

pcl::PointCloud<pcl::PointXYZ> tangentSharp;
pcl::PointCloud<pcl::PointXYZ> tangentFlat;
pcl::PointCloud<pcl::PointXYZ> tangentSharp2;
pcl::PointCloud<pcl::PointXYZ> tangentFlat2;


struct pointType {
    int seq;//id = seq
    ros::Time stamp;//.hear.stamp,
    Eigen::Vector2d pt;//points
    Eigen::Vector2d tan;//tangent vector
    double lp;//planarity
    double lc;//curvature
};

struct cloudType {
    int id;//seq
    ros::Time st;//st = stamp
    std::vector<pointType> cloud;
    Eigen::Vector4d lpStats;//planarity stats
    Eigen::Vector4d lcStats;//curvature stats
    Nabo::NNSearchD* nns = nullptr;//nnSearchD
    Eigen::MatrixXd kdTreeData;
    Eigen::Matrix3d T;//pose
    std::vector<double> hisLP;
    std::vector<double> hisLC;
};

void convertMat2cloudType(const std::vector<Eigen::Vector3d> &vec, cloudType &cloud) {
    size_t s = vec.size();
    cloud.cloud.resize(s);
    for (int i = 0; i < s; ++i) {
        cloud.cloud.at(i).pt = {vec[i](0),vec[i](1)};
    }
}

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

static void computeTangentCurvature(cloudType& cloudIn) {
    std::vector<double> vecLC;
    std::vector<double> vecLP;

    for (int i = 0; i < cloudIn.cloud.size(); ++i) {
        if (i == cloudIn.cloud.size() - 1) {
            cloudIn.cloud.at(i).tan(0) = cloudIn.cloud.at(i).tan(1) = std::numeric_limits<double>::infinity();
        } else {
            Eigen::Vector2d t1 = cloudIn.cloud.at(i+1).pt - cloudIn.cloud.at(i).pt;
            cloudIn.cloud.at(i).tan = t1.normalized();
        }

        if (i == 0 || i == cloudIn.cloud.size() - 1) {
            cloudIn.cloud.at(i).lc = std::numeric_limits<double>::infinity();
            vecLC.push_back(std::numeric_limits<double>::infinity());
        } else {
            Eigen::Vector2d t1 = cloudIn.cloud.at(i-1).tan;
            Eigen::Vector2d t2 = cloudIn.cloud.at(i).tan;
            double phi = acos(t1.dot(t2));
            Eigen::Vector2d p1 = (cloudIn.cloud.at(i).pt - cloudIn.cloud.at(i-1).pt) / 2;
            Eigen::Vector2d p2 = (cloudIn.cloud.at(i+1).pt - cloudIn.cloud.at(i).pt) / 2;
            cloudIn.cloud.at(i).lc = phi / (p1.norm() + p2.norm());
            vecLC.push_back(phi / (p1.norm() + p2.norm()));
        }

        if (i < 5 || i > cloudIn.cloud.size() - 6) {
            cloudIn.cloud.at(i).lp = std::numeric_limits<double>::infinity();
            vecLP.push_back(std::numeric_limits<double>::infinity());
        } else {
            Eigen::Vector2d lp = cloudIn.cloud.at(i-5).pt + cloudIn.cloud.at(i-4).pt + cloudIn.cloud.at(i-3).pt +
                                 cloudIn.cloud.at(i-2).pt + cloudIn.cloud.at(i-1).pt - 10 * cloudIn.cloud.at(i).pt +
                                 cloudIn.cloud.at(i+1).pt + cloudIn.cloud.at(i+2).pt + cloudIn.cloud.at(i+3).pt +
                                 cloudIn.cloud.at(i+4).pt + cloudIn.cloud.at(i+5).pt;
            cloudIn.cloud.at(i).lp = cloudIn.cloud.at(i).tan.dot(lp / 10);
            vecLP.push_back(cloudIn.cloud.at(i).tan.dot(lp / 10));
        }
    }
}

void setSourcePointCloud(std::vector<pointType>& _source_cloud, cloudType& source)
{
    source.cloud = _source_cloud;
    source.id = _source_cloud.at(0).seq;
    source.st = _source_cloud.at(0).stamp;
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
    target.id = _target_cloud.at(0).seq;
    target.st = _target_cloud.at(0).stamp;
}

void pcl2cloudType(pcl::PointCloud<pcl::PointXYZ>::Ptr &p, cloudType &t) {
    t.cloud.resize(p->points.size());
    for (int i = 0; i < p->points.size(); ++i) {
        t.cloud.at(i).pt = {p->points[i].x,p->points[i].y};
    }
}

class ICPCeres : public ceres::SizedCostFunction<3,6> {
public:
    ICPCeres(Eigen::Vector3d &tgt,Eigen::Vector3d &src) : _tgt(tgt), _src(src) {};
    virtual ~ICPCeres(){}
    bool Evaluate(double const* const* param, double *residuals, double **jacobi) const {
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


bool icp(cloudType &nowSharp,cloudType &nowFlat,cloudType &lastLessSharp,cloudType &lastLessFlat,Eigen::Matrix3d &T_st) {
    int cornerSharpNum;
    int surfFlatNum;
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    cloudType _nowCornerSharp, _nowSurfFlat;
    boost::timer timer;
    std::vector<Eigen::Vector3d> matchCornerSrc2, matchCornerTgt2;
    std::vector<Eigen::Vector3d> matchSurfSrc2, matchSurfTgt2;
    std::vector<Eigen::Vector3d> matchTanCornerSrc2, matchTanCornerTgt2;
    std::vector<Eigen::Vector3d> matchTanSurfSrc2, matchTanSurfTgt2;
    for (int i = 0; i < max_iteration; ++i) {
        //nowCornerSharp->cornerSharp->matchCornerSrc,m_cornerLessSharp->m_cornerLessSharpTgt->cloudCornerLessSharpTgt->matchCornerTgt
        std::vector<Eigen::Vector3d> matchCornerSrc, matchCornerTgt;
        //nowSurfFlat->matchSurfFlat,cloudSurfLessFlatTgt->matchSurfTgt
        std::vector<Eigen::Vector3d> matchSurfSrc, matchSurfTgt;
        //matchCornerSrc->matchTanCornerSrc,matchCornerTgt->matchTanCornerTgt
        std::vector<Eigen::Vector3d> matchTanCornerSrc, matchTanCornerTgt;
        //matchSurfSrc->matchTanSurfSrc,matchSurfTgt->matchTanSurfTgt
        std::vector<Eigen::Vector3d> matchTanSurfSrc, matchTanSurfTgt;
        std::cout<<"\033[31m"<<"iter: "<<i<<"\033[0m"<<std::endl;
        T_st = T * T_st;

        std::vector<pointType> p3(nowSharp.cloud.size());
        for (int j = 0; j < nowSharp.cloud.size(); ++j) {
            Eigen::Vector3d p0;
            p0 << nowSharp.cloud.at(j).pt(0),
                    nowSharp.cloud.at(j).pt(1),
                    1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p3.at(j).pt = {p1[0],p1[1]};
        }
        _nowCornerSharp.cloud = p3;

        std::vector<pointType> p4(nowFlat.cloud.size());
        for (int j = 0; j < nowFlat.cloud.size(); ++j) {
            Eigen::Vector3d p0;
            p0 << nowFlat.cloud.at(j).pt(0),
                    nowFlat.cloud.at(j).pt(1),
                    1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p4.at(j).pt = {p1[0],p1[1]};
        }
        _nowSurfFlat.cloud = p4;

        cornerSharpNum = _nowCornerSharp.cloud.size();
        surfFlatNum = _nowSurfFlat.cloud.size();

        for (auto & j : _nowCornerSharp.cloud) {
            show01.points.emplace_back(j.pt(0),j.pt(1),0);
        }
        for (auto & j : _nowSurfFlat.cloud) {
            show02.points.emplace_back(j.pt(0),j.pt(1),0);
        }


/*
            for (int j = 0; j < matchTanCornerSrc.size(); ++j) {
                Eigen::Matrix2d r_;
                r_ << T_st(0,0),T_st(0,1),
                      T_st(1,0),T_st(1,1);
                Eigen::Vector2d t0;
                t0 << matchTanCornerSrc[j](0),matchTanCornerSrc[j](1);
                Eigen::Vector2d t1;
                t1 = r_ * t0;
                _nowCornerSharp.kdTreeData(2,j) = t1(0);
                _nowCornerSharp.kdTreeData(3,j) = t1(1);
            }

            for (int j = 0; j < matchTanSurfSrc.size(); ++j) {
                Eigen::Matrix2d r_;
                r_ << T_st(0,0),T_st(0,1),
                      T_st(1,0),T_st(1,1);
                Eigen::Vector2d t0;
                t0 << matchTanSurfSrc[j](0),matchTanSurfSrc[1](0);
                Eigen::Vector2d t1;
                t1 = r_ * t0;
                _nowSurfFlat.kdTreeData(2,j) = t1(0);
                _nowSurfFlat.kdTreeData(3,j) = t1(1);
            }
             */



        int k = 1;
        Eigen::VectorXi ind(k);
        Eigen::VectorXd dist(k);
        Eigen::VectorXi index(k);
        Eigen::VectorXd distance(k);
            //deal with sharp points, red; less sharp, yellow
            for (int j = 0; j < cornerSharpNum; ++j) {
                Eigen::Matrix<double, 6, 1> tgt;
                // search closest pts for sharpPoints from lessSharpPoints
                lastLessSharp.nns->knn(_nowCornerSharp.cloud.at(j).pt, ind, dist, k, 0, 0, 0.5);
                if (dist[0] < std::numeric_limits<double>::infinity() && !std::isinf(dist[0])) {
                    tgt = lastLessSharp.kdTreeData.col(ind(0));
                    matchCornerSrc.emplace_back(_nowCornerSharp.cloud.at(j).pt(0), _nowCornerSharp.cloud.at(j).pt(1),
                                                0);
                    matchCornerTgt.emplace_back(tgt(0), tgt(1), 0);
                }
            }
            cout << endl;
            //deal with flat points, blue
            for (int j = 0; j < surfFlatNum; ++j) {
                Eigen::Matrix<double, 6, 1> tgt;
                lastLessFlat.nns->knn(_nowSurfFlat.cloud.at(j).pt, index, distance, k, 0, 0, 0.25);
                if (distance[0] < std::numeric_limits<double>::infinity() && !std::isinf(distance[0])) {
                    tgt = lastLessFlat.kdTreeData.col(index(0));
                    matchSurfSrc.emplace_back(_nowSurfFlat.cloud.at(j).pt(0), _nowSurfFlat.cloud.at(j).pt(1), 0);
                    matchSurfTgt.emplace_back(tgt(0), tgt(1), 0);
                }
            }

            //deal with tangent vector
            cloudType cts, ctt, ctSurfS, ctSurfT;
            convertMat2cloudType(matchCornerSrc, cts);
            convertMat2cloudType(matchCornerTgt, ctt);
            computeTangentCurvature(cts);
            computeTangentCurvature(ctt);
            size_t s1 = cts.cloud.size();
            for (int j = 0; j < s1 - 1; ++j) {
                double d1 = sqrt(pow(cts.cloud.at(j).pt(0), 2) + pow(cts.cloud.at(j).pt(1), 2));
                double d2 = sqrt(pow(ctt.cloud.at(j).pt(0), 2) + pow(ctt.cloud.at(j).pt(1), 2));
                double d3 = sqrt(pow(cts.cloud.at(j + 1).pt(0), 2) + pow(cts.cloud.at(j + 1).pt(1), 2));
                double d4 = sqrt(pow(ctt.cloud.at(j + 1).pt(0), 2) + pow(ctt.cloud.at(j + 1).pt(1), 2));
                if (abs(d1 - d2) < 0.1 && abs(d3 - d4) < 0.1) {
                    matchTanCornerSrc.emplace_back(cts.cloud.at(j).tan(0), cts.cloud.at(j).tan(1), 0);
                    matchTanCornerTgt.emplace_back(ctt.cloud.at(j).tan(0), ctt.cloud.at(j).tan(1), 0);
                }
            }

            convertMat2cloudType(matchSurfSrc, ctSurfS);
            convertMat2cloudType(matchSurfTgt, ctSurfT);
            computeTangentCurvature(ctSurfS);
            computeTangentCurvature(ctSurfT);
            size_t s2 = ctSurfS.cloud.size();
            for (int j = 0; j < s2 - 1; ++j) {
                double d1 = sqrt(pow(ctSurfS.cloud.at(j).pt(0), 2) + pow(ctSurfS.cloud.at(j).pt(1), 2));
                double d2 = sqrt(pow(ctSurfT.cloud.at(j).pt(0), 2) + pow(ctSurfT.cloud.at(j).pt(1), 2));
                double d3 = sqrt(pow(ctSurfS.cloud.at(j + 1).pt(0), 2) + pow(ctSurfS.cloud.at(j + 1).pt(1), 2));
                double d4 = sqrt(pow(ctSurfT.cloud.at(j + 1).pt(0), 2) + pow(ctSurfT.cloud.at(j + 1).pt(1), 2));
                double c1 = ctSurfS.cloud.at(j).lc - ctSurfT.cloud.at(j).lc;
                double c2 = ctSurfS.cloud.at(j + 1).lc - ctSurfT.cloud.at(j + 1).lc;
                double p1 = ctSurfS.cloud.at(j).lp - ctSurfT.cloud.at(j).lp;
                double p2 = ctSurfS.cloud.at(j + 1).lp - ctSurfT.cloud.at(j + 1).lp;
                if (abs(d1 - d2) < 0.05 && abs(d3 - d4) < 0.05 &&
                    abs(c1) < 0.05 && abs(c2) < 0.05 &&
                    abs(p1) < 0.05 && abs(p2) < 0.05) {
                    matchTanSurfSrc.emplace_back(ctSurfS.cloud.at(j).tan(0), ctSurfS.cloud.at(j).tan(1), 0);
                    matchTanSurfTgt.emplace_back(ctSurfT.cloud.at(j).tan(0), ctSurfT.cloud.at(j).tan(1), 0);
                }
            }

        ///solve by svd
        Eigen::Vector3d cTgt;
        Eigen::Vector3d cSrc;
        size_t size1 = matchCornerSrc.size();
        for (int j = 0; j < size1; ++j) {
            cTgt += matchCornerSrc[j];
            cSrc += matchCornerSrc[j];
        }
        cTgt = cTgt / size1;
        cSrc = cSrc / size1;
        std::vector<Eigen::Vector3d> rTgt(size1);
        std::vector<Eigen::Vector3d> rSrc(size1);
        for (int j = 0; j < size1; ++j) {
            rTgt[j] = matchCornerTgt[j] - cTgt;
            rSrc[j] = matchCornerSrc[j] - cSrc;
        }
        Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
        for (int j = 0; j < size1; ++j) {
            w += Eigen::Vector3d(rTgt[j](0),rTgt[j](1),rTgt[j](2)) * Eigen::Vector3d(rSrc[j](0),rSrc[j](1),rSrc[j](2)).transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(w,Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d u = svd.matrixU();
        Eigen::Matrix3d v = svd.matrixV();
        Eigen::Matrix3d r = u * (v.transpose());
        Eigen::Vector3d t = cTgt - r * cSrc;
        Eigen::Matrix3d TT;
        TT << r(0,0),r(0,1),t(0,0),
                r(1,0),r(1,1),t(1,0),
                r(2,0),r(2,1),t(2,0);
        cout<<"T_svd=\n"<<TT<<endl;
        ///solve by ceres with L-M
        Vector6d se3 = Mat33toVec61(T);
        double se[6] = {se3[0],se3[1],se3[2],se3[3],se3[4],se3[5]};///exp(Cauchy^) = T
        double so[3] = {se3[3],se3[4],se3[5]};///exp(Phi^) = R
        ceres::Problem problem;
        ceres::LossFunction *lossFunction;
        lossFunction = new ceres::CauchyLoss(0.01);

        for (int j = 0; j < matchCornerSrc.size(); ++j) {
            ceres::CostFunction *costFunctionCPt = new ICPCeres(matchCornerTgt[j],matchCornerSrc[j]);
            problem.AddResidualBlock(costFunctionCPt, nullptr,se);
        }
        for (int j = 0; j < matchSurfSrc.size(); ++j) {
            ceres::CostFunction *costFunctionSpt = new ICPCeres(matchSurfTgt[j],matchSurfSrc[j]);
            problem.AddResidualBlock(costFunctionSpt, nullptr,se);
        }
        for (int j = 0; j < matchTanCornerSrc.size(); ++j) {
            ceres::CostFunction *costFunctionTanCorner = new ICPCeresTan(matchTanCornerTgt[j],matchTanCornerSrc[j]);
            problem.AddResidualBlock(costFunctionTanCorner,lossFunction,so);
        }
        for (int j = 0; j < matchTanSurfSrc.size(); ++j) {
            ceres::CostFunction *costFunctionTanSurf = new ICPCeresTan(matchTanSurfTgt[j],matchTanSurfSrc[j]);
            problem.AddResidualBlock(costFunctionTanSurf,lossFunction,so);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.num_threads = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);

        Eigen::Map<Eigen::Matrix<double,6,1>> selie(se);
        Eigen::Matrix4d T_m1 = Sophus::SE3d::exp(selie).matrix();
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
/*
        T << T_m(0,0),T_m(0,1),T_m(0,3),
                T_m(1,0),T_m(1,1),T_m(1,3),
                0,0,1;
*/

        T << T_m1(0,0),T_m1(0,1),T_m1(0,3),
                T_m1(1,0),T_m1(1,1),T_m1(1,3),
                0,0,1;

/*
        T << r(0,0),r(0,1),t(0,0),
             r(1,0),r(1,1),t(1,0),
             0,0,1;*/
        double sum = 0;
        sum = abs(T(0,0)) + abs(T(0,1)) + abs(T(0,2)) +
              abs(T(1,0)) + abs(T(1,1)) + abs(T(1,2)) +
              abs(T(2,0)) + abs(T(2,1)) + abs(T(2,2)) - 3;
        if ((i > 0 && sum < 0.000001) || i == max_iteration-1) {
            T_st = T * T_st;
            std::cout<<"\033[32m"<<"Final Result!"<<"\033[0m"<<std::endl;
            std::cout<<"T:\n"<<T<<std::endl;
            std::cout<<"T_inv:\n"<<T.inverse()<<std::endl;
            std::cout<<"T_st:\n"<<T_st<<std::endl;
            std::cout<<"T_st_inv:\n"<<T_st.inverse()<<std::endl;
            std::cout<<"time: "<<timer.elapsed()<<std::endl;
            if (i == max_iteration-1) {
                std::cout<<"iteration "<<i<<std::endl;
            }
            break;
        }
    }

}

bool icpTangent(cloudType &nowSharp,cloudType &nowFlat,cloudType &lastLessSharp,cloudType &lastLessFlat,Eigen::Matrix3d &T_st) {
    int cornerSharpNum;
    int surfFlatNum;
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    cloudType _nowCornerSharp, _nowSurfFlat;
    std::vector<Eigen::Vector3d> matchCornerSrc2, matchCornerTgt2;
    std::vector<Eigen::Vector3d> matchSurfSrc2, matchSurfTgt2;
    std::vector<Eigen::Vector3d> matchTanCornerSrc2, matchTanCornerTgt2;
    std::vector<Eigen::Vector3d> matchTanSurfSrc2, matchTanSurfTgt2;
    for (int i = 0; i < max_iteration; ++i) {
        std::cout<<"i: "<<i<<std::endl;
        //nowCornerSharp->cornerSharp->matchCornerSrc,m_cornerLessSharp->m_cornerLessSharpTgt->cloudCornerLessSharpTgt->matchCornerTgt
        std::vector<Eigen::Vector3d> matchCornerSrc, matchCornerTgt;
        //nowSurfFlat->matchSurfFlat,cloudSurfLessFlatTgt->matchSurfTgt
        std::vector<Eigen::Vector3d> matchSurfSrc, matchSurfTgt;
        //matchCornerSrc->matchTanCornerSrc,matchCornerTgt->matchTanCornerTgt
        std::vector<Eigen::Vector3d> matchTanCornerSrc, matchTanCornerTgt;
        //matchSurfSrc->matchTanSurfSrc,matchSurfTgt->matchTanSurfTgt
        std::vector<Eigen::Vector3d> matchTanSurfSrc, matchTanSurfTgt;
        T_st = T * T_st;

        std::vector<pointType> p3(nowSharp.cloud.size());
        for (int j = 0; j < nowSharp.cloud.size(); ++j) {
            Eigen::Vector3d p0;
            p0 << nowSharp.cloud.at(j).pt(0),
                    nowSharp.cloud.at(j).pt(1),
                    1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p3.at(j).pt = {p1[0],p1[1]};
        }
        _nowCornerSharp.cloud = p3;

        std::vector<pointType> p4(nowFlat.cloud.size());
        for (int j = 0; j < nowFlat.cloud.size(); ++j) {
            Eigen::Vector3d p0;
            p0 << nowFlat.cloud.at(j).pt(0),
                    nowFlat.cloud.at(j).pt(1),
                    1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p4.at(j).pt = {p1[0],p1[1]};
        }
        _nowSurfFlat.cloud = p4;

        cornerSharpNum = _nowCornerSharp.cloud.size();
        surfFlatNum = _nowSurfFlat.cloud.size();
        for (auto & j : _nowCornerSharp.cloud) {
            show01.points.emplace_back(j.pt(0),j.pt(1),0);
        }
        for (auto & j : _nowSurfFlat.cloud) {
            show02.points.emplace_back(j.pt(0),j.pt(1),0);
        }
        int k = 1;
        Eigen::VectorXi ind(k);
        Eigen::VectorXd dist(k);
        Eigen::VectorXi index(k);
        Eigen::VectorXd distance(k);
            //deal with sharp points, red; less sharp, yellow
            for (int j = 0; j < cornerSharpNum; ++j) {
                Eigen::Matrix<double, 6, 1> tgt;
                // search closest pts for sharpPoints from lessSharpPoints
                lastLessSharp.nns->knn(_nowCornerSharp.cloud.at(j).pt, ind, dist, k, 0, 0, 0.5);
                if (dist[0] < std::numeric_limits<double>::infinity() && !std::isinf(dist[0])) {
                    tgt = lastLessSharp.kdTreeData.col(ind(0));
                    matchCornerSrc.emplace_back(_nowCornerSharp.cloud.at(j).pt(0), _nowCornerSharp.cloud.at(j).pt(1),0);
                    matchCornerTgt.emplace_back(tgt(0), tgt(1), 0);
                    //std::cout<<"cornerSrc: "<<_nowCornerSharp.cloud.at(j).pt(0)<<" "<<_nowCornerSharp.cloud.at(j).pt(1)<<std::endl;
                    //std::cout<<"cornerTgt: "<<tgt(0)<<" "<<tgt(1)<<std::endl;

                }
            }
            //deal with flat points, blue
            for (int j = 0; j < surfFlatNum; ++j) {
                Eigen::Matrix<double, 6, 1> tgt;
                lastLessFlat.nns->knn(_nowSurfFlat.cloud.at(j).pt, index, distance, k, 0, 0, 0.25);
                if (distance[0] < std::numeric_limits<double>::infinity() && !std::isinf(distance[0])) {
                    tgt = lastLessFlat.kdTreeData.col(index(0));
                    matchSurfSrc.emplace_back(_nowSurfFlat.cloud.at(j).pt(0), _nowSurfFlat.cloud.at(j).pt(1), 0);
                    matchSurfTgt.emplace_back(tgt(0), tgt(1), 0);
                }
            }

            //deal with tangent vector
            cloudType cts, ctt, ctSurfS, ctSurfT;
            convertMat2cloudType(matchCornerSrc, cts);
            convertMat2cloudType(matchCornerTgt, ctt);
            computeTangentCurvature(cts);
            computeTangentCurvature(ctt);
            size_t s1 = cts.cloud.size();
            if (!cts.cloud.empty()) {
                for (int j = 0; j < s1 - 1; ++j) {
                    double d1 = sqrt(pow(cts.cloud.at(j).pt(0),2) + pow(cts.cloud.at(j).pt(1),2))
                                - sqrt(pow(ctt.cloud.at(j).pt(0),2) + pow(ctt.cloud.at(j).pt(1),2));
                    double d2 = sqrt(pow(cts.cloud.at(j+1).pt(0),2) + pow(cts.cloud.at(j+1).pt(1),2))
                                - sqrt(pow(ctt.cloud.at(j+1).pt(0),2) + pow(ctt.cloud.at(j+1).pt(1),2));
                    //double d1 = sqrt(pow(cts.cloud.at(j).pt(0) - ctt.cloud.at(j).pt(0), 2) +
                    //                 pow(cts.cloud.at(j).pt(1) - ctt.cloud.at(j).pt(1), 2));
                    //double d2 = sqrt(pow(cts.cloud.at(j + 1).pt(0) - ctt.cloud.at(j + 1).pt(0), 2) +
                    //                 pow(cts.cloud.at(j + 1).pt(1) - ctt.cloud.at(j + 1).pt(1), 2));
                    //double d3 = sqrt(pow(cts.cloud.at(j+1).pt(0),2) + pow(cts.cloud.at(j+1).pt(1),2));
                    //double d4 = sqrt(pow(ctt.cloud.at(j+1).pt(0),2) + pow(ctt.cloud.at(j+1).pt(1),2));
                    double c1 = cts.cloud.at(j).lc;
                    double c2 = ctt.cloud.at(j).lc;
                    double c3 = cts.cloud.at(j+1).lc;
                    double c4 = ctt.cloud.at(j+1).lc;

                    if (d1 < 0.3 && d2 < 0.3 && !std::isinf(c1+c2+c3+c4) &&
                        c1 > 0.05 && c2 > 0.05 && c3 > 0.05 && c4 > 0.05 &&
                        cts.cloud.at(j).tan(0) != 0 &&
                        ctt.cloud.at(j).tan(0) != 0) {//0.1
                        matchTanCornerSrc.emplace_back(cts.cloud.at(j).tan(0), cts.cloud.at(j).tan(1), 0);
                        matchTanCornerTgt.emplace_back(ctt.cloud.at(j).tan(0), ctt.cloud.at(j).tan(1), 0);
                        //std::cout<<" cts: "<<cts.cloud.at(j).pt(0)<<" "<<cts.cloud.at(j).pt(1)
                        //         <<" ctt: "<<ctt.cloud.at(j).pt(0)<<" "<<ctt.cloud.at(j).pt(1)
                        //         <<" d1: "<<d1<<" d2: "<<d2
                        //         <<" c1: "<<c1<<" c2: "<<c2<<std::endl;

                        if (i == 0) {
                            tangentSharp.push_back(pcl::PointXYZ(cts.cloud.at(j).tan(0),cts.cloud.at(j).tan(1),1));
                            tangentSharp2.push_back(pcl::PointXYZ(ctt.cloud.at(j).tan(0),ctt.cloud.at(j).tan(1),0));
                            Eigen::Vector2d a = cts.cloud.at(j).tan;
                            Eigen::Vector2d b = ctt.cloud.at(j).tan;
                            double phi = acos(a.dot(b)/(a.norm() * b.norm()));
                            ///std::cout<<"src: "<<cts.cloud.at(j).pt(0)<<" "<<cts.cloud.at(j).pt(1)<<" "<<cts.cloud.at(j).tan(0)<<" "<<cts.cloud.at(j).tan(1)<<" "<<phi<<std::endl;
                            ///std::cout<<"tgt: "<<ctt.cloud.at(j).pt(0)<<" "<<ctt.cloud.at(j).pt(1)<<" "<<ctt.cloud.at(j).tan(0)<<" "<<ctt.cloud.at(j).tan(1)<<" "<<phi<<std::endl;
                        }

                    }
                }
            }


            convertMat2cloudType(matchSurfSrc, ctSurfS);
            convertMat2cloudType(matchSurfTgt, ctSurfT);
            computeTangentCurvature(ctSurfS);
            computeTangentCurvature(ctSurfT);
            size_t s2 = ctSurfS.cloud.size();
            if (!ctSurfS.cloud.empty()) {
                for (int j = 0; j < s2 - 1; ++j) {
                    //double d1 = sqrt(pow(ctSurfS.cloud.at(j).pt(0),2) + pow(ctSurfS.cloud.at(j).pt(1),2));
                    //double d2 = sqrt(pow(ctSurfT.cloud.at(j).pt(0),2) + pow(ctSurfT.cloud.at(j).pt(1),2));
                    //double d3 = sqrt(pow(ctSurfS.cloud.at(j+1).pt(0),2) + pow(ctSurfS.cloud.at(j+1).pt(1),2));
                    //double d4 = sqrt(pow(ctSurfT.cloud.at(j+1).pt(0),2) + pow(ctSurfT.cloud.at(j+1).pt(1),2));
                    double d1 = sqrt(pow(ctSurfS.cloud.at(j).pt(0) - ctSurfT.cloud.at(j).pt(0), 2) -
                                     pow(ctSurfS.cloud.at(j).pt(1) - ctSurfT.cloud.at(j).pt(1), 2));
                    double d2 = sqrt(pow(ctSurfS.cloud.at(j + 1).pt(0) - ctSurfT.cloud.at(j + 1).pt(0), 2) -
                                     pow(ctSurfS.cloud.at(j + 1).pt(1) - ctSurfT.cloud.at(j + 1).pt(1), 2));
                    double c1 = ctSurfS.cloud.at(j).lc - ctSurfT.cloud.at(j).lc;
                    double c2 = ctSurfS.cloud.at(j + 1).lc - ctSurfT.cloud.at(j + 1).lc;
                    double p1 = ctSurfS.cloud.at(j).lp - ctSurfT.cloud.at(j).lp;
                    double p2 = ctSurfS.cloud.at(j + 1).lp - ctSurfT.cloud.at(j + 1).lp;
                    if (d1 < 0.02 && d2 < 0.02 &&
                        abs(c1) < 0.05 && abs(c2) < 0.05 &&
                        abs(p1) < 0.02 && abs(p2) < 0.02 && !isnan(c1 + c2 + p1 + p2)) {//0.05
                        matchTanSurfSrc.emplace_back(ctSurfS.cloud.at(j).tan(0), ctSurfS.cloud.at(j).tan(1), 0);
                        matchTanSurfTgt.emplace_back(ctSurfT.cloud.at(j).tan(0), ctSurfT.cloud.at(j).tan(1), 0);

                        if (i == 0) {
                            tangentFlat.push_back(pcl::PointXYZ(ctSurfS.cloud.at(j).tan(0),ctSurfS.cloud.at(j).tan(1),1));
                            tangentFlat2.push_back(pcl::PointXYZ(ctSurfT.cloud.at(j).tan(0),ctSurfT.cloud.at(j).tan(1),0));

                        }

                    }
                }
            }



///solve by svd
        Eigen::Vector3d cTgt;
        Eigen::Vector3d cSrc;
        size_t size1 = matchCornerSrc.size();
        for (int j = 0; j < size1; ++j) {
            cTgt += matchCornerSrc[j];
            cSrc += matchCornerSrc[j];
        }
        cTgt = cTgt / size1;
        cSrc = cSrc / size1;
        std::vector<Eigen::Vector3d> rTgt(size1);
        std::vector<Eigen::Vector3d> rSrc(size1);
        for (int j = 0; j < size1; ++j) {
            rTgt[j] = matchCornerTgt[j] - cTgt;
            rSrc[j] = matchCornerSrc[j] - cSrc;
        }
        Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
        for (int j = 0; j < size1; ++j) {
            w += Eigen::Vector3d(rTgt[j](0),rTgt[j](1),rTgt[j](2)) * Eigen::Vector3d(rSrc[j](0),rSrc[j](1),rSrc[j](2)).transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(w,Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d u = svd.matrixU();
        Eigen::Matrix3d v = svd.matrixV();
        Eigen::Matrix3d r = u * (v.transpose());
        Eigen::Vector3d t = cTgt - r * cSrc;
        Eigen::Matrix3d TT;
        TT << r(0,0),r(0,1),t(0,0),
                r(1,0),r(1,1),t(1,0),
                r(2,0),r(2,1),1;
        //cout<<"T_svd=\n"<<TT<<endl;
        ///solve by ceres with L-M
        Vector6d se3 = Mat33toVec61(T);
        double se[6] = {se3[0],se3[1],se3[2],se3[3],se3[4],se3[5]};///exp(Cauchy^) = T
        double so[3] = {se3[3],se3[4],se3[5]};///exp(Phi^) = R
        ceres::Problem problem;
        ceres::LossFunction *lossFunction;
        lossFunction = new ceres::CauchyLoss(0.05);//0.05

        for (int j = 0; j < matchCornerSrc.size(); ++j) {
            ceres::CostFunction *costFunctionCPt = new ICPCeres(matchCornerTgt[j],matchCornerSrc[j]);
            problem.AddResidualBlock(costFunctionCPt, nullptr,se);
        }
        for (int j = 0; j < matchSurfSrc.size(); ++j) {
            ceres::CostFunction *costFunctionSpt = new ICPCeres(matchSurfTgt[j],matchSurfSrc[j]);
            problem.AddResidualBlock(costFunctionSpt, nullptr,se);
        }

        if (matchTanCornerSrc.size() > 5) {
            for (int j = 0; j < matchTanCornerSrc.size(); ++j) {
                ceres::CostFunction *costFunctionTanCorner = new ICPCeresTan(matchTanCornerTgt[j],matchTanCornerSrc[j]);
                problem.AddResidualBlock(costFunctionTanCorner,lossFunction,so);
            }
        }

        if (matchTanSurfSrc.size() > 10) {
            for (int j = 0; j < matchTanSurfSrc.size(); ++j) {
                ceres::CostFunction *costFunctionTanSurf = new ICPCeresTan(matchTanSurfTgt[j],matchTanSurfSrc[j]);
                problem.AddResidualBlock(costFunctionTanSurf,lossFunction,so);
            }
        }


        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.num_threads = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);

        Eigen::Map<Eigen::Matrix<double,6,1>> selie(se);
        Eigen::Matrix4d T_m1 = Sophus::SE3d::exp(selie).matrix();
        std::cout<<"estimateT =\n"<<T_m1<<std::endl;

        Eigen::Map<Eigen::Vector3d> solie(so);
        Eigen::Matrix3d T_m2 = Sophus::SO3d::exp(solie).matrix();
        std::cout<<"estimateR =\n"<<T_m2<<std::endl;

        Eigen::Matrix4d T_m2m = Eigen::Matrix4d::Identity();
        T_m2m << T_m2(0,0),T_m2(0,1),T_m2(0,2),0,
                T_m2(1,0),T_m2(1,1),T_m2(1,2),0,
                T_m2(2,0),T_m2(2,1),T_m2(2,2),0,
                0,0,0,1;

        Eigen::Matrix4d T_m = T_m2m * T_m1;

        if (!matchTanCornerSrc.empty() && !matchTanSurfSrc.empty()) {
            T << T_m(0,0),T_m(0,1),T_m(0,3),
            T_m(1,0),T_m(1,1),T_m(1,3),
            0,0,1;
        } else {
            T << T_m1(0,0),T_m1(0,1),T_m1(0,3),
            T_m1(1,0),T_m1(1,1),T_m1(1,3),
            0,0,1;
        }

/*
        T << T_m1(0,0),T_m1(0,1),T_m1(0,3),
                T_m1(1,0),T_m1(1,1),T_m1(1,3),
                0,0,1;
        */
        double sum = 0;
        sum = abs(T(0,0)) + abs(T(0,1)) + abs(T(0,2)) +
              abs(T(1,0)) + abs(T(1,1)) + abs(T(1,2)) +
              abs(T(2,0)) + abs(T(2,1)) + abs(T(2,2)) - 3;
        if ((i > 0 && sum < epsilon) || i == max_iteration-1) {
            T_st = T * T_st;
            std::cout<<"\033[32m"<<"Final Result!"<<"\033[0m"<<std::endl;
            std::cout<<"T:\n"<<T<<std::endl;
            std::cout<<"T_inv:\n"<<T.inverse()<<std::endl;
            std::cout<<"T_st:\n"<<T_st<<std::endl;
            std::cout<<"T_st_inv:\n"<<T_st.inverse()<<std::endl;
            if (i == max_iteration-1) {
                std::cout<<"iteration "<<i<<std::endl;
            }
            break;
        }
    }

}

bool GICP(pcl::PointCloud<pcl::PointXYZ>::Ptr &s, pcl::PointCloud<pcl::PointXYZ>::Ptr &t, Eigen::Matrix3d &T_st) {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
    tree1->setInputCloud(s);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud(t);
    gicp.setSearchMethodSource(tree1);
    gicp.setSearchMethodTarget(tree2);

    gicp.setInputSource(s);///source
    gicp.setInputTarget(t);///target
    gicp.setMaximumOptimizerIterations(100);
    gicp.setTransformationEpsilon(1e-10);
    gicp.setEuclideanFitnessEpsilon(0.001);
    gicp.setMaximumIterations(25);
    pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*gicp_cloud);

    Eigen::Matrix4f T;
    T = gicp.getFinalTransformation();

    T_st << T(0,0), T(0,1), T(0,3),
            T(1,0), T(1,1), T(1,3),
            0,0,1;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"newicp");
    ros::NodeHandle nh;
    ros::Publisher pub01full = nh.advertise<sensor_msgs::PointCloud2>("full01",10);
    ros::Publisher pub01sharp = nh.advertise<sensor_msgs::PointCloud2>("sharp01",10);
    ros::Publisher pub01lessSharp = nh.advertise<sensor_msgs::PointCloud2>("lessSharp01",10);
    ros::Publisher pub01flat = nh.advertise<sensor_msgs::PointCloud2>("flat01",10);
    ros::Publisher pub01lessFlat = nh.advertise<sensor_msgs::PointCloud2>("lessFlat01",10);

    ros::Publisher pub02full = nh.advertise<sensor_msgs::PointCloud2>("full02",10);
    ros::Publisher pub02sharp = nh.advertise<sensor_msgs::PointCloud2>("sharp02",10);
    ros::Publisher pub02lessSharp = nh.advertise<sensor_msgs::PointCloud2>("lessSharp02",10);
    ros::Publisher pub02flat = nh.advertise<sensor_msgs::PointCloud2>("flat02",10);
    ros::Publisher pub02lessFlat = nh.advertise<sensor_msgs::PointCloud2>("lessFlat02",10);

    ros::Publisher pubTSharp = nh.advertise<sensor_msgs::PointCloud2>("TSharp",10);
    ros::Publisher pubTFlat = nh.advertise<sensor_msgs::PointCloud2>("TFlat",10);
    ros::Publisher pubTSharp2 = nh.advertise<sensor_msgs::PointCloud2>("TSharp2",10);
    ros::Publisher pubTFlat2 = nh.advertise<sensor_msgs::PointCloud2>("TFlat2",10);


    pcl::PointCloud<pcl::PointXYZ>::Ptr full01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sharp01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lessSharp01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr flat01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lessFlat01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl04full.pcd",*full01);///target
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl04sharp.pcd",*sharp01);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl04lessSharp.pcd",*lessSharp01);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl04Flat.pcd",*flat01);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl04lessFlat.pcd",*lessFlat01);

    cloudType m_cornerLessSharpTgt,m_surfLessFlatTgt;
    pcl2cloudType(lessSharp01,m_cornerLessSharpTgt);
    pcl2cloudType(lessFlat01,m_surfLessFlatTgt);

    cloudType cloudCornerLessSharpTgt,cloudSurfLessFlatTgt;
    setTargetPointCloud(m_cornerLessSharpTgt.cloud,cloudCornerLessSharpTgt);
    setTargetPointCloud(m_surfLessFlatTgt.cloud,cloudSurfLessFlatTgt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr full02(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sharp02(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lessSharp02(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr flat02(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lessFlat02(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl05full.pcd",*full02);/////source
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl05sharp.pcd",*sharp02);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl05lessSharp.pcd",*lessSharp02);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl05Flat.pcd",*flat02);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/zl05lessFlat.pcd",*lessFlat02);

    cloudType m_cornerPtsSharp,m_surfPtsFlat;
    pcl2cloudType(sharp02,m_cornerPtsSharp);
    pcl2cloudType(flat02,m_surfPtsFlat);

    cloudType nowCornerSharp02, nowSurfFlat02;
    setSourcePointCloud(m_cornerPtsSharp.cloud,nowCornerSharp02);
    setSourcePointCloud(m_surfPtsFlat.cloud,nowSurfFlat02);

    boost::timer timer;
    cloudType _nowCornerSharp, _nowSurfFlat;
    int cornerSharpNum;
    int surfFlatNum;
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d T_st = Eigen::Matrix3d::Identity();

    ///gicp
    pcl::console::TicToc time;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
    tree1->setInputCloud(full02);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud(full01);
    gicp.setSearchMethodSource(tree1);
    gicp.setSearchMethodTarget(tree2);

    gicp.setInputSource(full02);///source
    gicp.setInputTarget(full01);///target
    gicp.setMaximumOptimizerIterations(100);
    gicp.setTransformationEpsilon(1e-10);
    gicp.setEuclideanFitnessEpsilon(0.001);
    gicp.setMaximumIterations(35);
    pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*gicp_cloud);
    cout << "Applied " << 35 << " GICP iterations in " << time.toc()/1000 << " s" << endl;
    cout << "\nGICP has converged, score is " << gicp.getFitnessScore() << endl;
    cout << "变换矩阵：\n" << gicp.getFinalTransformation() << endl;
    cout <<" 变换矩阵的逆：\n" << gicp.getFinalTransformation().inverse()<<endl;

    Eigen::Matrix3d T_gicp;
    GICP(full02,full01,T_gicp);
    cout << "变换矩阵：\n" << T_gicp << endl;

    pcl::transformPointCloud(*full02,*gicp_cloud,gicp.getFinalTransformation());
    //visualized
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("Result"));
    view->setBackgroundColor(255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(full01,255,0,0);
    view->addPointCloud<pcl::PointXYZ>(full01,target_color,"target cloud");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"target cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(full02,0,255,0);
    view->addPointCloud<pcl::PointXYZ>(full02,input_color,"input cloud");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"input cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(gicp_cloud,0,0,255);
    view->addPointCloud<pcl::PointXYZ>(gicp_cloud,output_color,"output cloud");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"output cloud");

    while (!view->wasStopped()) {
        view->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    ///NICP


    icpTangent(nowCornerSharp02,nowSurfFlat02,cloudCornerLessSharpTgt,cloudSurfLessFlatTgt,T_st);



        sensor_msgs::PointCloud2 tangentSharpMsg;
        pcl::toROSMsg(tangentSharp,tangentSharpMsg);
        tangentSharpMsg.header.frame_id = "map";

        sensor_msgs::PointCloud2 tangentFlatMsg;
        pcl::toROSMsg(tangentFlat,tangentFlatMsg);
        tangentFlatMsg.header.frame_id = "map";

        sensor_msgs::PointCloud2 tangentSharpMsg2;
        pcl::toROSMsg(tangentSharp2,tangentSharpMsg2);
        tangentSharpMsg2.header.frame_id = "map";

        sensor_msgs::PointCloud2 tangentFlatMsg2;
        pcl::toROSMsg(tangentFlat2,tangentFlatMsg2);
        tangentFlatMsg2.header.frame_id = "map";



/*
    for (int i = 0; i < max_iteration; ++i) {
        //nowCornerSharp->cornerSharp->matchCornerSrc,m_cornerLessSharp->m_cornerLessSharpTgt->cloudCornerLessSharpTgt->matchCornerTgt
        std::vector<Eigen::Vector3d> matchCornerSrc, matchCornerTgt;
        //nowSurfFlat->matchSurfFlat,cloudSurfLessFlatTgt->matchSurfTgt
        std::vector<Eigen::Vector3d> matchSurfSrc, matchSurfTgt;
        //matchCornerSrc->matchTanCornerSrc,matchCornerTgt->matchTanCornerTgt
        std::vector<Eigen::Vector3d> matchTanCornerSrc, matchTanCornerTgt;
        //matchSurfSrc->matchTanSurfSrc,matchSurfTgt->matchTanSurfTgt
        std::vector<Eigen::Vector3d> matchTanSurfSrc, matchTanSurfTgt;
        std::cout<<"\033[31m"<<"iter: "<<i<<"\033[0m"<<std::endl;
        T_st = T * T_st;

        std::vector<pointType> p3(nowCornerSharp02.cloud.size());
        for (int j = 0; j < nowCornerSharp02.cloud.size(); ++j) {
            Eigen::Vector3d p0;
            p0 << nowCornerSharp02.cloud.at(j).pt(0),
                  nowCornerSharp02.cloud.at(j).pt(1),
                  1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p3.at(j).pt = {p1[0],p1[1]};
        }
        _nowCornerSharp.cloud = p3;

        std::vector<pointType> p4(nowSurfFlat02.cloud.size());
        for (int j = 0; j < nowSurfFlat02.cloud.size(); ++j) {
            Eigen::Vector3d p0;
            p0 << nowSurfFlat02.cloud.at(j).pt(0),
                  nowSurfFlat02.cloud.at(j).pt(1),
                  1;
            Eigen::Vector3d p1;
            p1 = T_st * p0;
            p4.at(j).pt = {p1[0],p1[1]};
        }
        _nowSurfFlat.cloud = p4;

        cornerSharpNum = _nowCornerSharp.cloud.size();
        surfFlatNum = _nowSurfFlat.cloud.size();

        for (auto & j : _nowCornerSharp.cloud) {
            show01.points.emplace_back(j.pt(0),j.pt(1),0);
        }
        for (auto & j : _nowSurfFlat.cloud) {
            show02.points.emplace_back(j.pt(0),j.pt(1),0);
        }




        int k = 1;
        Eigen::VectorXi ind(k);
        Eigen::VectorXd dist(k);
        Eigen::VectorXi index(k);
        Eigen::VectorXd distance(k);
        //deal with sharp points, red; less sharp, yellow
        for (int j = 0; j < cornerSharpNum; ++j) {
            Eigen::Matrix<double,6,1> tgt;
            // search closest pts for sharpPoints from lessSharpPoints
            cloudCornerLessSharpTgt.nns->knn(_nowCornerSharp.cloud.at(j).pt,ind,dist,k,0,0,0.5);
            if (dist[0] < std::numeric_limits<double>::infinity() && !std::isinf(dist[0])) {
                tgt = cloudCornerLessSharpTgt.kdTreeData.col(ind(0));
                matchCornerSrc.emplace_back(_nowCornerSharp.cloud.at(j).pt(0),_nowCornerSharp.cloud.at(j).pt(1),0);
                matchCornerTgt.emplace_back(tgt(0),tgt(1),0);
            }
        }
        cout<<endl;
        //deal with flat points, blue
        for (int j = 0; j < surfFlatNum; ++j) {
            Eigen::Matrix<double,6,1> tgt;
            cloudSurfLessFlatTgt.nns->knn(_nowSurfFlat.cloud.at(j).pt,index,distance,k,0,0,0.25);
            if (distance[0] < std::numeric_limits<double>::infinity() && !std::isinf(distance[0])) {
                tgt = cloudSurfLessFlatTgt.kdTreeData.col(index(0));
                matchSurfSrc.emplace_back(_nowSurfFlat.cloud.at(j).pt(0),_nowSurfFlat.cloud.at(j).pt(1),0);
                matchSurfTgt.emplace_back(tgt(0),tgt(1),0);
            }
        }

        //deal with tangent vector
        cloudType cts,ctt,ctSurfS,ctSurfT;
        convertMat2cloudType(matchCornerSrc,cts);
        convertMat2cloudType(matchCornerTgt,ctt);
        computeTangentCurvature(cts);
        computeTangentCurvature(ctt);
        size_t s1 = cts.cloud.size();
        for (int j = 0; j < s1 - 1; ++j) {
            double d1 = sqrt(pow(cts.cloud.at(j).pt(0),2) + pow(cts.cloud.at(j).pt(1),2));
            double d2 = sqrt(pow(ctt.cloud.at(j).pt(0),2) + pow(ctt.cloud.at(j).pt(1),2));
            double d3 = sqrt(pow(cts.cloud.at(j+1).pt(0),2) + pow(cts.cloud.at(j+1).pt(1),2));
            double d4 = sqrt(pow(ctt.cloud.at(j+1).pt(0),2) + pow(ctt.cloud.at(j+1).pt(1),2));
            if (abs(d1 - d2) < 0.1 && abs(d3 -d4) < 0.1) {
                matchTanCornerSrc.emplace_back(cts.cloud.at(j).tan(0),cts.cloud.at(j).tan(1),0);
                matchTanCornerTgt.emplace_back(ctt.cloud.at(j).tan(0),ctt.cloud.at(j).tan(1),0);
            }
        }

        convertMat2cloudType(matchSurfSrc,ctSurfS);
        convertMat2cloudType(matchSurfTgt,ctSurfT);
        computeTangentCurvature(ctSurfS);
        computeTangentCurvature(ctSurfT);
        size_t s2 = ctSurfS.cloud.size();
        for (int j = 0; j < s2 - 1; ++j) {
            double d1 = sqrt(pow(ctSurfS.cloud.at(j).pt(0),2) + pow(ctSurfS.cloud.at(j).pt(1),2));
            double d2 = sqrt(pow(ctSurfT.cloud.at(j).pt(0),2) + pow(ctSurfT.cloud.at(j).pt(1),2));
            double d3 = sqrt(pow(ctSurfS.cloud.at(j+1).pt(0),2) + pow(ctSurfS.cloud.at(j+1).pt(1),2));
            double d4 = sqrt(pow(ctSurfT.cloud.at(j+1).pt(0),2) + pow(ctSurfT.cloud.at(j+1).pt(1),2));
            double c1 = ctSurfS.cloud.at(j).lc - ctSurfT.cloud.at(j).lc;
            double c2 = ctSurfS.cloud.at(j+1).lc - ctSurfT.cloud.at(j+1).lc;
            double p1 = ctSurfS.cloud.at(j).lp - ctSurfT.cloud.at(j).lp;
            double p2 = ctSurfS.cloud.at(j+1).lp - ctSurfT.cloud.at(j+1).lp;
            if (abs(d1 - d2) < 0.05 && abs(d3 -d4) < 0.05 &&
                abs(c1) < 0.05 && abs(c2) < 0.05 &&
                abs(p1) < 0.05 && abs(p2) < 0.05) {
                matchTanSurfSrc.emplace_back(ctSurfS.cloud.at(j).tan(0),ctSurfS.cloud.at(j).tan(1),0);
                matchTanSurfTgt.emplace_back(ctSurfT.cloud.at(j).tan(0),ctSurfT.cloud.at(j).tan(1),0);
            }
        }


        ///solve by svd
        Eigen::Vector3d cTgt;
        Eigen::Vector3d cSrc;
        size_t size1 = matchCornerSrc.size();
        for (int j = 0; j < size1; ++j) {
            cTgt += matchCornerSrc[j];
            cSrc += matchCornerSrc[j];
        }
        cTgt = cTgt / size1;
        cSrc = cSrc / size1;
        std::vector<Eigen::Vector3d> rTgt(size1);
        std::vector<Eigen::Vector3d> rSrc(size1);
        for (int j = 0; j < size1; ++j) {
            rTgt[j] = matchCornerTgt[j] - cTgt;
            rSrc[j] = matchCornerSrc[j] - cSrc;
        }
        Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
        for (int j = 0; j < size1; ++j) {
            w += Eigen::Vector3d(rTgt[j](0),rTgt[j](1),rTgt[j](2)) * Eigen::Vector3d(rSrc[j](0),rSrc[j](1),rSrc[j](2)).transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(w,Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d u = svd.matrixU();
        Eigen::Matrix3d v = svd.matrixV();
        Eigen::Matrix3d r = u * (v.transpose());
        Eigen::Vector3d t = cTgt - r * cSrc;
        Eigen::Matrix3d TT;
        TT << r(0,0),r(0,1),t(0,0),
              r(1,0),r(1,1),t(1,0),
              r(2,0),r(2,1),t(2,0);
        cout<<"T_svd=\n"<<TT<<endl;
        ///solve by ceres with L-M
        Vector6d se3 = Mat33toVec61(T);
        double se[6] = {se3[0],se3[1],se3[2],se3[3],se3[4],se3[5]};///exp(Cauchy^) = T
        double so[3] = {se3[3],se3[4],se3[5]};///exp(Phi^) = R
        ceres::Problem problem;
        ceres::LossFunction *lossFunction;
        lossFunction = new ceres::CauchyLoss(0.01);

        for (int j = 0; j < matchCornerSrc.size(); ++j) {
            ceres::CostFunction *costFunctionCPt = new ICPCeres(matchCornerTgt[j],matchCornerSrc[j]);
            problem.AddResidualBlock(costFunctionCPt, nullptr,se);
        }
        for (int j = 0; j < matchSurfSrc.size(); ++j) {
            ceres::CostFunction *costFunctionSpt = new ICPCeres(matchSurfTgt[j],matchSurfSrc[j]);
            problem.AddResidualBlock(costFunctionSpt, nullptr,se);
        }
        for (int j = 0; j < matchTanCornerSrc.size(); ++j) {
            ceres::CostFunction *costFunctionTanCorner = new ICPCeresTan(matchTanCornerTgt[j],matchTanCornerSrc[j]);
            problem.AddResidualBlock(costFunctionTanCorner,lossFunction,so);
        }
        for (int j = 0; j < matchTanSurfSrc.size(); ++j) {
            ceres::CostFunction *costFunctionTanSurf = new ICPCeresTan(matchTanSurfTgt[j],matchTanSurfSrc[j]);
            problem.AddResidualBlock(costFunctionTanSurf,lossFunction,so);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.num_threads = 4;

        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);

        Eigen::Map<Eigen::Matrix<double,6,1>> selie(se);
        Eigen::Matrix4d T_m1 = Sophus::SE3d::exp(selie).matrix();
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


        T << T_m1(0,0),T_m1(0,1),T_m1(0,3),
                T_m1(1,0),T_m1(1,1),T_m1(1,3),
                0,0,1;

        double sum = 0;
        sum = abs(T(0,0)) + abs(T(0,1)) + abs(T(0,2)) +
              abs(T(1,0)) + abs(T(1,1)) + abs(T(1,2)) +
              abs(T(2,0)) + abs(T(2,1)) + abs(T(2,2)) - 3;
        if ((i > 0 && sum < 0.000001) || i == max_iteration-1) {
            T_st = T * T_st;
            std::cout<<"\033[32m"<<"Final Result!"<<"\033[0m"<<std::endl;
            std::cout<<"T:\n"<<T<<std::endl;
            std::cout<<"T_inv:\n"<<T.inverse()<<std::endl;
            std::cout<<"T_st:\n"<<T_st<<std::endl;
            std::cout<<"T_st_inv:\n"<<T_st.inverse()<<std::endl;
            std::cout<<"time: "<<timer.elapsed()<<"(s)"<<std::endl;
            if (i == max_iteration-1) {
                std::cout<<"iteration "<<i<<std::endl;
            }
            break;
        }
    }
*/

    sensor_msgs::PointCloud2 fullOut01,sharpOut01,lessSharpOut01,flatOut01,lessFlatOut01;
    pcl::toROSMsg(*full01,fullOut01);
    pcl::toROSMsg(*sharp01,sharpOut01);
    pcl::toROSMsg(*lessSharp01,lessSharpOut01);
    pcl::toROSMsg(*flat01,flatOut01);
    pcl::toROSMsg(*lessFlat01,lessFlatOut01);
    fullOut01.header.frame_id = "map";
    sharpOut01.header.frame_id = "map";
    lessSharpOut01.header.frame_id = "map";
    flatOut01.header.frame_id = "map";
    lessFlatOut01.header.frame_id = "map";

    sensor_msgs::PointCloud2 fullMidOut01,sharpMidOut01,lessSharpMidOut01,flatMidOut01,lessFlatMidOut01;
    pcl::toROSMsg(show01,sharpMidOut01);
    pcl::toROSMsg(show02,flatMidOut01);
    sharpMidOut01.header.frame_id = "map";
    flatMidOut01.header.frame_id = "map";
    ros::Publisher sharpMidPub = nh.advertise<sensor_msgs::PointCloud2>("sharpMid01",10);
    ros::Publisher flatMidPub = nh.advertise<sensor_msgs::PointCloud2>("flatMid01",10);

    sensor_msgs::PointCloud2 fullOut02,sharpOut02,lessSharpOut02,flatOut02,lessFlatOut02;
    pcl::toROSMsg(*full02,fullOut02);
    pcl::toROSMsg(*sharp02,sharpOut02);
    pcl::toROSMsg(*lessSharp02,lessSharpOut02);
    pcl::toROSMsg(*flat02,flatOut02);
    pcl::toROSMsg(*lessFlat02,lessFlatOut02);
    fullOut02.header.frame_id = "map";
    sharpOut02.header.frame_id = "map";
    lessSharpOut02.header.frame_id = "map";
    flatOut02.header.frame_id = "map";
    lessFlatOut02.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while (ros::ok()){
        pubTSharp.publish(tangentSharpMsg);
        pubTFlat.publish(tangentFlatMsg);
        pubTSharp2.publish(tangentSharpMsg2);
        pubTFlat2.publish(tangentFlatMsg2);




        pub01full.publish(fullOut01);//white
        pub01sharp.publish(sharpOut01);//red
        pub01lessSharp.publish(lessSharpOut01);//yellow
        pub01flat.publish(flatOut01);//blue
        pub01lessFlat.publish(lessFlatOut01);//deep blue

        pub02full.publish(fullOut02);
        pub02sharp.publish(sharpOut02);
        pub02lessSharp.publish(lessSharpOut02);
        pub02flat.publish(flatOut02);
        pub02lessFlat.publish(lessFlatOut02);

        sharpMidPub.publish(sharpMidOut01);//green
        flatMidPub.publish(flatMidOut01);//green

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
