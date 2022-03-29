//
// Created by lee on 2021/4/30.
//
#include <ros/ros.h>
#include <iostream>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nabo/nabo.h>
#include <boost/timer.hpp>
using namespace std;

Nabo::NNSearchD* nnSNeighbor = nullptr;
int max_iteration = 100;
double epsilon = 1.0e-10;
double minDist = 0.5;

void findTransformation ( const std::vector< Eigen::Vector3d >& pts_model,
                          const std::vector< Eigen::Vector3d >& pts_cloud,
                          int max_iteration, double epsilon, double min_err,
                          Eigen::Matrix3d& R, Eigen::Vector3d& t )
{
    // default settings.
    const double min_err2 = min_err * min_err;
    const double factor = 9.0;///
    const int n_selected_pts = 100;///
    const int step = pts_cloud.size() / n_selected_pts;	/// step for select points.

    // two vectors for matched points.
    std::vector<Eigen::Vector3d> pts_cloud_matched;
    pts_cloud_matched.reserve (n_selected_pts);
    std::vector<Eigen::Vector3d> pts_model_matched;
    pts_model_matched.reserve (n_selected_pts);

    // construct kd-tree for model cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for ( size_t i = 0; i < pts_model.size(); ++i ) {
        const Eigen::Vector3d& ptm = pts_model.at(i);
        pcl::PointXYZ pt (ptm[0],ptm[1],ptm[2]);
        model_cloud->push_back (pt);
    }

    auto* kd_tree = new pcl::KdTreeFLANN <pcl::PointXYZ>();
    kd_tree->setInputCloud ( model_cloud );

    // used for search.
    std::vector<int> index (1);
    std::vector<float> squared_distance (1);

    // Dth
    double squared_distance_th = std::numeric_limits <double>::max();
    double cur_squared_dist;
    double last_squared_dist = std::numeric_limits<double>::max();

    //for n_iters
    for ( int n = 0; n < max_iteration; n ++ ) {
        // clear two point clouds.
        pts_cloud_matched.clear();
        pts_model_matched.clear();
        ///01 construct aligned points
        double sum_squared_dist = 0.0;
        for (size_t i = 0; i < pts_cloud.size(); i += step) {
            // transformed by T
            Eigen::Vector3d pt = R * pts_cloud.at(i) + t;///
            // find the nearest points by knn
            pcl::PointXYZ pt_d(pt[0], pt[1], pt[2]);
            if (!kd_tree->nearestKSearch (pt_d,1,index,squared_distance)) {
                std::cerr << "ERROR: no points found.\n";
                return;
            }

            if(squared_distance[0] < squared_distance_th) {
                // add squared distance.
                sum_squared_dist += squared_distance[0];
                // add the pt in cloud.
                pts_cloud_matched.push_back(pts_cloud.at(i));
                // add the pt in model.
                pts_model_matched.push_back(pts_model.at(index[0]));
            }
        } // for all pts_cloud

        ///02 get R & t
        // step 2.1 find center of model(X) and cloud(P)
        Eigen::Vector3d mu_x(0.0, 0.0, 0.0);
        Eigen::Vector3d mu_p(0.0, 0.0, 0.0);
        for(size_t i = 0; i < pts_cloud_matched.size(); i++){
            mu_x += pts_model_matched.at(i);
            mu_p += pts_cloud_matched.at(i);
        }
        mu_x = mu_x / double(pts_model_matched.size());
        mu_p = mu_p / double(pts_cloud_matched.size());
        ///remove the center
        // step 2.2 Get W.
        Eigen::Matrix3d W;
        for(size_t i = 0; i < pts_cloud_matched.size(); i++) {
            W += (pts_model_matched.at(i)-mu_x) * ((pts_cloud_matched.at(i)-mu_p).transpose());
        }

        // step 2.3 Get R
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * (svd.matrixV().transpose());
        // step 2.4 Get t
        t  = mu_x - R * mu_p;

        ///check if convergence
        cur_squared_dist = sum_squared_dist / (double)pts_cloud_matched.size();
        double squared_dist_change = last_squared_dist -cur_squared_dist;

        if(squared_dist_change < epsilon || cur_squared_dist < min_err2) {break;}
        last_squared_dist = cur_squared_dist;
        squared_distance_th = factor * cur_squared_dist;
    }
    delete kd_tree;
}
void icpAlgorithm ( const std::vector<Eigen::Vector3d>& pts_tgt,
                    const std::vector<Eigen::Vector3d>& pts_src,
                    Eigen::Matrix3d& wr, Eigen::Vector3d& wt )
{
    Eigen::Vector3d center_tgt = {0,0,0};
    Eigen::Vector3d center_src = {0,0,0};

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Eigen::Vector3d> closestPoints_src;
    std::vector<Eigen::Vector3d> closestPoints_tgt;

    for (const auto & i : pts_tgt) {
        double x = i(0);
        double y = i(1);
        double z = i(2);
        cloud_tgt->push_back(pcl::PointXYZ(x,y,z));
    }

    auto* kd_tree = new pcl::KdTreeFLANN<pcl::PointXYZ>();
    int k = 1;
    std::vector<int> index(k);
    std::vector<float> squareDist(k);
    kd_tree->setInputCloud(cloud_tgt);

    Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_ = {0,0,0};

    int count = 0;
    for (int n = 0;n < max_iteration; ++n) {
        closestPoints_tgt.clear();
        closestPoints_src.clear();
        wr = R_ * wr;
        wt = wr * wt + t_;
        for (size_t i = 0; i < pts_src.size(); ++i) {//for pts_src,find closest points in pts_tgt
            Eigen::Vector3d pt = wr * pts_src.at(i) + wt;
            pcl::PointXYZ ptt(pt[0],pt[1],pt[2]);
            if (!kd_tree->nearestKSearch(ptt,k,index,squareDist)){
                cout<<"Can't find the relative point in pts_tgt!"<<endl;
                cout<<"index: "<< i <<endl;
                return;
            }
            if(squareDist[0]<minDist){
                //cout<<"squareDist"<<" "<<i<<": "<<squareDist[0]<<endl;
                closestPoints_tgt.push_back(pts_tgt.at(index[0]));
                closestPoints_src.emplace_back(pt[0],pt[1],pt[2]);
            }
        }

        size_t cloudSize;
        cloudSize = closestPoints_src.size() < pts_tgt.size() ? closestPoints_src.size() : pts_tgt.size();
        for (size_t i = 0; i < cloudSize; i++) {
            center_tgt += closestPoints_tgt[i];
            center_src += closestPoints_src[i];
        }
        center_tgt = center_tgt / cloudSize;
        center_src = center_src / cloudSize;
        //remove the center
        std::vector<Eigen::Vector3d> removed_tgt(cloudSize);
        std::vector<Eigen::Vector3d> removed_src(cloudSize);
        for (size_t i = 0; i < cloudSize; i++) {
            removed_tgt[i] = closestPoints_tgt[i] - center_tgt;
            removed_src[i] = closestPoints_src[i] - center_src;
        }
        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < cloudSize; ++i) {
            W += Eigen::Vector3d(removed_tgt[i](0),removed_tgt[i](1),removed_tgt[i](2)) *
                 Eigen::Vector3d(removed_src[i](0),removed_src[i](1),removed_src[i](2)).transpose();
        }
        //svd on W
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix3d& U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        R_ = U * (V.transpose());
        t_ = center_tgt - R_ * center_src;
        if (std::isnan(t_[0]) || std::isinf(t_[0]) ||
            std::isnan(t_[1]) || std::isinf(t_[1]) ||
            std::isnan(t_[2]) || std::isinf(t_[2])) {
            std::cout<<"iteration:"<<n<<std::endl;
            break;
        }
        ///check if convergence
        size_t size = removed_tgt.size();
        double dist = 0.0;
        for (size_t i = 0; i < size; ++i) {
            Eigen::Vector3d check_src = R_ * closestPoints_src.at(i) + t_;
            Eigen::Vector3d check_tgt = closestPoints_tgt.at(i) - check_src;
            dist += sqrt(pow(check_tgt[0],2) + pow(check_tgt[1],2) + pow(check_tgt[2],2));
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
    delete kd_tree;
}

void icpAlgorithm2D ( const std::vector<Eigen::Vector2d>& pts_tgt,
                    const std::vector<Eigen::Vector2d>& pts_src,
                    Eigen::Matrix2d& wr, Eigen::Vector2d& wt )
{
    Eigen::Vector2d center_tgt = {0,0};
    Eigen::Vector2d center_src = {0,0};

    Nabo::NNSearchD* nnSearchD = nullptr;
    std::vector<Eigen::Vector2d> closestPoints_src;
    std::vector<Eigen::Vector2d> closestPoints_tgt;
    std::vector<Eigen::Vector3d> aa;


    Eigen::MatrixXd kdTreeData;
    kdTreeData.resize(2,pts_tgt.size());
    for (int i = 0; i < pts_tgt.size(); ++i) {
        kdTreeData(0,i) = pts_tgt[i](0);
        kdTreeData(1,i) = pts_tgt[i](1);
        //kdTreeData(2,i) = i;
    }
    nnSearchD = Nabo::NNSearchD::createKDTreeLinearHeap(kdTreeData,2);

    int k = 1;
    Eigen::VectorXi index(k);
    Eigen::VectorXd squareDist(k);

    Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t_ = {0,0};

    int count = 0;
    for (int n = 0;n < max_iteration; ++n) {
        closestPoints_tgt.clear();
        closestPoints_src.clear();
        wr = R_ * wr;
        wt = wr * wt + t_;
        for (size_t i = 0; i < pts_src.size(); ++i) {//for pts_src,find closest points in pts_tgt
            Eigen::Vector2d pt = wr * pts_src.at(i) + wt;
            nnSearchD->knn(pt,index,squareDist,k,0,0,2);
            if(squareDist[0]<minDist && squareDist[0] < numeric_limits<double>::infinity() && !std::isinf(squareDist[0])){
                if (!kdTreeData.col(index[0]).isZero()) {
                    closestPoints_tgt.emplace_back(kdTreeData.col(index[0]));
                    closestPoints_src.emplace_back(pt[0],pt[1]);
                } else {
                    cout<<"Can't find the relative point in pts_tgt!"<<endl;
                    cout<<"index: "<< i <<endl;
                    continue;
                }
            }
        }

        size_t cloudSize;
        cloudSize = closestPoints_src.size() < pts_tgt.size() ? closestPoints_src.size() : pts_tgt.size();
        for (size_t i = 0; i < cloudSize; i++) {
            center_tgt += closestPoints_tgt[i];
            center_src += closestPoints_src[i];
        }
        center_tgt = center_tgt / cloudSize;
        center_src = center_src / cloudSize;
        //remove the center
        std::vector<Eigen::Vector2d> removed_tgt(cloudSize);
        std::vector<Eigen::Vector2d> removed_src(cloudSize);
        for (size_t i = 0; i < cloudSize; i++) {
            removed_tgt[i] = closestPoints_tgt[i] - center_tgt;
            removed_src[i] = closestPoints_src[i] - center_src;
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
            Eigen::Vector2d check_src = R_ * closestPoints_src.at(i) + t_;
            Eigen::Vector2d check_tgt = closestPoints_tgt.at(i) - check_src;
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
    delete nnSearchD;
}

static Eigen::Vector2d computeNormal(std::vector<Eigen::Vector2d> &nearPoints) {
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
    normal = eigenVectors.col(0);
    return normal;
}

void histogram(std::vector<double>& data,std::vector<double>& H,double min,double max,int n){
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
        //std::cout<<"Ni: "<<i<<" num: "<<N[i]<<std::endl;
        His[i] = N[i] / (N.size() + 1);
    }
    H = His;
}

void min_maxNormalization(std::vector<double>& a,double oMin,double oMax,double nMin,double nMax) {
    for (double & i : a) {
        if (!std::isinf(i)) {
            i =nMin + (nMax - nMin) * (i - oMin) / (oMax - oMin);
        }
    }
}


void pairwiseCloud(const std::vector<Eigen::Vector2d>& scanPoints,
                         std::vector<Eigen::Vector2d>& ptsNormal,
                         std::vector<Eigen::Vector2d>& lp_lc,
                         Eigen::Vector4d& lpStats,
                         Eigen::Vector4d& lcStats) {
    boost::timer t;
    boost::timer t6;
    double t_other;
    double angle_increment = 0.0087145;//rad
    //construct kd-tree to find selected points
    Eigen::MatrixXd kdTreeData;
    kdTreeData.resize(2,scanPoints.size());
    for (int i = 0; i < scanPoints.size(); ++i) {
        kdTreeData(0,i) = scanPoints[i](0);
        kdTreeData(1,i) = scanPoints[i](1);
    }
    Nabo::NNSearchD* nnSearchD = nullptr;
    nnSearchD = Nabo::NNSearchD::createKDTreeLinearHeap(kdTreeData);
    int searchNum = 15;
    Eigen::VectorXi searchIndex(searchNum);
    Eigen::VectorXd searchDist(searchNum);

    int num = 10;
    Eigen::VectorXi index(num);
    Eigen::VectorXd dist(num);

    std::vector<double> vecLC;
    std::vector<double> vecLP;
    std::vector<double> vecLN;
    t_other=t6.elapsed();
    clock_t knns,knne;

    knns = clock();
    double tc,tn,tp,tknn;
    for (int i = 0; i < scanPoints.size(); ++i) {
        boost::timer t5;
        std::vector<Eigen::Vector2d> neighborNormal;//for pts surround point i
        nnSearchD->knn(scanPoints[i],searchIndex,searchDist,searchNum,0,Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::SORT_RESULTS,2);
        std::vector<Eigen::Vector2d> selectedPts;
        for (int j = 0; j < searchNum; ++j) {
            //remove useless points
            if (searchDist[j] < numeric_limits<double>::infinity() && !isinf(searchDist(j))) {
                selectedPts.emplace_back(kdTreeData.col(searchIndex[j]));
            } else {break;}
        }

        //remove points that are not adjacent
        double b = atan2(selectedPts[0](1),selectedPts[0](0));
        for (int j = selectedPts.size() - 1; j > 1; --j) {
            double a = atan2(selectedPts[j](1),selectedPts[j](0));
            if (abs(a - b) > 3*(j-1)*angle_increment) {
                selectedPts.erase(selectedPts.begin() + j);
            }
        }
tknn += t5.elapsed();
        double LPlanarity = 0;
        double LNormalDiff = 0;
        ///compute curvature
        double LCurvature = 0;
        boost::timer t1;
        std::deque<Eigen::Vector2d> sortedPts;
        sortedPts.push_back(selectedPts[0]);
        double angle0 = atan2(selectedPts[0](1),selectedPts[0](0));
        int count1 = 0;
        int count2 = 0;
        int count3 = 0;
        for (auto & selectedPt : selectedPts) {
            double angle1 = atan2(selectedPt(1),selectedPt(0));
            if (angle1 >= 0) {count1++;}
            if (angle1 < 0) {count2++;}
            if (abs(angle1) < M_PI / 4) {count3++;}
        }
/*
        if (count1 == selectedPts.size()) {
            for (int j = 1; j < selectedPts.size(); ++j) {
                double angle1 = atan2(selectedPts[i](1),selectedPts[i](0));
                double angle = angle1 - angle;
                if (angle > 0) {
                    sortedPts.push_back(selectedPts[j]);
                } else {
                    sortedPts.push_front(selectedPts[j]);
                }
            }
        } else if (count2 == selectedPts.size()) {
            for (int j = 1; j < selectedPts.size(); ++j) {
                double angle1 = atan2(selectedPts[i](1),selectedPts[i](0));
                double angle = angle1 - angle;
                if (angle > 0) {
                    sortedPts.push_back(selectedPts[j]);
                } else {
                    sortedPts.push_front(selectedPts[j]);
                }
            }
        } else if (count3 == selectedPts.size()) {
            for (int j = 1; j < selectedPts.size(); ++j) {
                double angle1 = atan2(selectedPts[i](1),selectedPts[i](0));
                double angle = angle1 - angle;
                if (angle > 0) {
                    sortedPts.push_back(selectedPts[j]);
                } else {
                    sortedPts.push_front(selectedPts[j]);
                }
            }
        }
        */
        if (count1 == selectedPts.size() ||
            count2 == selectedPts.size() ||
            count3 == selectedPts.size()) {
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
            if (scanPoints[i](0) == sortedPts[j](0)) {
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
/*
                    std::cout<<sortedPts[j-5](0)<<" "<<sortedPts[j-5](1)<<"\n"
                             <<sortedPts[j-4](0)<<" "<<sortedPts[j-4](1)<<"\n"
                             <<sortedPts[j-3](0)<<" "<<sortedPts[j-3](1)<<"\n"
                             <<sortedPts[j-2](0)<<" "<<sortedPts[j-2](1)<<"\n"
                             <<sortedPts[j-1](0)<<" "<<sortedPts[j-1](1)<<"\n"
                             <<sortedPts[j](0)<<" "<<sortedPts[j](1)<<"\n"
                             <<sortedPts[j+1](0)<<" "<<sortedPts[j+1](1)<<"\n"
                             <<sortedPts[j+2](0)<<" "<<sortedPts[j+2](1)<<"\n"
                             <<sortedPts[j+3](0)<<" "<<sortedPts[j+3](1)<<"\n"
                             <<sortedPts[j+4](0)<<" "<<sortedPts[j+4](1)<<"\n"
                             <<sortedPts[j+5](0)<<" "<<sortedPts[j+5](1)<<std::endl;
                             */
                    //std::cout<<"c:"<<LCurvature<<" "<<sortedPts[j](0)<<" "<<sortedPts[j](1)<<std::endl;
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
                    //std::cout<<"Abandon calculating CURVATURE!"<<std::endl;
                    LCurvature = std::numeric_limits<double>::infinity();
                    vecLC.push_back(LCurvature);
                    break;
                }
            }
        }
        tc += t1.elapsed();
        //std::cout<<"curvature_time: "<<t1.elapsed()<<"(s)"<<std::endl;

/*
        double diffX = 0;
        double diffY = 0;
        for (int j = 1; j < selectedPts.size(); ++j) {
            std::cout<<"pts for curvature: "<<selectedPts[j](0)<<" "<<selectedPts[j](1)<<std::endl;
            diffX += selectedPts[j](0) - selectedPts[0](0);
            diffY += selectedPts[j](1) - selectedPts[0](1);
        }
        LCurvature = diffX * diffX + diffY * diffY;
        vecLC.push_back(LCurvature);

        if (i >= 5 || i <= scanPoints.size() - 5) {
            double a0 = atan2(scanPoints[i](1),scanPoints[i](0));
            double a1 = atan2(scanPoints[i-5](1),scanPoints[i-5](0));
            double a2 = atan2(scanPoints[i+5](1),scanPoints[i+5](0));
            if (abs(a1-a0) < 15 * angle_increment && abs(a2-a0) < 15 * angle_increment) {

            }
        } else {
            LCurvature = std::numeric_limits<double>::infinity();
        }
*/


        ///compute normal for every selected points, or assume all the selected points have same normal
        boost::timer t2;

        Eigen::Vector2d normal;
        if (selectedPts.size() >= 10) {
            normal = computeNormal(selectedPts);
/*
            //compute normal for neighbor pts
            for (int j = 1; j < selectedPts.size(); ++j) {
                Eigen::Vector2d nor;
                std::vector<Eigen::Vector2d> neighborPts;
                nnSearchD->knn(selectedPts[j],index,dist,num,0,Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::SORT_RESULTS,2.5);
                for (int k = 0; k < searchNum; ++k) {
                    if (searchDist[k] < numeric_limits<double>::infinity() && !isinf(searchDist(k))) {
                        neighborPts.emplace_back(kdTreeData.col(searchIndex[k]));
                    } else {break;}
                }
                if (neighborPts.size() > 3) {
                    nor = computeNormal(neighborPts);
                    neighborNormal.push_back(nor);
                }
            }
*/
        } else {
            normal(0) = normal(1) = std::numeric_limits<double>::infinity();
        }

        ptsNormal.push_back(normal);

        ///compute normal difference (or assume all the selected points have same normal)
        /*
        if (!std::isinf(normal(0)) || !std::isinf(normal(1))) {
            double nor = 0;
            for (auto & neiNor : neighborNormal) {
                nor += neiNor.dot(normal);
            }
            LNormalDiff = 1 - nor / neighborNormal.size();
            vecLN.push_back(LNormalDiff);
            neighborNormal.clear();
        } else {
            LNormalDiff = std::numeric_limits<double>::infinity();
            vecLN.push_back(LNormalDiff);
            //continue;
        }
         */
        tn += t2.elapsed();
        ///compute planarity
        boost::timer t3;
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
            //continue;
        }
        tp += t3.elapsed();
        std::cout<<"i: "<<i<<" x: "<<scanPoints[i](0)<<" y: "<<scanPoints[i](1)
                 <<" normal: "<<normal(0)<<" "<<normal(1)
                 <<" LP: "<<LPlanarity<<" LC: "<<LCurvature<<std::endl;
        //std::cout<<"LP: "<<LPlanarity<<" N: "<<normal(0) <<" "<<normal(1)<<" LC: "<<LCurvature<<" x: "<<scanPoints[i](0)<<" y: "<<scanPoints[i](1)<<std::endl;
        lp_lc.emplace_back(LPlanarity,LCurvature);
    }
    delete nnSearchD;
    ///histogram of planarity,normal difference, curvature
    boost::timer t4;
    std::vector<double> hisLC;
    min_maxNormalization(vecLC,0,50,0,1);
    histogram(vecLC,hisLC,0,1,40);
    //for (int i = 0; i < hisLC.size(); ++i) {
    //    std::cout<<"LC i: "<<i<<" val: "<<hisLC[i]<<std::endl;
    //}

    std::vector<double> hisLP;
    min_maxNormalization(vecLP,-0.1,0.1,0,1);
    histogram(vecLP,hisLP,0,1,40);
    //for (int i = 0; i < hisLP.size(); ++i) {
    //    std::cout<<"LP i: "<<i<<" val: "<<hisLP[i]<<std::endl;
    //}

    std::vector<double> hisLN;
    min_maxNormalization(vecLN,-0.0001,0.0009,0,1);
    histogram(vecLN,hisLN,0,1,40);
    //for (int i = 0; i < hisLN.size(); ++i) {
    //    std::cout<<"LN i: "<<i<<" val: "<<hisLN[i]<<std::endl;
    //}

    ///compute mean/variance/energy/entropy
    //for planarity
    double LP_mean = 0,LP_var = 0,LP_e = 0,LP_entropy = 0;
    double lp_m = 0,lp_v = 0;

    //for curvature
    double LC_mean = 0,LC_var = 0,LC_e = 0,LC_entropy = 0;
    double lc_m = 0,lc_v = 0;

    for (int i = 0; i < scanPoints.size(); ++i) {
        if (!std::isinf(vecLP[i])) {
            lp_m += vecLP[i];
        }
        if (!std::isinf(vecLC[i])) {
            lc_m += vecLC[i];
        }
    }
    LP_mean = lp_m / (scanPoints.size() + 1);
    LC_mean = lc_m / (scanPoints.size() + 1);

    for (int i = 0; i < scanPoints.size(); ++i) {
        if (!std::isinf(vecLP[i])) {
            lp_v += pow(vecLP[i] - LP_mean,2);
        }
        if (!std::isinf(vecLC[i])) {
            lc_v += pow(vecLC[i] - LC_mean,2);
        }
    }
    LP_var = lp_v / (scanPoints.size() + 1);
    LC_var = lc_v / (scanPoints.size() + 1);

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

    lpStats << LP_mean,LP_var,LP_e,LP_entropy;
    lcStats << LC_mean,LC_var,LC_e,LC_entropy;
    std::cout<<"LP_mean: "<<LP_mean<<" LP_var: "<<LP_var<<" LP_e: "<<LP_e<<" LP_entropy: "<<LP_entropy<<std::endl;

    std::cout<<"LC_mean: "<<LC_mean<<" LC_var: "<<LC_var<<" LC_e: "<<LC_e<<" LC_entropy: "<<LC_entropy<<std::endl;
    knne = clock();
    std::cout<<"knn time: "<< (knne-knns)*1000/CLOCKS_PER_SEC<<"(ms)"<<std::endl;
    std::cout<<"t_total: "<< t.elapsed() << "(s)"<<std::endl;
    std::cout<<"tc:"<<tc<<std::endl;
    std::cout<<"tn:"<<tn<<std::endl;
    std::cout<<"tp:"<<tp<<std::endl;
    std::cout<<"tknn:"<<tknn<<std::endl;
    std::cout<<"th:"<<t4.elapsed()<<std::endl;
    std::cout<<"t_other:"<<t_other<<std::endl;

}

int main(int argc,char **argv){
    ros::init(argc,argv,"readpcd");
    ros::NodeHandle nh;
    ros::Publisher readPub = nh.advertise<sensor_msgs::PointCloud2>("pclOut",6-1);
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud01(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/sortpts04.pcd",*cloud01);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud02(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lee/catkinReadPCD_ws/sortpts04.pcd",*cloud02);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud03(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<Eigen::Vector3d> cloud_tgt;
    std::vector<Eigen::Vector3d> cloud_src;
    std::vector<Eigen::Vector2d> tgtPts;
    std::vector<Eigen::Vector2d> srcPts;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = {0.0,0.0,0.0};
    Eigen::Matrix3d R02 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t02 = {0.0,0.0,0.0};
    Eigen::Matrix2d R03 = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t03 = {0,0};
    for (int i = 0; i < cloud01->size(); ++i) {
        float x = cloud01->points[i].x;
        float y = cloud01->points[i].y;
        float z = cloud01->points[i].z;
        cloud_tgt.emplace_back(x,y,z);
        tgtPts.emplace_back(x,y);
    }
    double a = 0;
    Eigen::Matrix3d Rz;
    Rz << cos(a),-sin(a),0,
          sin(a), cos(a),0,
          0, 0, 1;
    for (int i = 0; i < cloud02->size(); ++i) {
        float x = cloud02->points[i].x;
        float y = cloud02->points[i].y;
        float z = cloud02->points[i].z;
        Eigen::Vector3d v = {x,y,z};
        v = Rz * v;
        cloud_src.emplace_back(v);
        srcPts.emplace_back(v[0],v[1]);
        cloud03->points.emplace_back(v[0],v[1],v[2]);
    }

    Eigen::Matrix4f T;
    pcl::PointCloud<pcl::PointXYZ> cloudAligned;
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(5);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-5);
    icp.setEuclideanFitnessEpsilon(0.1);
    icp.setInputSource(cloud03->makeShared());
    icp.setInputTarget(cloud01->makeShared());
    icp.align(cloudAligned);
    T = icp.getFinalTransformation();
    //std::cout<<"T:\n"<<T<<std::endl;

    std::vector<Eigen::Vector2d> ptsNormal;
    std::vector<Eigen::Vector2d> lp_lc;
    Eigen::Vector4d lpStats;
    Eigen::Vector4d lcStats;

    pairwiseCloud(tgtPts,ptsNormal,lp_lc,lpStats,lcStats);

    std::vector<Eigen::Vector2d> ptsNormal1;
    std::vector<Eigen::Vector2d> lp_lc1;
    Eigen::Vector4d lpStats1;
    Eigen::Vector4d lcStats1;
    pairwiseCloud(srcPts,ptsNormal1,lp_lc1,lpStats1,lcStats1);
    //findTransformation(cloud_tgt,cloud_src,500,1.0e-12,1.0e-12,R,t);
    //std::cout<<"\nRotation01:\n"<<R<<"\ntranslation01:\n"<<t<<std::endl;
    //std::cout<<"\nRotation_inv01:\n"<<R.inverse()<<std::endl;
    icpAlgorithm(cloud_tgt,cloud_src,R02,t02);
    std::cout<<"\nRotation02:\n"<<R02<<"\ntranslation02:\n"<<t02<<std::endl;
    std::cout<<"\nRotation_inv02:\n"<<R02.inverse()<<std::endl;
    icpAlgorithm2D(tgtPts,srcPts,R03,t03);
    std::cout<<"\nRotation03:\n"<<R03<<"\ntranslation03:\n"<<t03<<std::endl;



    pcl::toROSMsg(*cloud01,output);
    output.header.frame_id = "map";
    ros::Rate loop_rate(1);
    while (ros::ok()){
        readPub.publish(output);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}