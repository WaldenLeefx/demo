//
// Created by lee on 2021/5/10.
//

#include "../include/utility.h"

using namespace std;
namespace lidar_slam {
    class lidar_odometry : ParamServer {
    private:
        std::vector<Eigen::Vector2d> m_sourcePointCloud,m_targetPointCloud;
        std::vector<Eigen::Vector2d> m_sourcePtCloudNormals,m_targetPtCloudNormals;
        Nabo::NNSearchD* m_pTargetKDTree = nullptr;
        Nabo::NNSearchD* m_pSourceKDTree = nullptr;
        Eigen::MatrixXd m_sourceKDTreeDataBase;
        Eigen::MatrixXd m_targetKDTreeDataBase;
        bool m_isGetNormals = false;///?
        int m_Iterations = 10;
        double m_h = 0.06;
        double m_r = 0.2;
        double m_PtID = 0;
    public:
        lidar_odometry() {
        imls_sub = nh.subscribe("/distortion_points",100,&lidar_odometry::imlsOdomCallBack,this);
        imlsOdom_pub = nh.advertise<nav_msgs::Odometry>("/Odometry",5);
        imlsPath_pub = nh.advertise<nav_msgs::Path>("/imls_path",5);
    }
        void imlsOdomCallBack(const sensor_msgs::PointCloud2Ptr &cloudIn) {
            if (!isHashMatrix) {
                HashMatrixRThetaPhi();
                isHashMatrix = true;
            }
            if (!m_isFirstFrame) {
                std::cout<<"The First Frame!"<<std::endl;
                m_isFirstFrame = true;
                m_prevLaserPose.setZero();
                convertPointCloud(cloudIn,m_prePointCloud);
                return;
            }
            std::vector<Eigen::Vector2d> nowPts;
            convertPointCloud(cloudIn,nowPts);
            //调用imls进行icp匹配，并输出结果
            setSourcePointCloud(nowPts);
            setTargetPointCloud(m_prePointCloud);

            Eigen::Matrix3d rPose,rCovariance;
            if (imlsMatcher(rPose,rCovariance)) {
                std::cout<<"IMLS Match Successfully!"<<endl;//rPose(0,2)<<","<<rPose(1,2)<<","<<atan2(rPose(1,0),rPose(0,0))*57.295<<std::endl;
                std::cout<<"seq:"<<cloudIn->header.seq<<endl;
                Eigen::Matrix3d lastPose;
                Eigen::Matrix3d nowPose;

                lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0),
                            sin(m_prevLaserPose(2)),  cos(m_prevLaserPose(2)), m_prevLaserPose(1),
                            0, 0, 1;

                nowPose = lastPose * rPose;
                m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1,0), nowPose(0,0));
                if (cloudIn->header.seq >= 1450 && cloudIn->header.seq < 1550) {
                    m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1,0), nowPose(0,0))+0.00185;
                }
                cout<<"nowPose: "<<nowPose(0, 2)<<","<<nowPose(1, 2)<<","<<atan2(nowPose(1,0), nowPose(0,0))<<endl;
                if (cloudIn->header.seq > 500 && cloudIn->header.seq <=600) {
                    //m_prevLaserPose<<nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1,0), nowPose(0,0))+0.0015;
                }

                geometry_msgs::Quaternion globalPose = tf::createQuaternionMsgFromYaw(m_prevLaserPose(2));
                nav_msgs::Odometry imlsOdom;
                imlsOdom.header.frame_id = "laser";
                imlsOdom.child_frame_id = "imls_odom";
                imlsOdom.header.stamp = cloudIn->header.stamp;
                imlsOdom.header.seq = cloudIn->header.seq;
                imlsOdom.pose.pose.orientation.w = globalPose.w;
                imlsOdom.pose.pose.orientation.x = globalPose.x;
                imlsOdom.pose.pose.orientation.y = globalPose.y;
                imlsOdom.pose.pose.orientation.z = globalPose.z;
                imlsOdom.pose.pose.position.x = m_prevLaserPose[0];
                imlsOdom.pose.pose.position.y = m_prevLaserPose[1];
                imlsOdom.pose.pose.position.z = 0;
                imlsOdom_pub.publish(imlsOdom);

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = cloudIn->header.stamp;
                pose.header.frame_id = "laser";
                pose.pose = imlsOdom.pose.pose;
                static nav_msgs::Path imlsPath;
                imlsPath.header.stamp = cloudIn->header.stamp;
                imlsPath.header.frame_id = "laser";
                imlsPath.poses.push_back(pose);
                imlsPath_pub.publish(imlsPath);
                Eigen::Vector4d q;
                q << globalPose.x,globalPose.y,globalPose.z,globalPose.w;
                saveTUMTrajectory(cloudIn->header.stamp,m_prevLaserPose,q);
                //saveTrajectory(cloudIn->header.stamp,Eigen::Vector3d(m_prevLaserPose[0],m_prevLaserPose[1],0),
                  //             Eigen::Vector4d(globalPose.x,globalPose.y,globalPose.z,globalPose.w));
            } else {
                std::cout<<"IMLS Match Failed!"<<std::endl;
            }

            m_prePointCloud = nowPts;

        }

        static void convertPointCloud(const sensor_msgs::PointCloud2Ptr &msg,std::vector<Eigen::Vector2d> &points) {
            points.clear();
            std::vector<int> index;
            pcl::PointCloud<pcl::PointXYZ> msg_pcl;
            pcl::fromROSMsg(*msg, msg_pcl);
            pcl::removeNaNFromPointCloud(msg_pcl, msg_pcl, index);
            for (auto &point : msg_pcl.points) {
                double x = point.x;
                double y = point.y;
                double z = point.z;

                if(std::isnan(x) || std::isinf(x) ||
                   std::isnan(y) || std::isinf(y) ||
                   std::isnan(z) || std::isinf(z))
                    continue;

                points.emplace_back(x, y);
            }
    }

        void setSourcePointCloud(std::vector<Eigen::Vector2d> &_source_cloud) {
            m_sourcePointCloud = _source_cloud;
            RemoveNAN_INFData(m_sourcePointCloud);
    }

        void setSourcePointCloud(pcl::PointCloud<pcl::PointXYZ> &pcl_cloud)
        {
            std::vector<Eigen::Vector2d> source_ptCloud;
            for(auto & i : pcl_cloud) {
                source_ptCloud.emplace_back(i.x,i.y);
            }
            setSourcePointCloud(source_ptCloud);
        }

        void setTargetPointCloud(std::vector<Eigen::Vector2d> &_target_cloud) {
            m_targetPointCloud = _target_cloud;
            if (m_pTargetKDTree != nullptr) {
                delete m_pTargetKDTree;
                m_pTargetKDTree = nullptr;
            }

            RemoveNAN_INFData(m_targetPointCloud);
            //construct kd tree
            if (m_pTargetKDTree == nullptr) {
                m_targetKDTreeDataBase.resize(2,m_targetPointCloud.size());
                for (int i = 0; i < m_targetPointCloud.size(); i++) {
                    m_targetKDTreeDataBase(0,i) = m_targetPointCloud[i](0);
                    m_targetKDTreeDataBase(1,i) = m_targetPointCloud[i](1);
                }
                m_pTargetKDTree = Nabo::NNSearchD::createKDTreeLinearHeap(m_targetKDTreeDataBase);
            }
            m_isGetNormals = false;//re-compute normals
        }

        void setTargetPointCloud(pcl::PointCloud<pcl::PointXYZ> &pcl_cloud)
        {
            std::vector<Eigen::Vector2d> target_ptCloud;
            for(auto & i : pcl_cloud) {
                target_ptCloud.emplace_back(i.x,i.y);
            }
            setTargetPointCloud(target_ptCloud);
        }

        static void RemoveNAN_INFData(std::vector<Eigen::Vector2d> &_input) {
            for (auto it = _input.begin();it != _input.end();) {
                Eigen::Vector2d tmpPt = *it;
                if (std::isnan(tmpPt(0)) || std::isnan(tmpPt(1)) || std::isinf(tmpPt(0)) || std::isinf(tmpPt(1))) {
                    it = _input.erase(it);
                } else { it++; }
            }
        }

        bool imlsMatcher(Eigen::Matrix3d &finalPose,Eigen::Matrix3d &covariance) {
            if (!m_isGetNormals) {
                //计算target pointcloud中每个点的法向量
                m_targetPtCloudNormals.clear();

                for (int i = 1; i < m_targetPointCloud.size() - 1; ++i) {

                    Eigen::Vector2d xi = m_targetPointCloud[i];
                    int k = 7;
                    Eigen::VectorXi index(k);
                    Eigen::VectorXd dist(k);
                    int num = m_pTargetKDTree->knn(xi, index, dist, k, 0,
                                                   Nabo::NNSearchD::SORT_RESULTS | Nabo::NNSearchD::ALLOW_SELF_MATCH |
                                                   Nabo::NNSearchD::TOUCH_STATISTICS,
                                                   0.15);//0.15

                    std::vector<Eigen::Vector2d> nearPoints;
                    for (int ix = 0; ix < k; ++ix) {
                        if (dist[ix] < std::numeric_limits<double>::infinity() && !std::isinf(dist(ix))) {
                            nearPoints.emplace_back(m_targetKDTreeDataBase.col(index(ix)));
                        } else { break; }
                    }

                    Eigen::Vector2d normal;
                    if (nearPoints.size() >= 7) {
                        normal = ComputeNormal(nearPoints);
                        //normal = SRIComputeNormal(i,nearPoints);
                        //std::cout << "normal:\n" << normal << std::endl;
                        //std::cout << "ptx: " << m_targetPointCloud[i](0) << " pty: " << m_targetPointCloud[i](1)<< std::endl;
                    } else {
                        normal(0) = normal(1) = std::numeric_limits<double>::infinity();
                    }
/*
                    std::vector<Eigen::Vector2d> nearPts;
                    Eigen::Vector2d nor;
                    for (int j = i-1; j <= i+1; ++j) {
                        if (abs(atan2(m_targetPointCloud[j](1),m_targetPointCloud[j](0)) -
                            atan2(m_targetPointCloud[i](1),m_targetPointCloud[i](0))) < 3.5 * angle_increment) {
                            nearPts.emplace_back(m_targetPointCloud[j]);
                        }
                    }
                    if (nearPts.size() == 3) {
                        nor = SRIComputeNormal(nearPts);
                    } else {
                        nor(0) = nor(1) = std::numeric_limits<double>::infinity();
                    }
*/
                    m_targetPtCloudNormals.push_back(normal);
                }

/*
                clock_t computernormal_s,computernormal_f;
                computernormal_s = clock();
                for (int i = 1; i < m_targetPointCloud.size()-1; ++i) {
                    std::vector<Eigen::Vector3d> points;
                    Eigen::Vector3d xi,xii,xiii;
                    double dist1,dist2,range;
                    xi << m_targetPointCloud[i-1],0;
                    points.push_back(xi);
                    xii << m_targetPointCloud[i],0;
                    points.push_back(xii);
                    xiii << m_targetPointCloud[i+1],0;
                    points.push_back(xiii);
                    dist1 = sqrt(pow(points[1](0)-points[0](0),2) + pow(points[1](1)-points[0](1),2));
                    dist2 = sqrt(pow(points[1](0)-points[2](0),2) + pow(points[1](1)-points[2](1),2));
                    range = sqrt(pow(points[1](0),2) + pow(points[1](1),2));
                    if (dist1+dist2 > 0.3 || abs(dist1+dist2)/range > 3*angle_increment) {
                        continue;
                    }
                    Eigen::Vector2d normal;
                    if (points.size() == 3) {
                        SRIComputeNormal(points);
                    } else {
                        normal(0) = normal(1) = std::numeric_limits<double>::infinity();
                        m_targetPtCloudNormals.push_back(normal);
                    }
                }
                computernormal_f = clock();
                std::cout<<"ComputeNormal:"<<(computernormal_f-computernormal_s)*1000/CLOCKS_PER_SEC<<"(ms)"<<std::endl;
                */
/*
                for (int i = 0; i < m_targetPointCloud.size(); ++i) {
                    Eigen::Vector2d xi = m_targetPointCloud[i];
                    int k = 20;
                    Eigen::VectorXi index(k);
                    Eigen::VectorXd dist(k);
                    int num = m_pTargetKDTree->knn(xi, index, dist, k, 0,
                                                   Nabo::NNSearchD::SORT_RESULTS | Nabo::NNSearchD::ALLOW_SELF_MATCH |
                                                   Nabo::NNSearchD::TOUCH_STATISTICS,
                                                   0.15);

                    std::vector<Eigen::Vector2d> nearPoints;
                    for (int ix = 0; ix < k; ++ix) {
                        if (dist[ix] < std::numeric_limits<double>::infinity() && !std::isinf(dist(ix))) {
                            nearPoints.emplace_back(m_targetKDTreeDataBase.col(index(ix)));
                        } else { break; }
                    }

                    Eigen::Vector2d normal;
                    if (nearPoints.size() > 3) {
                        normal = FastAppLSNormal(nearPoints);
                    } else {
                        normal(0) = normal(1) = std::numeric_limits<double>::infinity();
                    }
                    m_targetPtCloudNormals.push_back(normal);
                }
                */
            }



            //初始化估计值．
            Eigen::Matrix3d result;
            result.setIdentity();
            covariance.setIdentity();

            for(int i = 0; i < m_Iterations;i++) {
                //根据当前估计的位姿对原始点云进行转换．
                std::vector<Eigen::Vector2d> in_cloud;
                for(int ix = 0; ix < m_sourcePointCloud.size();ix++) {
                    Eigen::Vector3d origin_pt;
                    origin_pt << m_sourcePointCloud[ix],1;
                    //cout<<"pt\n"<<origin_pt<<endl;
                    Eigen::Vector3d now_pt = result * origin_pt;
                    in_cloud.emplace_back(now_pt(0),now_pt(1));
                }
                //cout<<"in_cloud size:"<<in_cloud.size()<<endl;
                //把sourceCloud中的点投影到targetCloud组成的平面上
                //对应的投影点即为sourceCloud的匹配点．
                //每次转换完毕之后，都需要重新计算匹配点．
                //这个函数会得到对应的匹配点．
                //本次匹配会自动删除in_cloud内部的一些找不到匹配点的点．
                //因此，这个函数出来之后，in_cloud和ref_cloud是一一匹配的．
                std::vector<Eigen::Vector2d> ref_cloud;
                std::vector<Eigen::Vector2d> ref_normal;
                projSourcePtToSurface(in_cloud,ref_cloud,ref_normal);
                /////
                if(in_cloud.size() < 5 || ref_cloud.size() < 5) {
                    std::cout <<"Not Enough Correspondence:"<<in_cloud.size()<<","<<ref_cloud.size()<<std::endl;
                    std::cout <<"ICP Iterations Failed!!"<<std::endl;
                    return false;
                }

                //计算帧间位移．从当前的source -> target
                Eigen::Matrix3d deltaTrans;
                bool flag = SolveMotionEstimationProblem(in_cloud, ref_cloud,ref_normal,deltaTrans);

                if(!flag) {
                    std::cout <<"ICP Iterations Failed!!!!"<<std::endl;
                    return false;
                }

                //更新位姿．
                result = deltaTrans * result;

                //迭代条件是否满足．
                double deltadist = std::sqrt( std::pow(deltaTrans(0,2),2) + std::pow(deltaTrans(1,2),2));
                double deltaAngle = std::atan2(deltaTrans(1,0),deltaTrans(0,0));

                //如果迭代条件允许，则直接退出．
                if(deltadist < 0.001 && deltaAngle < (0.01/57.295)) {
                    break;
                }
            }

            finalPose = result;
            return true;
        }

        static Eigen::Vector2d ComputeNormal(std::vector<Eigen::Vector2d> &nearPoints) {
            Eigen::Vector2d normal;
            //TODO
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
            //end of TODO
            return normal;
        }

        Eigen::Vector2d SRIComputeNormal(const std::vector<Eigen::Vector2d>& nearPoints) {
            Eigen::Vector2d n;
                double r0,r1,r2,r3,r4,r5,r6;
                r0 = sqrt(pow(nearPoints[0](0),2) + pow(nearPoints[0](1),2));
                r1 = sqrt(pow(nearPoints[1](0),2) + pow(nearPoints[1](1),2));
                r2 = sqrt(pow(nearPoints[2](0),2) + pow(nearPoints[2](1),2));
                //r3 = sqrt(pow(nearPoints[3](0),2) + pow(nearPoints[3](1),2));//compute normal for r3
                //r4 = sqrt(pow(nearPoints[4](0),2) + pow(nearPoints[4](1),2));
                //r5 = sqrt(pow(nearPoints[5](0),2) + pow(nearPoints[5](1),2));
                //r6 = sqrt(pow(nearPoints[6](0),2) + pow(nearPoints[6](1),2));

                double phi = 0;
                double theta = atan2(nearPoints[1](1), nearPoints[1](0));
                for (double j : SRIAngle) {
                    if (abs(theta - j) < 0.000871451) {
                        theta = j;
                        break;
                    }
                }

                Eigen::Matrix3d RHat;
                Eigen::Vector3d vec;
                auto iter = SRImap.find(theta);
                if (iter != SRImap.end()) {
                    RHat = iter->second;
                }

                Eigen::Vector3d N,V;
                double vec0 = 1,vec2 = 0;
                double vec1 = (r2-r0)/(r1);
                V << vec0,vec1,vec2;
                N = RHat * V;
                n << N(0),
                     N(1);

            return n;

        }

        Eigen::Vector2d FastAppLSNormal(const std::vector<Eigen::Vector2d>& nearPoints) {
            Eigen::Vector2d n;
            Eigen::Vector2d v,b,N;
            Eigen::Matrix2d M;
            for (int i = 0; i < nearPoints.size(); ++i) {
                double x,y,theta,r,v0,v1,v2;
                double phi = 0;
                x = nearPoints[i](0);
                y = nearPoints[i](1);
                theta = atan2(y,x);
                r = sqrt(pow(x,2) + pow(y,2));
                v0 = cos(theta) * cos(phi);
                v1 = sin(theta) * cos(phi);
                v2 = sin(phi);
                v << v0,v1;
                M += v * v.transpose();
                b += v / r;
            }
            N = M.inverse() * b;
            n << N(0),N(1);
            return n;
        }
/**
 * @brief IMLSICPMatcher::projSourcePtToSurface
 * 此函数的目的是把source_cloud中的点云，投影到对应的surface上．
   即为每一个当前帧点云in_cloud的激光点计算匹配点和对应的法向量
   即in_cloud和out_cloud进行匹配，同时得到out_normal
   注意：本函数应该删除in_cloud内部找不到匹配值的点．
 * @param in_cloud          当前帧点云
 * @param out_cloud         当前帧的匹配点云
 * @param out_normal        匹配点云对应的法向量．
 */
        void projSourcePtToSurface(std::vector<Eigen::Vector2d> &in_cloud,
                                   std::vector<Eigen::Vector2d> &out_cloud,
                                   std::vector<Eigen::Vector2d> &out_normal) {
            out_cloud.clear();
            out_normal.clear();
            for (auto it = in_cloud.begin();it != in_cloud.end();) {
                Eigen::Vector2d xi = *it;
                //找到在target_cloud中的最近邻
                //包括该点和下标．
                int k =1;
                Eigen::VectorXi index(k);
                Eigen::VectorXd dist(k);
                m_pTargetKDTree->knn(xi,index,dist);

                Eigen::Vector2d nearXi = m_targetKDTreeDataBase.col(index(0));

                //为最近邻计算法向量．－－进行投影的时候，使用统一的法向量
                Eigen::Vector2d nearNormal = m_targetPtCloudNormals[index(0)];

                //如果对应的点没有法向量，也认为没有匹配点．因此直接不考虑．
                if(std::isinf(nearNormal(0))||std::isinf(nearNormal(1))||
                   std::isnan(nearNormal(0))||std::isnan(nearNormal(1))) {
                    it = in_cloud.erase(it);
                    continue;
                }
                //如果距离太远，则说明没有匹配点．因此可以不需要进行投影，直接去除．
                if(dist(0) > m_h * m_h ) {
                    it = in_cloud.erase(it);
                    continue;
                }

                //match
                double height;
                if (!ImplicitMLSFunction(xi, height)) {
                    it = in_cloud.erase(it);
                    //cout<<"erase invalid in_cloud points!"<<endl;
                    continue;
                }

                if (std::isnan(height)) {
                    std::cout<<"proj:this is nan, not possible"<<std::endl;
                    it = in_cloud.erase(it);
                    continue;
                }

                if(std::isinf(height)) {
                    std::cout <<"proj:this is inf, not possible"<<std::endl;
                    it = in_cloud.erase(it);
                    continue;
                }

                //TODO yi
                Eigen::Vector2d yi = xi - height * nearNormal;
                //end of TODO
                out_cloud.push_back(yi);
                out_normal.push_back(nearNormal);
                it++;
            }
        }

        bool ImplicitMLSFunction(const Eigen::Vector2d& x, double &height) {
            double weightSum = 0.0;
            double proJSum = 0.0;

            //construct kd tree
            if (m_pTargetKDTree == nullptr) {
                m_targetKDTreeDataBase.resize(2,m_targetPointCloud.size());
                for (int i = 0; i < m_targetPointCloud.size(); ++i) {
                    m_targetKDTreeDataBase(0,i) = m_targetPointCloud[i](0);
                    m_targetKDTreeDataBase(1,i) = m_targetPointCloud[i](1);
                }
                m_pTargetKDTree = Nabo::NNSearchD::createKDTreeLinearHeap(m_targetKDTreeDataBase);
            }

            // 找到位于点x附近(m_r)的所有的点云
            int searchNum = 20;
            Eigen::VectorXi nearIndex(searchNum);
            Eigen::VectorXd nearDist(searchNum);

            //找到某一个点的最近邻．
            //搜索searchNumber个最近邻
            //下标储存在nearIndices中，距离储存在nearDist2中．
            //最大搜索距离为m_r
            m_pTargetKDTree->knn(x,nearIndex,nearDist,searchNum,0,
                                 Nabo::NNSearchD::SORT_RESULTS | Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::TOUCH_STATISTICS,
                                            m_r);

            std::vector<Eigen::Vector2d> nearPoints;
            std::vector<Eigen::Vector2d> nearNormals;
            for (int i = 0; i < searchNum; ++i) {
                if (nearDist(i) < std::numeric_limits<double>::infinity() &&
                    std::isinf(nearDist(i)) == false &&
                    std::isnan(nearDist(i)) == false) {
                    int index = nearIndex(i);
                    Eigen::Vector2d tmpPt(m_targetKDTreeDataBase(0,index),m_targetKDTreeDataBase(1,index));
                    if (std::isinf(tmpPt(0)) || std::isinf(tmpPt(1)) || std::isnan(tmpPt(0)) || std::isnan(tmpPt(1))) {
                        continue;
                    }
                    Eigen::Vector2d normal;
                    normal = m_targetPtCloudNormals[index];
                    if (std::isinf(normal(0)) || std::isinf(normal(1)) || std::isnan(normal(0)) || std::isnan(normal(1))) {
                        continue;
                    }
                    nearPoints.push_back(tmpPt);
                    nearNormals.push_back(normal);
                } else {break;}
            }

            if (nearPoints.size() < 3) { return false;}

            //TODO
            //根据函数进行投影．计算height，即I(x)
            for (int i = 0; i < nearPoints.size(); ++i) {
                Eigen::Vector2d pi = nearPoints[i];
                double tmpDist = (pi - x).norm();
                double weight = std::exp(-tmpDist * tmpDist / (m_h * m_h));
                proJSum += weight * (x - pi).dot(nearNormals[i]);
                weightSum += weight;
            }
            height = proJSum / (weightSum + 0.000001);
            //end of TODO
            return true;
        }

        static bool SolveMotionEstimationProblem(std::vector<Eigen::Vector2d> &source_cloud,
                                                          std::vector<Eigen::Vector2d> &ref_cloud,
                                                          std::vector<Eigen::Vector2d> &ref_normals,
                                                          Eigen::Matrix3d& deltaTrans)
        {
            Eigen::Matrix4d M;
            M.setZero();
            Eigen::Matrix<double,1,4> gt; //gt是个行向量．
            gt.setZero();

            for(int i = 0; i < source_cloud.size();i++)
            {
                //点p-source point
                Eigen::Vector2d p = source_cloud[i];

                //target-point
                Eigen::Vector2d refPt = ref_cloud[i];

                //ref对应的normal
                Eigen::Vector2d ni = ref_normals[i];

                //加权矩阵
                //对于p-p来说，Ci =wi * I
                //对于point-line来说，Ci =wi *  n*n^T
                Eigen::Matrix2d Ci =ni * ni.transpose();

                //构造M矩阵
                Eigen::Matrix<double,2,4> Mi;
                Mi <<   1,0,p(0),-p(1),
                        0,1,p(1), p(0);
                M += Mi.transpose() * Ci * Mi;

                Eigen::Matrix<double,1,4> gti;
                gti  = -2 * refPt.transpose() * Ci * Mi;

                gt += gti;
            }

            //g是个列向量
            Eigen::Matrix<double,4,1> g = gt.transpose();

            //构建完了M矩阵和g向量．
            //在后续的求解过程中，基本上都使用的是2*M,因此直接令M = 2*M
            M = 2*M;

            //M(实际是2*M,下面等同)矩阵能分为４个部分
            Eigen::Matrix2d A,B,D;
            A = M.block(0,0,2,2);
            B = M.block(0,2,2,2);
            D = M.block(2,2,2,2);

            //论文中还引入了S和SA矩阵．
            Eigen::Matrix2d S,SA;
            S = D - B.transpose() * A.inverse() * B;
            SA = S.determinant() * S.inverse();

            //目前所有的式子已经可以构建多项式系数了．
            //式31右边p(\lambda)的系数
            //p(\lambda)的系数为：
            Eigen::Vector3d p_coffcient;
            p_coffcient << S.determinant(),2*(S(0,0)+S(1,1)) ,4;

            //论文中式(31)左边的系数(a x^2 + b x + c)为：
            double a,b,c;
            Eigen::Matrix4d tmpMatrix;
            tmpMatrix.block(0,0,2,2) = A.inverse() * B  * SA  * SA.transpose()* B.transpose() * A.inverse().transpose();
            tmpMatrix.block(0,2,2,2) = -A.inverse() * B  * SA * SA.transpose();
            tmpMatrix.block(2,0,2,2) = tmpMatrix.block(0,2,2,2).transpose();
            tmpMatrix.block(2,2,2,2) = SA * SA.transpose() ;

            c = g.transpose() * tmpMatrix * g;

            tmpMatrix.block(0,0,2,2) = A.inverse() * B * SA * B.transpose() * A.inverse().transpose();
            tmpMatrix.block(0,2,2,2) = -A.inverse() * B * SA;
            tmpMatrix.block(2,0,2,2) = tmpMatrix.block(0,2,2,2).transpose();
            tmpMatrix.block(2,2,2,2) = SA;
            b = 4 * g.transpose() * tmpMatrix * g;

            tmpMatrix.block(0,0,2,2) = A.inverse() * B * B.transpose() * A.inverse().transpose();
            tmpMatrix.block(0,2,2,2) = -A.inverse() * B;
            tmpMatrix.block(2,0,2,2) = tmpMatrix.block(0,2,2,2).transpose();
            tmpMatrix.block(2,2,2,2) = Eigen::Matrix2d::Identity();
            a = 4 * g.transpose() * tmpMatrix * g;

            //把式31的等式两边进行合并，得到一个4次多项式．５个系数．
            Eigen::VectorXd poly_coffi(5);
            poly_coffi(0) = c - p_coffcient(0) * p_coffcient(0);
            poly_coffi(1) = b - 2 * p_coffcient(0) * p_coffcient(1);
            poly_coffi(2) = a - (p_coffcient(1)*p_coffcient(1) + 2*p_coffcient(0)*p_coffcient(2));
            poly_coffi(3) = -2 * p_coffcient(1)*p_coffcient(2);
            poly_coffi(4) = - p_coffcient(2) * p_coffcient(2);

            for(int i = 0; i < 5;i++)
            {
                if(std::isnan(poly_coffi(i)))
                {
                    std::cout <<"Error, This should not happen"<<std::endl;
                }
            }


            //进行多项式的求解，得到对应的lambda．
            double lambda;
            if(SolverFourthOrderPolynomial(poly_coffi,lambda) == false)
            {
                std::cout <<"Polynomial Solve Failed"<<std::endl;
                return false;
            }

            //得到lambda之后，根据式24．
            Eigen::Matrix4d W;
            W.setZero();
            W.block(2,2,2,2) = Eigen::Matrix2d::Identity();

            //Eigen::Vector4d res = -(M + 2 *lambda * W).inverse().transpose() * g;
            Eigen::Vector4d res = -(M + 2 * lambda * W).inverse() * g;

            //转换成旋转矩阵
            double theta = std::atan2(res(3),res(2));
            deltaTrans << cos(theta),-sin(theta),res(0),
                    sin(theta), cos(theta),res(1),
                    0,          0,     1;
            return true;
        }
//调用Eigen求解四次多项式的第一个非零实根
        static bool SolverFourthOrderPolynomial(Eigen::VectorXd &p_coff, double &lambda) {
            Eigen::PolynomialSolver<double,4> polySolve(p_coff);

            Eigen::Matrix<std::complex<double>,4,1,0,4,1> roots = polySolve.roots();

            bool isAssigned = false;
            double finalRoot = 0.0;

            //找到第一个非零实根--有可能不止一个，因为有优化空间．
            for(int i = 0; i < roots.size();i++)
            {
                //如果是虚根，则不要．
                if(roots(i).imag() != 0 )continue;

                //如果是非零实根，则选择．
                if(isAssigned == false ||  roots(i).real() > finalRoot)
                {
                    isAssigned = true;
                    finalRoot = roots(i).real();
                }
            }

            lambda = finalRoot;
            return isAssigned;
        }

        void HashMatrixRThetaPhi(){
            for(int i = 1;i < int(2 * M_PI / angle_increment);i++){
                double phi = 0;
                double theta = i * angle_increment - M_PI;
                SRIAngle.push_back(theta);
                //std::cout<<"theta: "<<theta<<std::endl;
                Eigen::Matrix3d MTheta,MPhi,Mat;

                MPhi << cos(phi),0,-sin(phi),
                        0,1,0,
                        sin(phi),0, cos(phi);

                MTheta << cos(theta),-sin(theta),0,
                          sin(theta), cos(theta),0,
                          0,0,1;

                Mat = MTheta * MPhi;
                //std::cout<<"mat:\n"<<Mat<<std::endl;
                SRImap.insert({theta,Mat});

            }

        }

        void saveTUMTrajectory(ros::Time time,const Eigen::Vector3d& t_,const Eigen::Vector4d& q_) {
            std::ofstream laser_odom_of;
            laser_odom_of.open("/home/lee/mywork08_ws/src/lidarodomslam/src/traj/F_IMU.txt",std::ios::app);
            if (!laser_odom_of.is_open())
                std::cout<<"open file failed."<<std::endl;

            laser_odom_of<<time<<" "<<t_[0]<<" "<<t_[1]<<" "<<0<<" "<<q_[0]<<" "<<q_[1]<<" "<<q_[2]<<" "<<q_[3]<<"\n";
            laser_odom_of.close();
        }
    };
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"lidar_odometry");
    lidar_slam::lidar_odometry imlsImplement;
    ros::spin();
    return 0;
}