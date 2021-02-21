#include "lidar_localization/models/registration/my_gn_icp_registration.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/transforms.h"
#include "glog/logging.h"
#include "sophus/se3.hpp"

namespace lidar_localization {

MyGNICPRegistration::MyGNICPRegistration(const YAML::Node& node) {
    max_corr_dist_ = node["max_corr_dist"].as<float>();
    trans_eps_ = node["trans_eps"].as<float>();
    euc_fitness_eps_ = node["euc_fitness_eps"].as<float>();
    max_iter_ = node["max_iter"].as<int>();
}


MyGNICPRegistration::MyGNICPRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) : max_corr_dist_(max_corr_dist), trans_eps_(trans_eps), euc_fitness_eps_(euc_fitness_eps), max_iter_(max_iter){

}

bool MyGNICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    target_ = input_target;
    return true;
}

bool MyGNICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {

    //1. 构建 KD-TREE 
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target_);

    LOG(INFO)<<"Predict_pose : \n"<<predict_pose;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    CloudData::CLOUD_PTR transformed_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source, *transformed_source, predict_pose);

    double cost = 0.0, last_cost = 0.0;
    // ICP 迭代
    for (size_t i = 0; i < max_iter_; i++)
    {
        LOG(INFO)<<"Iter : "<<i;
        // 寻找最近邻点
        int k = 1;
        pcl::Indices point_index(k);
        std::vector<float> point_squared_distance;

        CloudData::CLOUD_PTR temp_target(new CloudData::CLOUD());
        CloudData::CLOUD_PTR temp_source(new CloudData::CLOUD());

        // 2. 在target中寻找source最近邻点
        for (size_t i = 0; i < transformed_source->size(); i++)
        {
            pcl::PointXYZ p = transformed_source->at(i);
            int nums = kdtree.nearestKSearch(p, k, point_index, point_squared_distance);
            if (nums > 0) {
                // 只添加最邻近距离小于 max_corr_dist 的点用于计算
                if (point_squared_distance[0] < max_corr_dist_ * max_corr_dist_) {
                    temp_source->push_back(transformed_source->at(i));
                    temp_target->push_back(target_->at(point_index[0]));
                }
            }
        }
        LOG(INFO)<<"temp source size : "<<temp_source->size()<<" / "<<input_source->size();
        LOG(INFO)<<"temp target size : "<<temp_target->size()<<" / "<<target_->size();

        // 3. 计算 Hessian 和 b
        // getMatrixXfMap()返回 4XN 矩阵， 每一列为(x, y, z, 1)
        auto source = temp_source->getMatrixXfMap();//.block(0, 0, 3, temp_source->points.size());
        auto target = temp_target->getMatrixXfMap();//.block(0, 0, 3, temp_target->points.size());
        
        cost = 0.0;
        // Jacobian
        Eigen::MatrixXf Hessian = Eigen::MatrixXf::Zero(6, 6);
        Eigen::VectorXf b = Eigen::VectorXf::Zero(6);
        for (size_t i = 0; i < source.cols(); i++) {
            // residual
            Eigen::Vector3f error = target.block(0, i, 3, 1) - source.block(0, i, 3, 1);
            cost += error.squaredNorm();
            Eigen::MatrixXf Jpose = Eigen::MatrixXf::Zero(3, 6);
            Jpose(0, 0) = -1;
            Jpose(0, 4) = -source.col(i).z();
            Jpose(0, 5) = source.col(i).y();
            Jpose(1, 1) = -1;
            Jpose(1, 3) = source.col(i).z();
            Jpose(1, 5) = -source.col(i).x();
            Jpose(2, 2) = -1;
            Jpose(2, 3) = -source.col(i).y();
            Jpose(2, 4) = source.col(i).x();

            Hessian += Jpose.transpose() * Jpose;
            b += -Jpose.transpose() * error;
        }
        // LOG(INFO)<<"Hessian : "<<Hessian.rows()<<" x "<<Hessian.cols()<<"\n"<<Hessian;
        // LOG(INFO)<<"b : "<<b.rows()<<" x "<<b.cols()<<"\n"<<b.transpose();
        // LOG(INFO)<<"Cost : "<<cost;

        Eigen::VectorXf delta = Hessian.ldlt().solve(b);
        // LOG(INFO)<<"delta : "<<delta.rows()<<" x "<<delta.cols()<<"\n"<<delta.transpose();

        if (std::isnan(delta[0])) {
            LOG(INFO) <<"Result is nan";
            break;
        }

        if (i > 0 && cost > last_cost) {
            LOG(INFO) << "cost : "<<cost<<", last cost : "<<last_cost;
            break;
        }

        auto delta_T = Sophus::SE3f::exp(delta);
        LOG(INFO)<<"delta_T : \n"<<delta_T.matrix();
        last_cost = cost;

        // 更新transform
        transform = delta_T.matrix() * transform;
        // 变换点云
        // source = transform * source;
        pcl::transformPointCloud(*transformed_source, *transformed_source, delta_T.matrix());
        // pcl::transformPointCloud(*input_source, *transformed_source, transform);
        // 5. 判断是否收敛
        if (delta.norm() < 1e-6) {
            LOG(INFO) << "Converge";
            break;
        }
        
    }
    // result_pose = transform;
    result_pose = transform * predict_pose;
    LOG(INFO)<<"Result Pose : \n"<<result_pose;
    pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose);

    return true;
}

float MyGNICPRegistration::GetFitnessScore() {
    return 0.0;
}

}