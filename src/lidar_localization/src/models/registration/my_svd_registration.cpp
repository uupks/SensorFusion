#include "lidar_localization/models/registration/my_svd_registration.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/transforms.h"
#include "glog/logging.h"

namespace lidar_localization {

MySVDICPRegistration::MySVDICPRegistration(const YAML::Node& node) {
    max_corr_dist_ = node["max_corr_dist"].as<float>();
    trans_eps_ = node["trans_eps"].as<float>();
    euc_fitness_eps_ = node["euc_fitness_eps"].as<float>();
    max_iter_ = node["max_iter"].as<int>();
}

MySVDICPRegistration::MySVDICPRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) : max_corr_dist_(max_corr_dist), trans_eps_(trans_eps), euc_fitness_eps_(euc_fitness_eps), max_iter_(max_iter){

}

bool MySVDICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    LOG(INFO) <<"Target Cloud Size : "<<input_target->size();
    target_ = input_target;
    return true;
}


bool MySVDICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {

    //1. 构建 KD-TREE 
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target_);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    CloudData::CLOUD_PTR transformed_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source, *transformed_source, predict_pose);

    // ICP 迭代
    for (size_t i = 0; i < max_iter_; i++)
    {
        LOG(INFO)<<"Iter : "<<i;
        // 寻找一个最近邻点
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
                // 只添加最邻近距离小于 max_corr_dist_ 的点用于SVD计算
                if (point_squared_distance[0] < max_corr_dist_ * max_corr_dist_) {
                    temp_source->push_back(transformed_source->at(i));
                    temp_target->push_back(target_->at(point_index[0]));
                }
            }
        }
        LOG(INFO)<<"temp source size : "<<temp_source->size()<<" / "<<input_source->size();
        LOG(INFO)<<"temp target size : "<<temp_target->size()<<" / "<<target_->size();

        // 2. 计算2组点去质心坐标
        // getMatrixXfMap()返回 4XN 矩阵， 每一列为(x, y, z, 1)
        auto source = temp_source->getMatrixXfMap().block(0, 0, 3, temp_source->points.size());
        auto target = temp_target->getMatrixXfMap().block(0, 0, 3, temp_target->points.size());

        Eigen::Vector3f source_center = source.rowwise().mean();
        Eigen::Vector3f target_center = target.rowwise().mean();
        Eigen::MatrixXf centered_source = source.colwise() - source_center;
        Eigen::MatrixXf centered_target = target.colwise() - target_center;

        // 3. 计算旋转矩阵 3X3 = 3XN * NX3
        // 系数矩阵
        // Eigen::MatrixXf W = centered_target * centered_source.transpose();
        Eigen::MatrixXf W = centered_source * centered_target.transpose();
        Eigen::MatrixXf U, S, V, Vt;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        U = svd.matrixU();
        V = svd.matrixV();
        if (U.determinant() * V.determinant() < 0)
        {
            for (size_t i = 0; i < V.rows(); i++) {
                V(i, 2) *= - 1;
            }
        }
        // Eigen::Matrix3f rotation = U * V.transpose();
        Eigen::Matrix3f rotation = V * U.transpose();

        // 4. 计算平移 t
        Eigen::Vector3f t = target_center - rotation * source_center;
        Eigen::Matrix4f delta_T = Eigen::Matrix4f::Identity();
        delta_T.block(0, 0, 3, 3) = rotation;
        delta_T.block(0, 3, 3, 1) = t;
        // LOG(INFO)<<"delta_T : \n"<<delta_T<<std::endl;
        // 更新transform
        transform = delta_T * transform;
        // 变换点云
        pcl::transformPointCloud(*transformed_source, *transformed_source, delta_T);
        
        // 5. 判断是否收敛
        int nr = 0;
        double fitness_score = 0.0;
        for (size_t i = 0; i < transformed_source->size(); i++)
        {
            const auto p = transformed_source->at(i);
            kdtree.nearestKSearch(p, k, point_index, point_squared_distance);
            fitness_score += point_squared_distance[0];
            nr++;
        }

        double euc_fitness = 0.0;
        if (nr > 0) {
            euc_fitness = fitness_score / nr;
            LOG(INFO)<<"euc_fitness : "<<euc_fitness;
        }
        if (t.squaredNorm() < trans_eps_ || ((euc_fitness < euc_fitness_eps_) && (euc_fitness_eps_ != 0.0))) {
            break;
        }
    }
    result_pose = transform * predict_pose;
    pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose);

    return true;
}

float MySVDICPRegistration::GetFitnessScore() {
    return 0.0;
}

}