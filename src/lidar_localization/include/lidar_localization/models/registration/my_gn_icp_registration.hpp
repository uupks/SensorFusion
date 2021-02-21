#ifndef MY_GN_ICP_REGISTRATION_HPP
#define MY_GN_ICP_REGISTRATION_HPP

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "Eigen/Eigen"
namespace lidar_localization {

class MyGNICPRegistration: public RegistrationInterface {
  public:
    MyGNICPRegistration(const YAML::Node& node);
    MyGNICPRegistration(
      float max_corr_dist, 
      float trans_eps, 
      float euc_fitness_eps, 
      int max_iter
    );

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;

    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
    
  private:
    bool ComputeCorrespondences(
        CloudData::CLOUD_PTR& transformed_source,
        CloudData::CLOUD_PTR& matched_target);

  private:
    CloudData::CLOUD_PTR target_;
      float max_corr_dist_;
      float trans_eps_;
      float euc_fitness_eps_;
      int max_iter_;

};

}



#endif