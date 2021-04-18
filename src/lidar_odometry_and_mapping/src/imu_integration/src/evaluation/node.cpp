#include <fstream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>

std::ofstream ground_truth_file;
std::ofstream mid_point_estimation_file;
std::ofstream euler_estimation_file;

std::string ground_truth = "./ground_truth_file.txt";
std::string mid_point_estimation = "./mid_point_estimation.txt";
std::string euler_estimation = "./euler_estimation.txt";

void EvaluationCallback(const nav_msgs::OdometryConstPtr& gt_ptr, 
                        const nav_msgs::OdometryConstPtr& mid_point_est_ptr,
                        const nav_msgs::OdometryConstPtr& euler_est_ptr)
{
    printf("GT : [%f, %f, %f]\n", gt_ptr->pose.pose.position.x, 
                                  gt_ptr->pose.pose.position.y,
                                  gt_ptr->pose.pose.position.z);
            
    printf("Estimation : [%f, %f, %f]\n", mid_point_est_ptr->pose.pose.position.x, 
                                  mid_point_est_ptr->pose.pose.position.y,
                                  mid_point_est_ptr->pose.pose.position.z);

    ground_truth_file<<gt_ptr->header.stamp.toNSec()<<" "
                     <<gt_ptr->pose.pose.position.x<<" "
                     <<gt_ptr->pose.pose.position.y<<" "
                     <<gt_ptr->pose.pose.position.z<<" "
                     <<gt_ptr->pose.pose.orientation.x<<" "
                     <<gt_ptr->pose.pose.orientation.y<<" "
                     <<gt_ptr->pose.pose.orientation.z<<" "
                     <<gt_ptr->pose.pose.orientation.w<<std::endl;

    mid_point_estimation_file<<mid_point_est_ptr->header.stamp.toNSec()<<" "
                     <<mid_point_est_ptr->pose.pose.position.x<<" "
                     <<mid_point_est_ptr->pose.pose.position.y<<" "
                     <<mid_point_est_ptr->pose.pose.position.z<<" "
                     <<mid_point_est_ptr->pose.pose.orientation.x<<" "
                     <<mid_point_est_ptr->pose.pose.orientation.y<<" "
                     <<mid_point_est_ptr->pose.pose.orientation.z<<" "
                     <<mid_point_est_ptr->pose.pose.orientation.w<<std::endl;
    
    euler_estimation_file<<euler_est_ptr->header.stamp.toNSec()<<" "
                     <<euler_est_ptr->pose.pose.position.x<<" "
                     <<euler_est_ptr->pose.pose.position.y<<" "
                     <<euler_est_ptr->pose.pose.position.z<<" "
                     <<euler_est_ptr->pose.pose.orientation.x<<" "
                     <<euler_est_ptr->pose.pose.orientation.y<<" "
                     <<euler_est_ptr->pose.pose.orientation.z<<" "
                     <<euler_est_ptr->pose.pose.orientation.w<<std::endl;
}

int main(int argc, char **argv) {
    std::string node_name{"imu_integration_evaluation_node"};
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    message_filters::Subscriber<nav_msgs::Odometry> groundtruth_sub(nh, "/pose/ground_truth", 100);
    message_filters::Subscriber<nav_msgs::Odometry> mid_point_est_sub(nh, "/pose/mid_point_estimation", 100);
    message_filters::Subscriber<nav_msgs::Odometry> euler_est_sub(nh, "/pose/euler_estimation", 100);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), groundtruth_sub, mid_point_est_sub, euler_est_sub);
    sync.registerCallback(EvaluationCallback);

    ground_truth_file.open(ground_truth.c_str());
    mid_point_estimation_file.open(mid_point_estimation.c_str());
    euler_estimation_file.open(euler_estimation.c_str());
    
    ground_truth_file<<std::fixed;
    mid_point_estimation_file<<std::fixed;
    euler_estimation_file<<std::fixed;

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ground_truth_file.close();
    mid_point_estimation_file.close();
    euler_estimation_file.close();
    return 0;
}
