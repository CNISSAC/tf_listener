#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <fstream>
#include <string>
#include <unistd.h> // for getting user's home directory
#include <chrono>
#include <iostream>
#include <ctime>


int saveMessage(const Eigen::Affine3d & eigen_pose, std::string tip_name,int count){

  // get user's home directory
  const char* home_dir = getenv("HOME");
  if (!home_dir) {
    ROS_ERROR("Failed to get home directory");
    return -1;
  }
  std::string file_name = "/"+tip_name + ".csv";
  // std::string file_path = std::string(home_dir) + "/transform.csv";
  std::string file_path = std::string(home_dir) + file_name;
  // create and open csv file
  std::ofstream csv_file(file_path, std::ofstream::out | std::ofstream::app);

  if (!csv_file.is_open()) {
    ROS_ERROR("Failed to open file");
    return -1;
  }

  // write timestamp to csv file
  auto now = std::chrono::system_clock::now();
  std::time_t time = std::chrono::system_clock::to_time_t(now);

  // Format the time as year, month, day, hour, minute, and second
  char timestamp[20];
  std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&time));
  // std::string timestamp = std::to_string(ros::Time::now().toSec());
  csv_file << "Transform "<<count<< " at "<<timestamp<<"is :"<<"\n";
  
  // write eigen_pose to csv file
  
  Eigen::Matrix<double, 4, 4> pose_mat = eigen_pose.matrix();
  csv_file << pose_mat(0, 0) << "," << pose_mat(0, 1) << "," << pose_mat(0, 2) << "," << pose_mat(0, 3) << "\n"
           << pose_mat(1, 0) << "," << pose_mat(1, 1) << "," << pose_mat(1, 2) << "," << pose_mat(1, 3) << "\n"
           << pose_mat(2, 0) << "," << pose_mat(2, 1) << "," << pose_mat(2, 2) << "," << pose_mat(2, 3) << "\n"
           << 0 << "," << 0 << "," << 0 << "," << 1 << "\n";

      
  ROS_INFO_STREAM("Transform:"<<count<<"\n" << eigen_pose.matrix()<<"\nhas been saved");
  return 0;

} 



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  //Define node parameters
  std::string base_frame, tool_frame, tip_name;
  node.param<std::string>("base_frame", base_frame, "base_link");
  node.param<std::string>("tool_frame", tool_frame, "tool");
  node.param<std::string>("tip_name", tip_name, "middle_tip");

  tf::TransformListener listener;
  ros::Rate rate(10.0);
  ROS_INFO("Start listener");
  std::string line;
  int count = 1;
  while (node.ok()){
    tf::StampedTransform transform;

    ROS_INFO("Pose %d: Jog robot to a new location touching the shared position and"
             " press enter.", count);

    std::getline(std::cin, line); // Blocks program until enter is pressed

    try{
      // listener.lookupTransform("/turtle2", "/turtle1",
      //                          ros::Time(0), transform);
      listener.lookupTransform(base_frame, tool_frame,
                               ros::Time(0), transform);                         
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    Eigen::Affine3d eigen_pose;
    tf::poseTFToEigen(transform, eigen_pose);
    // ROS_INFO_STREAM("Pose " << ": captured transform:\n" << eigen_pose.matrix());
    saveMessage(eigen_pose,tip_name,count);
    count++;
    rate.sleep();
  }

  return 0;
};