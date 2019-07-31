/* Copyright (C) 2019 XU WANG , Hirain
 *
 * License BSD
 *
 * Author: XU WANG
 *
 * This is a nodelet class for lidar start
 *
 */

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include "driver.h"

namespace lidar_start {

class LidarStartNodelet : public nodelet::Nodelet {

 private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ros::Publisher cloud_pub;
  // lidar driver
  itd_lidar::lidar_driver::Driver *lidar;
  // input param
  std::string lidar_type, ip, multicast_ip;
  std::string innovizPro_correction_file_path;
  std::string p40p_correction_file_path;
  std::string RS32_angle_correction_file_path, RS32_ChannelNum_correction_file_path, RS32_CurveRate_correction_file_path, RS32_curves_correction_file_path;
  int data_port, mode, direction, version;
  std::string origin_cloud_topic, frame_id;
  // cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudPtr;

 public:
  LidarStartNodelet() {};
  virtual ~LidarStartNodelet() {};

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();
    if(!initialize_params()) return;
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(origin_cloud_topic, 1);
    // start lidar sdk
    lidar->Start();
  }

 private:
  bool initialize_params(void) {
    // param input
    lidar_type = private_nh.param<std::string>("lidar_type", "");
    data_port = private_nh.param<int>("data_port", 2368);
    ip = private_nh.param<std::string>("ip", "");
    multicast_ip = private_nh.param<std::string>("multicast_ip", "");
    mode = private_nh.param<int>("mode", 0);
    direction = private_nh.param<int>("direction", 0);
    version = private_nh.param<int>("version", 1);
    innovizPro_correction_file_path = private_nh.param<std::string>("innovizPro_correction_file_path", "");
    p40p_correction_file_path = private_nh.param<std::string>("p40p_correction_file_path", "");
    RS32_angle_correction_file_path = private_nh.param<std::string>("RS32_angle_correction_file_path", "");
    RS32_ChannelNum_correction_file_path = private_nh.param<std::string>("RS32_ChannelNum_correction_file_path", "");
    RS32_CurveRate_correction_file_path = private_nh.param<std::string>("RS32_CurveRate_correction_file_path", "");
    RS32_curves_correction_file_path = private_nh.param<std::string>("RS32_curves_correction_file_path", "");
    origin_cloud_topic = private_nh.param<std::string>("origin_cloud_topic", "");
    frame_id = private_nh.param<std::string>("frame_id", "");

    // cloud init
    CloudPtr.reset(new pcl::PointCloud<pcl::PointXYZI>());

    if (lidar_type == "P40P") {
      lidar = new itd_lidar::lidar_driver::Driver(ip, multicast_ip, data_port,
                                                  lidar_type, mode, p40p_correction_file_path,
                                                  boost::bind(&LidarStartNodelet::LidarCallback, this, _1, _2));
    }
    else if (lidar_type == "RSL32"){
      std::initializer_list<std::string> correctionfileList{
        RS32_angle_correction_file_path,
        RS32_ChannelNum_correction_file_path,
        RS32_CurveRate_correction_file_path,
        RS32_curves_correction_file_path
      };

      lidar = new itd_lidar::lidar_driver::Driver(ip, multicast_ip, data_port, lidar_type, mode, direction, version, correctionfileList,
                                                  boost::bind(&LidarStartNodelet::LidarCallback, this, _1, _2));
    } else if (lidar_type == "VLP16") {
      lidar = new itd_lidar::lidar_driver::Driver(ip, multicast_ip, data_port, lidar_type, mode, "",
                                                  boost::bind(&LidarStartNodelet::LidarCallback, this, _1, _2));
    } else if (lidar_type == "InnovizPro") {
      lidar = new itd_lidar::lidar_driver::Driver(ip, multicast_ip, data_port, lidar_type, mode, innovizPro_correction_file_path,
                                                  boost::bind(&LidarStartNodelet::LidarCallback, this, _1, _2));
    } else {
      ROS_INFO("Invalid lidar type!");
      return false;
    }

    return true;
  }

  void LidarCallback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cld, double timestamp) {
    (void) timestamp;

    // cloud process
    pcl::copyPointCloud(*cld, *CloudPtr);

    // cloud send
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*CloudPtr, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pub.publish(cloud_msg);
  }

};

} // lidar_star

PLUGINLIB_EXPORT_CLASS(lidar_start::LidarStartNodelet, nodelet::Nodelet)
