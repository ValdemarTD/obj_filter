#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cstdlib>
#include <string>
#include "pointcloud_transform_handler.h"


//Takes in a pointcloud, converts it to PC2 format, then publishes
void PCTransformHandler::scan_callback(const sensor_msgs::PointCloud::ConstPtr& pc_in){
  sensor_msgs::PointCloud2 pc_out;
  convertPointCloudToPointCloud2(*pc_in, pc_out);
  cloud_out.publish(pc_out);
}

//Sets input topic. Should be of type "PointCloud"
void PCTransformHandler::set_input_topic(std::string topic, ros::NodeHandle * nh){
  cloud_in = nh->subscribe(topic, 1, &PCTransformHandler::scan_callback, this);
}

//Sets output topic. Should be of type "PointCloud2"
void PCTransformHandler::set_output_topic(std::string topic, ros::NodeHandle * nh){
  cloud_out = nh->advertise<sensor_msgs::PointCloud2>(topic, 1);
}
