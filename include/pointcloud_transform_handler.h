#ifndef __PCT_HANDLER_H
#define __PCT_HANDLER_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <string>

//Class to transform a PointCloud to PointCloud2 format
class PCTransformHandler{
private:
  ros::Publisher cloud_out;
  ros::Subscriber cloud_in;
  void scan_callback(const sensor_msgs::PointCloud::ConstPtr&);
public:
  void set_input_topic(std::string, ros::NodeHandle*);
  void set_output_topic(std::string, ros::NodeHandle*);
};

#endif
