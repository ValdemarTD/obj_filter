#ifndef __LT_HANDLER_H
#define __LT_HANDLER_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <string>

//Class to handle all of the laser_geometry laserscan to pointcloud conversions.
class TransformHandler{
private:
  ros::Publisher cloud_out;
  ros::Subscriber scan_in;
  tf::TransformListener tf_listener;
  laser_geometry::LaserProjection projector;
  string frame;
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr&);

public:
  void set_input_topic(string, &ros::NodeHandle);
  void set_output_topic(string, &ros::NodeHandle);
};

#endif
