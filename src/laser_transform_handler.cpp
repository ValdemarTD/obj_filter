#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <string>
#include <cstdlib>
#include "laser_transform_handler.h"


//Scan callback. Takes the given scan, uses the laser_geometry library to convert
//it to a PointCloud in the frame specified by frame, then publishes that cloud to
//the previously set output topic
void TransformHandler::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
  if(!tf_listener.waitForTransform(
        scan_in->header.frame_id,
        frame,
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector.transformLaserScanToPointCloud(frame , *scan_in, cloud, listener);

  cloud_out.publish(cloud);
}

//Sets the input topic and creates a publisher for that topic
void TransformHandler::set_input_topic(string topic, &ros::NodeHandle nh){
  scan_in = nh->subscribe(topic, 1, scan_callback);
}

//Sets the output topic and initializes a subscriber to that topic
void TransformHandler::set_output_topic(string, topic &ros::NodeHandle nh){
  cloud_out = nh->advertise<sensor_msgs::PointCloud>(topic, 1);
}

//Sets the frame the pointcloud will be set in
void TransformHandler::set_frame(string frame_in){
  frame = frame_in;
}
