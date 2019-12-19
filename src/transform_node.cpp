#include <ros/ros.h>
#include "laser_transform_handler.h"
#include <string>

int main(int argc, char** argv){
  ros::init(argc, argv, "scan_to_pointcloud_transform")
  ros::NodeHandle node;
  TransformHandler transformer;
  string frame_name, topic_out, topic_in;

  //Initialize strings
  frame_name = "map";
  topic_in = "base_scan";
  topic_out = "localized_pointcloud";

  //Set internal variables
  transformer.set_frame(frame_name);
  transformer.set_output_topic(topic_out, node);
  transformer.set_input_topic(topic_in, node);

  //Prevents node from exiting until shutdown
  ros::spin();
  return 0;
}
