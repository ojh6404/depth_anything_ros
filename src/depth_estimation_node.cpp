#include <NvInfer.h>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <tuple>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "depth_anything.h"
#include "utils.h"
#include <opencv2/opencv.hpp>

/**
 * @brief Setting up Tensorrt logger
 */
class Logger : public nvinfer1::ILogger {
  void log(Severity severity, const char *msg) noexcept override {
    // Only output logs with severity greater than warning
    if (severity <= Severity::kWARNING)
      std::cout << msg << std::endl;
  }
} logger;

cv::Mat image;
std::string frame_id;

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
    frame_id = msg->header.frame_id;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "depth_estimation");
  ros::NodeHandle nh("~");

  // get the engine file path from rosparam
  std::string model_path;
  if (!nh.getParam("model_path", model_path)) {
    ROS_ERROR("Failed to model from rosparam!");
    return -1;
  }

  double depth_scale;
  if (!nh.getParam("depth_scale", depth_scale)) {
    ROS_ERROR("Failed to get depth_scale from rosparam!");
    return -1;
  }

  // ros info
  ROS_INFO("Loading model from %s...", model_path.c_str());
  DepthAnything depth_model(model_path, logger);
  ROS_INFO("Model loaded successfully!");

  ros::Subscriber sub_image = nh.subscribe("input_image", 1, imageCallback);
  ros::Publisher pub_depth =
      nh.advertise<sensor_msgs::Image>("depth_registered/image_rect", 1);

  while (ros::ok()) {
    ros::spinOnce();
    if (image.empty()) {
      continue;
    }

    cv::Mat depth_mat = depth_model.predict(image);
    // Rescale the depth image
    depth_mat = depth_mat * depth_scale;

    // Resizing predicted depth image to the same size as the input image with nearest neighbor interpolation
    cv::resize(depth_mat, depth_mat, cv::Size(image.cols, image.rows), 0, 0, cv::INTER_NEAREST);

    // publish depth image
    sensor_msgs::ImagePtr depth_msg =
        cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_mat).toImageMsg();
    depth_msg->header.stamp = ros::Time::now();
    depth_msg->header.frame_id = frame_id;
    pub_depth.publish(depth_msg);
  }

  return 0;
}
