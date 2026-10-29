#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <stdio.h>

#include <math.h> //fabs, floor

#include <tactile_info_framework/ProcessingNodelet.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace fs = std::filesystem;

struct CentroidForce
{
  float x;
  float y;
  float force;
};

namespace tactile_info_framework
{

ProcessingNodelet::ProcessingNodelet(const rclcpp::NodeOptions & options) : rclcpp::Node("ProcessingNodelet", options)
{
  declare_parameter("publish_blured", false);
  declare_parameter("publish_filtered", false);
  declare_parameter("publish_contours", false);
  declare_parameter("publish_centroids", false);
  declare_parameter("activation_threshold", 100);
  declare_parameter("harc_config", rclcpp::PARAMETER_STRING);

  // get parameters
  if(this->has_parameter("publish_blured"))
  {
    this->get_parameter("publish_blured", publish_blured_);
    RCLCPP_INFO(get_logger(), "Got param '/publish_blured': %i", publish_blured_);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Failed to get param '/publish_blured'. Used default value instead: %i", publish_blured_);
  }

  if(this->has_parameter("publish_filtered"))
  {
    this->get_parameter("publish_filtered", publish_filtered_);
    RCLCPP_INFO(get_logger(), "Got param '/publish_filtered': %i", publish_filtered_);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Failed to get param '/publish_filtered'. Used default value instead: %i",
                publish_filtered_);
  }

  if(this->has_parameter("publish_contours"))
  {
    this->get_parameter("publish_contours", publish_contours_);
    RCLCPP_INFO(get_logger(), "Got param '/publish_contours': %i", publish_contours_);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Failed to get param '/publish_contours'. Used default value instead: %i",
                publish_contours_);
  }

  if(this->has_parameter("publish_centroids"))
  {
    this->get_parameter("publish_centroids", publish_centroids_);
    RCLCPP_INFO(get_logger(), "Got param '/publish_centroids': %i", publish_centroids_);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Failed to get param '/publish_centroids'. Used default value instead: %i",
                publish_centroids_);
  }

  if(this->has_parameter("activation_threshold"))
  {
    this->get_parameter("activation_threshold", activation_threshold_);
    RCLCPP_INFO_STREAM(get_logger(), "[" << get_name() << "] Activation threshold: " << activation_threshold_);
  }
  else
  {
    RCLCPP_WARN_STREAM(get_logger(), "[" << get_name()
                                         << "] Failed to get activation_threshold param, Using default value: "
                                         << activation_threshold_);
  }

  fs::path harc_config_path = get_parameter("harc_config").as_string();
  auto harc_config = harc_ifbox::Configuration::loadFile(harc_config_path);
  const auto & harc_patches = harc_config.patches;
  patches_.resize(harc_patches.size());

  for(size_t i = 0; i < harc_patches.size(); ++i)
  {
    const auto & name = (harc_config_path.stem() / harc_patches[i].name).string();
    auto & patch = patches_[i];
    patch.parent_ = this;
    patch.pub_blured_ =
        this->create_publisher<sensor_msgs::msg::Image>("/ProcessingNodelet/" + name + "/blured_array", 10);
    patch.pub_filtered_ =
        this->create_publisher<sensor_msgs::msg::Image>("/ProcessingNodelet/" + name + "/filtered_array", 10);
    patch.pub_contours_ =
        this->create_publisher<sensor_msgs::msg::Image>("/ProcessingNodelet/" + name + "/contours_array", 10);
    patch.pub_centroids_ =
        this->create_publisher<sensor_msgs::msg::Image>("/ProcessingNodelet/" + name + "/centroids_array", 10);
    patch.pub_force_ = this->create_publisher<tactile_info_framework::msg::TactileContact>(
        "/ProcessingNodelet/" + name + "/force", 10);
    // FIXME Need to make the acquisition nodelet configurable?
    // FIXME Need to pass the acquisition nodelet name down?
    patch.sub_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/AcquisitionNodelet/" + name + "/message", 10,
        std::bind(&ProcessingNodelet::PatchProcess::callback, &patch, std::placeholders::_1));
  }
}

void ProcessingNodelet::PatchProcess::callback(const sensor_msgs::msg::Image::ConstSharedPtr & input)
{
  // get raw image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::MONO8);
  }
  catch(cv_bridge::Exception & e)
  {
    RCLCPP_ERROR(parent_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat new_matrix_raw;
  new_matrix_raw = cv_ptr->image;

  cv::Mat new_matrix_blured;
  cv::GaussianBlur(new_matrix_raw, new_matrix_blured, cv::Size(5, 3), 0, 0);

  // apply filter
  cv::Mat new_matrix_filtered;
  cv::threshold(new_matrix_raw, // source
                new_matrix_filtered, // destination
                parent_->activation_threshold_, // threshold value
                255, // max binary value
                cv::THRESH_TOZERO // threshold type
  );

  // find contours
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(new_matrix_filtered, // source
                   contours, // destination
                   hierarchy, // hierarchy
                   cv::RETR_TREE, // mode
                   cv::CHAIN_APPROX_SIMPLE // method
  );

  // find centroids
  cv::Mat new_matrix_centroids = cv::Mat::zeros(new_matrix_filtered.size(), CV_8UC1);
  std::vector<CentroidForce> centroids_forces;
  for(unsigned int k = 0; k < contours.size(); k++)
  {
    // bounding rectangle
    cv::Rect rect = cv::boundingRect(contours[k]);
    // sum of all pixels in the bounding rectangle
    cv::Scalar sumOfIntensities = cv::sum(new_matrix_filtered(rect));
    unsigned int sum = sumOfIntensities[0];
    // centroid
    cv::Mat roi = cv::Mat(new_matrix_filtered, rect);
    roi.convertTo(roi, CV_32F);
    int sum_x = 0; // weighted sum of x coordinates
    cv::Mat reduced_x = cv::Mat::zeros(1, rect.width, CV_32F); // vector of weights on x axis
    cv::reduce(roi, reduced_x, 0, cv::REDUCE_SUM);
    int sum_y = 0; // weighted sum of y coordinates
    cv::Mat reduced_y = cv::Mat::zeros(rect.height, 1, CV_32F); // vector of weights on y axis
    cv::reduce(roi, reduced_y, 1, cv::REDUCE_SUM);
    reduced_x.convertTo(reduced_x, CV_8UC1);
    reduced_y.convertTo(reduced_y, CV_8UC1);
    for(int i = 0; i < rect.width; i++) { sum_x += (int)reduced_x.at<uchar>(0, i) * i; }
    for(int i = 0; i < rect.height; i++) { sum_y += (int)reduced_y.at<uchar>(i, 0) * i; }
    int centroid_x = fabs(cvRound(sum_x / cv::sum(reduced_x)[0]));
    int centroid_y = fabs(cvRound(sum_y / cv::sum(reduced_y)[0]));
    cv::Point2i centroid(centroid_x + rect.x, centroid_y + rect.y);
    new_matrix_centroids.at<uchar>(centroid) = 255; // sum
    // add to vector
    CentroidForce centroid_force;
    centroid_force.x = centroid.y;
    centroid_force.y = centroid.x;
    centroid_force.force = sum;
    centroids_forces.push_back(centroid_force);
  }

  // draw contours
  cv::Mat new_matrix_contours = cv::Mat::zeros(new_matrix_filtered.size(), CV_8UC1);
  if(parent_->publish_contours_)
  {
    for(size_t i = 0; i < contours.size(); i++)
    {
      cv::Scalar color = 150;
      cv::drawContours(new_matrix_contours, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
    }
  }

  // publish
  if(parent_->publish_blured_)
  {
    cv_bridge::CvImage cv_bridge_matrix_blured;
    sensor_msgs::msg::Image msg_matrix_blured;
    cv_bridge_matrix_blured =
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, new_matrix_blured);
    cv_bridge_matrix_blured.toImageMsg(msg_matrix_blured);
    msg_matrix_blured.header.stamp = parent_->get_clock()->now();
    pub_blured_->publish(msg_matrix_blured);
  }
  if(parent_->publish_filtered_)
  {
    cv_bridge::CvImage cv_bridge_matrix_filtered;
    sensor_msgs::msg::Image msg_matrix_filtered;
    cv_bridge_matrix_filtered =
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, new_matrix_filtered);
    cv_bridge_matrix_filtered.toImageMsg(msg_matrix_filtered);
    msg_matrix_filtered.header.stamp = parent_->get_clock()->now();
    pub_filtered_->publish(msg_matrix_filtered);
  }
  if(parent_->publish_contours_)
  {
    cv_bridge::CvImage cv_bridge_matrix_contours;
    sensor_msgs::msg::Image msg_matrix_contours;
    cv_bridge_matrix_contours =
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, new_matrix_contours);
    cv_bridge_matrix_contours.toImageMsg(msg_matrix_contours);
    msg_matrix_contours.header.stamp = parent_->get_clock()->now();
    pub_contours_->publish(msg_matrix_contours);
  }
  if(parent_->publish_centroids_)
  {
    cv_bridge::CvImage cv_bridge_matrix_centroids;
    sensor_msgs::msg::Image msg_matrix_centroids;
    cv_bridge_matrix_centroids =
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, new_matrix_centroids);
    cv_bridge_matrix_centroids.toImageMsg(msg_matrix_centroids);
    msg_matrix_centroids.header.stamp = parent_->get_clock()->now();
    pub_centroids_->publish(msg_matrix_centroids);
  }

  // force message
  tactile_info_framework::msg::TactileContact msg_force;
  int i = 0;
  for(std::vector<CentroidForce>::iterator it = centroids_forces.begin(); it != centroids_forces.end(); ++it)
  {
    geometry_msgs::msg::Vector3 vector;
    vector.x = (*it).x;
    vector.y = (*it).y;
    vector.z = (*it).force;
    msg_force.forces.push_back(vector);
    i++;
  }

  pub_force_->publish(msg_force);

  matrix_raw_ = new_matrix_raw;
  matrix_blured_ = new_matrix_blured;
  matrix_filtered_ = new_matrix_filtered;
  matrix_centroids_ = new_matrix_centroids;

  centroids_forces.clear();
}
} // namespace tactile_info_framework

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(tactile_info_framework::ProcessingNodelet)
