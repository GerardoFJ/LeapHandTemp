#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <stdio.h>

#include <math.h> //fabs

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <cv_bridge/cv_bridge.hpp>
// #include <visualization_msgs/msg/marker.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <HARC_IFBOX/Configuration.h>

#include <tactile_info_framework/msg/tactile_contact.hpp>

namespace tactile_info_framework
{

class ProcessingNodelet : public rclcpp::Node
{
public:
  // constructor
  ProcessingNodelet(const rclcpp::NodeOptions & options);

private:
  // internal variables
  struct PatchProcess
  {
    ProcessingNodelet * parent_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_filtered_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_blured_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_contours_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_centroids_;
    rclcpp::Publisher<tactile_info_framework::msg::TactileContact>::SharedPtr pub_force_;
    cv::Mat matrix_raw_;
    cv::Mat matrix_blured_;
    cv::Mat matrix_filtered_;
    cv::Mat matrix_centroids_;
    cv::Mat first_matrix_;
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr & input);
  };
  std::vector<PatchProcess> patches_;

  // params
  bool publish_blured_;
  bool publish_filtered_;
  bool publish_contours_;
  bool publish_centroids_;
  int activation_threshold_;
};
} // namespace tactile_info_framework
