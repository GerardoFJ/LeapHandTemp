#include <filesystem>

#include <opencv2/highgui.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <tactile_info_framework/AcquisitionNodelet.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.hpp>

namespace fs = std::filesystem;

namespace tactile_info_framework
{
AcquisitionNodelet::AcquisitionNodelet(const rclcpp::NodeOptions & options)
: rclcpp::Node("AcquisitionNodelet", options)
{
  this->declare_parameter("harc_config", rclcpp::PARAMETER_STRING);
  fs::path harc_config_path = this->get_parameter("harc_config").as_string();
  auto harc_config = harc_ifbox::Configuration::loadFile(harc_config_path);
  harc_ = std::make_unique<harc_ifbox::HARC_IFBOX>(harc_config);
  harc_->startCapture();

  const auto & patches = harc_->getPatches();
  patch_publishers_.reserve(patches.size());
  for(const auto & p : patches)
  {
    patch_publishers_.push_back(this->create_publisher<sensor_msgs::msg::Image>(
        "/AcquisitionNodelet/" + (harc_config_path.stem() / p.name()).string() + "/message", 10));
  }

  running_ = true;

  deviceThread_ = std::make_shared<std::thread>(&AcquisitionNodelet::acquisitionThread, this);
}

AcquisitionNodelet::~AcquisitionNodelet()
{
  running_ = false;
  if(deviceThread_->joinable()) { deviceThread_->join(); }
}

void AcquisitionNodelet::acquisitionThread()
{
  RCLCPP_INFO(this->get_logger(), "Starts acquisition");

  sensor_msgs::msg::Image msg_matrix_raw;
  cv_bridge::CvImage cv_bridge_matrix_raw;

  while(running_)
  {
    harc_->capture();
    auto stamp = this->get_clock()->now();
    for(size_t i = 0; i < patch_publishers_.size(); ++i)
    {
      const auto & data = harc_->getPatches()[i].data();

      cv::Mat matrix_raw(data.rows(), data.cols(), CV_8UC1,
                         static_cast<void *>(const_cast<unsigned char *>(data.data())));

      cv_bridge_matrix_raw =
          cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, matrix_raw);
      cv_bridge_matrix_raw.toImageMsg(msg_matrix_raw);
      msg_matrix_raw.header.stamp = stamp;
      patch_publishers_[i]->publish(msg_matrix_raw);
    }
  }
}
} // namespace tactile_info_framework

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(tactile_info_framework::AcquisitionNodelet)
