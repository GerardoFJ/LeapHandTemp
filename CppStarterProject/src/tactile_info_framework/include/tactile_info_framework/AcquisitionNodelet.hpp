/**
 * @file AcquisitionNodelet.hpp
 * @author Selim Chefchaouni, Rafael Cisneros
 * @brief Acquisition nodelet for JST MIRAI tactile demo
 * @version 0.1
 * @date 2023-09-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <memory>

#include <sensor_msgs/msg/image.hpp>
#include <HARC_IFBOX/HARC_IFBOX.h>
#include <rclcpp/rclcpp.hpp>

namespace tactile_info_framework
{
class AcquisitionNodelet : public rclcpp::Node
{
public:
  AcquisitionNodelet(const rclcpp::NodeOptions & options);

  ~AcquisitionNodelet();

private:
  std::string harc_cfg_path_;
  std::unique_ptr<harc_ifbox::HARC_IFBOX> harc_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> patch_publishers_;

  bool running_;
  std::shared_ptr<std::thread> deviceThread_;

  void acquisitionThread();
};
} // namespace tactile_info_framework
