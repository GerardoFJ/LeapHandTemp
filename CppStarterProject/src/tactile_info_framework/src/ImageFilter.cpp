#include "pluginlib/class_list_macros.h"
#include <tactile_info_framework/ImageFilter.hpp>

namespace tactile_info_framework
{

ImageFilter::ImageFilter() {}

bool ImageFilter::configure()
{
  return true;
}

template<typename T>
ImageFilter::~ImageFilter()
{
}

bool ImageFilter::update(const int & data_in, int & data_out)
{
  data_out = data_in + 1;
  return true;
};
} // namespace tactile_info_framework
