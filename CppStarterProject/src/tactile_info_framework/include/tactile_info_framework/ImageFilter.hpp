#include <cstring>
#include <stdint.h>
#include <stdio.h>

#include <boost/scoped_ptr.hpp>

#include "filters/filter_base.h"
#include <pluginlib/class_list_macros.h>

namespace tactile_info_framework
{

class ImageFilter : public FilterBase<int>
{
public:
  ImageFilter();
  ~ImageFilter();
  virtual bool configure();
  virtual bool update(const T & data_in, T & data_out);
};

PLUGINLIB_EXPORT_CLASS(IncrementFilterInt, filters::IncrementFilter<int>, filters::FilterBase<int>)

} // namespace tactile_info_framework
