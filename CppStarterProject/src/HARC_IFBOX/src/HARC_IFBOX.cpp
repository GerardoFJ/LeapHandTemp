#include <HARC_IFBOX/HARC_IFBOX.h>
#include <iostream>

namespace harc_ifbox
{

static const std::string start_code = "start\n";
static const std::string stop_code = "stop\n";

HARC_IFBOX::HARC_IFBOX(const char * device, uint32_t baudrate) : HARC_IFBOX(Configuration::default_(device, baudrate))
{
}

HARC_IFBOX::HARC_IFBOX(const Configuration & config)
: port(config.device, config.baudrate, serial::Timeout::simpleTimeout(500)), capture_(config.rows, config.cols),
  patches_(
      [](const Configuration::Patches & patches)
      {
        Patches out;
        out.reserve(patches.size());
        for(const auto & p : patches) { out.push_back(Patch{p.name, p.rows, p.cols, p.mappings}); }
        return out;
      }(config.patches))
{
  if(!port.isOpen())
  {
    std::cerr << "Failed to open port " << config.device << " at " << config.baudrate << " rate" << std::endl;
    throw std::runtime_error("Failed to open");
  }
}

size_t HARC_IFBOX::startCapture()
{
  return port.write(start_code);
}

size_t HARC_IFBOX::endCapture()
{
  return port.write(stop_code);
}

Capture::Return HARC_IFBOX::capture()
{
  unsigned char byte_data;
  size_t len;
  Capture::Return ret;

  unsigned int count = 0;
  unsigned int max_frame_size = MAX_ROWS * MAX_COLS;

  while(count <= 2 * max_frame_size)
  {

    len = port.read(&byte_data, 1);
    if(len == 0)
    {
      std::cerr << "Failed to read data, retrying\n";
      continue;
    }
    ret = capture_.capture(byte_data);

    if(ret == Capture::Return::E_DRDY)
    {
      for(auto & p : patches_) { p.update(capture_.data()); }
      break;
    }

    count++;
  }

  return ret;
}

} // namespace harc_ifbox
