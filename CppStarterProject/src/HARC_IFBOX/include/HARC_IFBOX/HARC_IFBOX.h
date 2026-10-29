#pragma once

#include <HARC_IFBOX/Capture.h>
#include <HARC_IFBOX/Configuration.h>
#include <HARC_IFBOX/Patch.h>

#include <serial/serial.h>

namespace harc_ifbox
{

class HARC_IFBOX
{
public:
  /** Creates an HARC_IFBOX with a single patch corresponding to the full sheet of the maximum size*/
  HARC_IFBOX(const char * device, uint32_t baudrate);

  /** Creates an HARC_IFBOX with the settings specificed in the Configuration */
  explicit HARC_IFBOX(const Configuration & config);

  /** Prevent copy or move */
  HARC_IFBOX(const HARC_IFBOX &) = delete;
  HARC_IFBOX & operator=(const HARC_IFBOX &) = delete;
  HARC_IFBOX(HARC_IFBOX &&) = delete;
  HARC_IFBOX & operator=(HARC_IFBOX &&) = delete;

  size_t startCapture();
  size_t endCapture();

  Capture::Return capture();

  inline size_t frame() const noexcept { return capture_.frame(); }

  inline const Patches & getPatches() const noexcept { return patches_; }

private:
  serial::Serial port;
  Capture capture_ = Capture{MAX_ROWS, MAX_COLS};
  Patches patches_ = {Patch{"default", MAX_ROWS, MAX_COLS}};
};

} // namespace harc_ifbox
