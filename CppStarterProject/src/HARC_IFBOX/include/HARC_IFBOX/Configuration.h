#pragma once

#include <HARC_IFBOX/Capture.h>
#include <HARC_IFBOX/Patch.h>

#include <optional>
#include <string>

namespace harc_ifbox
{

struct Configuration
{
  static inline constexpr uint32_t DEFAULT_BAUDRATE = 921600;

  /** Serial device this should connect to */
  std::string device;
  /** Baudrate for the connection */
  uint32_t baudrate = DEFAULT_BAUDRATE;
  /** Number of rows in the sheet */
  size_t rows = MAX_ROWS;
  /** Number of cols in the sheet */
  size_t cols = MAX_COLS;
  /** Configuration for a patch */
  struct Patch
  {
    /** Name of the patch */
    std::string name;
    /** Number of rows in the patch */
    size_t rows;
    /** Number of rows in the patch */
    size_t cols;
    /** Mapping for this patch (no mapping => copy the raw sheet) */
    std::optional<DataMatrix::Mappings> mappings = std::nullopt;
  };
  using Patches = std::vector<Patch>;
  Patches patches;

  static inline Configuration default_(const char * device, uint32_t baudrate)
  {
    auto out = Configuration{};
    out.device = device;
    out.baudrate = baudrate;
    out.patches = {Patch{"default", out.rows, out.cols}};
    return out;
  }

  static Configuration loadFile(const std::string & f);

  static Configuration loadData(const char * data);
};

} // namespace harc_ifbox
