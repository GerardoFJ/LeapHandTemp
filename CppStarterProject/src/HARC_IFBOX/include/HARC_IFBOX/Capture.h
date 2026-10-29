#pragma once

#include <HARC_IFBOX/DataMatrix.h>

#include <limits>

namespace harc_ifbox
{

/** Number of bytes reserved for publishing the frame number */
static inline constexpr size_t FRAME_SIZE = 4;

/** Number of bytes reserved for publish the cols/rows value */
static inline constexpr size_t SIZE_SIZE = 2;

/** @def
 * maximum number of rows (Y size)
 */
static inline constexpr size_t MAX_ROWS = 32;

/** @def
 * maximum number of columns (X size)
 */
static inline constexpr size_t MAX_COLS = 16;

/** Maximum size of a single frame */
static inline constexpr size_t MAX_FRAME_SIZE = MAX_ROWS * MAX_COLS + FRAME_SIZE + SIZE_SIZE;

/** Raw data from the HARC_IFBOX device and processing from the serial interface */
struct Capture
{
  /**
   * @enum Return
   *
   * @brief return code of CaptureDataArray
   */
  enum class Return
  {
    E_NOERR = 0,
    E_CAPTURE = 1,
    E_DRDY = 2,
    E_GEN_ERR = -1,
    E_SIZE_ERR = -2,
    E_HDR_ERR = -3,
  };

  /**
   * @enum State
   *
   * @brief status code of CaptureDataArray
   */
  enum class State
  {
    E_INIT = 0,
    E_WAIT_HEADER,
    E_RUN,
    E_DLE_CHK,
    E_STATE_ETX,
    E_WAIT_STX,
  };

  inline Capture(size_t rows, size_t cols) : data_(rows, cols) {}

  /** Last data acquired by this Capture object */
  inline const DataMatrix & data() const noexcept { return data_; }

  /** Last frame acquired by this Capture object */
  inline size_t frame() const noexcept { return frame_; }

  /**
   * @fn capture
   *
   * @brief captures byte stream data to matrix in HARC format
   * @param[in] (byte_data) data from byte stream
   * @return status code @see CAPTURE_RETURN
   */
  Return capture(uint8_t byte_data);

private:
  State state_ = State::E_INIT;
  size_t current_index_ = 0;
  size_t frame_ = std::numeric_limits<size_t>::max();
  std::vector<uint8_t> buffer_ = std::vector<uint8_t>(MAX_FRAME_SIZE, 0);

  DataMatrix data_;

  Return handle_byte(uint8_t byte_data);

  /** Called just before \ref capture would return E_DRDY
   *
   * - Do a sanity check on the message (returns E_SIZE_ERR if it fails)
   * - Copy data from the raw array to the DataMatrix
   */
  Return finalize_capture();
};

} // namespace harc_ifbox
