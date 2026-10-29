#include <HARC_IFBOX/Capture.h>

#include <iostream>

namespace harc_ifbox
{

/** @def
 * escape code for data status identifier
 */
static inline constexpr int DLE_BYTE = 0x10;

/** @def
 * Data start byte
 */
static inline constexpr int STX_BYTE = 0x02;

/** @def
 * Data end byte
 */
static inline constexpr int ETX_BYTE = 0x03;

auto Capture::capture(uint8_t byte_data) -> Return
{
  auto ret = handle_byte(byte_data);
  if(ret == Return::E_DRDY) { return finalize_capture(); }
  return ret;
}

auto Capture::handle_byte(uint8_t byte_data) -> Return
{
  switch(state_)
  {
    case State::E_INIT:
      state_ = State::E_WAIT_HEADER;
      return Return::E_NOERR;
    case State::E_WAIT_HEADER:
      if(byte_data == DLE_BYTE) { state_ = State::E_DLE_CHK; }
      return Return::E_NOERR;
    case State::E_RUN:
      if(byte_data == DLE_BYTE)
      {
        state_ = State::E_DLE_CHK;
        return Return::E_NOERR;
      }
      if(current_index_ > MAX_FRAME_SIZE)
      {
        current_index_ = 0;
        state_ = State::E_WAIT_HEADER;
        return Return::E_SIZE_ERR;
      }
      buffer_[current_index_] = byte_data;
      current_index_++;
      state_ = State::E_RUN;
      return Return::E_NOERR;
    case State::E_DLE_CHK:
      if(byte_data == DLE_BYTE)
      {
        if(current_index_ > MAX_FRAME_SIZE)
        {
          current_index_ = 0;
          state_ = State::E_WAIT_HEADER;
          return Return::E_SIZE_ERR;
        }
        buffer_[current_index_] = byte_data;
        current_index_++;
        state_ = State::E_RUN;
        return Return::E_NOERR;
      }
      if(byte_data == STX_BYTE)
      {
        current_index_ = 0;
        state_ = State::E_RUN;
        return Return::E_CAPTURE;
      }
      if(byte_data == ETX_BYTE)
      {
        current_index_ = 0;
        state_ = State::E_WAIT_HEADER;
        return Return::E_DRDY;
      }
      current_index_ = 0;
      state_ = State::E_WAIT_HEADER;
      return Return::E_HDR_ERR;
    default:
      current_index_ = 0;
      state_ = State::E_INIT;
      return Return::E_GEN_ERR;
  }
}

auto Capture::finalize_capture() -> Return
{
  size_t cols = buffer_[4];
  size_t rows = buffer_[5];
  if(cols != data_.cols() || rows != data_.rows())
  {
    std::cerr << "Acquisition error got " << rows << "x" << cols << " message but expected " << data_.rows() << "x"
              << data_.cols() << "\n";
    return Return::E_SIZE_ERR;
  }
  frame_ = ((size_t)buffer_[0]) << 24 | ((size_t)buffer_[1]) << 16 | ((size_t)buffer_[2]) << 8 | ((size_t)buffer_[3]);
  // data_matrix stores in row-major order so we can just copy to it
  data_.copy(&buffer_[6]);
  return Return::E_DRDY;
}

} // namespace harc_ifbox
