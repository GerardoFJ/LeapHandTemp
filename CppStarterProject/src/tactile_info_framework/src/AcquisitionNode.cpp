jrlgerardo@jrlgerardo:~$ cat AcquisitionNode.cpp 
// Acquires tactile frames from the sensor's serial port and publishes each one
// as a mono8 image of rows x cols on "message".
//
// Frame format sent by the sensor:
//   DLE STX | frame number (4 bytes, big endian) | size_x | size_y |
//   size_x * size_y values, x changing fastest | DLE ETX
// with DLE = 0x10, STX = 0x02, ETX = 0x03, and every 0x10 inside the frame sent
// twice (DLE DLE). size_x runs along the image columns and size_y along its
// rows, so the values arrive already in row-major image order.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <serial/serial.h>

namespace
{

constexpr uint8_t DLE = 0x10;
constexpr uint8_t STX = 0x02;
constexpr uint8_t ETX = 0x03;
constexpr size_t HEADER_SIZE = 6; // frame number (4) + size_x + size_y
constexpr size_t MAX_PAYLOAD = HEADER_SIZE + 255 * 255;

// Splits the byte stream into frame payloads and undoes the DLE stuffing
class FrameParser
{
public:
  // Feeds one byte; returns true when a complete payload is available in payload()
  bool feed(uint8_t byte)
  {
    if(!escape_)
    {
      if(byte == DLE) { escape_ = true; }
      else if(in_frame_) { append(byte); }
      return false;
    }

    escape_ = false;
    if(byte == DLE)
    { // stuffed 0x10 value
      if(in_frame_) { append(DLE); }
    }
    else if(byte == STX)
    { // start of frame, drops any unfinished one
      if(in_frame_ && !payload_.empty()) { dropped_++; }
      in_frame_ = true;
      payload_.clear();
    }
    else if(in_frame_)
    {
      in_frame_ = false;
      if(byte == ETX) { return true; }
      dropped_++; // DLE followed by anything else is invalid
    }
    return false;
  }

  const std::vector<uint8_t> & payload() const { return payload_; }

  // Number of frames dropped because they were cut off or malformed
  size_t dropped() const { return dropped_; }

private:
  void append(uint8_t byte)
  {
    if(payload_.size() < MAX_PAYLOAD) { payload_.push_back(byte); }
    else
    { // no DLE ETX within any valid frame size, give up on this frame
      in_frame_ = false;
      dropped_++;
    }
  }

  bool in_frame_ = false;
  bool escape_ = false;
  std::vector<uint8_t> payload_;
  size_t dropped_ = 0;
};

} // namespace

class AcquisitionNode : public rclcpp::Node
{
public:
  AcquisitionNode() : Node("acquisition_node")
  {
    // same names and defaults as the tactile YAML config
    device_ = declare_parameter<std::string>(
        "device", "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_56AAAD9B81683EF9-if00");
    baudrate_ = declare_parameter<int>("baudrate", 921600);
    rows_ = declare_parameter<int>("rows", 32); // image height = the sensor's size_y
    cols_ = declare_parameter<int>("cols", 16); // image width = the sensor's size_x

    publisher_ = create_publisher<sensor_msgs::msg::Image>("message", 10);
    reader_ = std::thread(&AcquisitionNode::read_loop, this);
  }

  ~AcquisitionNode() override
  {
    running_ = false;
    if(reader_.joinable()) { reader_.join(); }
  }

private:
  void read_loop()
  {
    serial::Serial port;
    port.setPort(device_);
    port.setBaudrate(baudrate_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100); // read() waits at most 100 ms
    port.setTimeout(timeout);

    FrameParser parser;
    size_t reported_dropped = 0;
    std::vector<uint8_t> buffer(4096);

    while(running_ && rclcpp::ok())
    {
      try
      {
        if(!port.isOpen())
        {
          port.open();
          RCLCPP_INFO(get_logger(), "Opened %s", device_.c_str());
        }

        // wait for at least one byte, then take everything that is already there
        size_t wanted = std::clamp<size_t>(port.available(), 1, buffer.size());
        size_t got = port.read(buffer.data(), wanted);
        for(size_t i = 0; i < got; i++)
        {
          if(parser.feed(buffer[i])) { publish(parser.payload()); }
        }
      }
      catch(const std::exception & e)
      {
        // port missing, unplugged or busy: close it and try again in a second
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Serial port %s: %s", device_.c_str(),
                              e.what());
        try
        {
          port.close();
        }
        catch(const std::exception &)
        {
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      if(parser.dropped() > reported_dropped)
      {
        reported_dropped = parser.dropped();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropped %zu malformed frames so far",
                             reported_dropped);
      }
    }
  }

  void publish(const std::vector<uint8_t> & payload)
  {
    if(payload.size() < HEADER_SIZE)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Frame of only %zu bytes, dropped",
                           payload.size());
      return;
    }

    const uint32_t number = (static_cast<uint32_t>(payload[0]) << 24) | (static_cast<uint32_t>(payload[1]) << 16)
                            | (static_cast<uint32_t>(payload[2]) << 8) | static_cast<uint32_t>(payload[3]);
    const int size_x = payload[4];
    const int size_y = payload[5];
    const size_t values = payload.size() - HEADER_SIZE;

    if(size_x != cols_ || size_y != rows_ || values != static_cast<size_t>(size_x * size_y))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Frame %u has size_x=%d size_y=%d and %zu values, expected size_x=cols=%d "
                           "size_y=rows=%d; dropped",
                           number, size_x, size_y, values, cols_, rows_);
      return;
    }

    if(frames_ == 0) { RCLCPP_INFO(get_logger(), "Receiving %d x %d frames", rows_, cols_); }
    else if(number != last_number_ + 1)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Frame number jumped %u -> %u, frames were lost",
                           last_number_, number);
    }
    last_number_ = number;
    frames_++;

    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.height = rows_;
    msg.width = cols_;
    msg.encoding = sensor_msgs::image_encodings::MONO8;
    msg.is_bigendian = false;
    msg.step = cols_;
    msg.data.assign(payload.begin() + HEADER_SIZE, payload.end());
    publisher_->publish(msg);
  }

  std::string device_;
  int baudrate_ = 0;
  int rows_ = 0;
  int cols_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  // only used by the reader thread
  uint32_t last_number_ = 0;
  uint64_t frames_ = 0;

  std::atomic<bool> running_{true};
  std::thread reader_; // last, so everything above exists before the thread starts
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AcquisitionNode>());
  rclcpp::shutdown();
  return 0;
}
