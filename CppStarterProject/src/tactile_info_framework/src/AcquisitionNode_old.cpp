#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <stdio.h>

#include <boost/thread.hpp>

#include <math.h> //fabs

// #include <tactile_info_framework/AcquisitionNodelet.hpp>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg//multi_array_dimension.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
// #include "visualization_msgs/msg/markers.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

// acquire data from serial port
// and publish it
#include <serial/serial.h>

void do_stuff(int * publish_rate)
{
  rclcpp::Node::SharedPtr node;
  auto pub_matrix_raw_ = node->create_publisher<sensor_msgs::msg::Image>("message", 10);

  // matrix
  // cv::Mat matrix_raw;

  // message
  sensor_msgs::msg::Image msg_matrix_raw;

  // init CV bridge
  //  cv_bridge_matrix_raw;

  rclcpp::Rate loop_rate(*publish_rate);

  // get parameters
  int rows_ = 6;
  int cols_ = 22;
  std::string port_name_ = "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_56AAAD9B81683EF9-if00";
  int baudrate_ = 4800;

  // add rows and cols offset
  rows_ += 1;
  cols_ += 1;

  // init running state
  bool running_ = true;

  // init serial connection
  serial::Serial serial_port_;
  serial_port_.setPort(port_name_);
  serial_port_.setBaudrate(baudrate_);

  // try to open serial connection
  try
  {
    if(serial_port_.isOpen()) { std::cout << "Port already opened" << std::endl; }
    else
    {
      serial_port_.open();
      std::cout << "Port opened" << std::endl;
    }
  }
  catch(std::exception & e)
  {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
    serial_port_.close();
    std::cout << "Port closed" << std::endl;
  }

  std::cout << "Starts acquisition" << std::endl;

  int DLE_MARKER = 0x10;
  int STX_MARKER = 0x02;
  int ETX_MARKER = 0x03;

  int got_byte;
  int k = 0;
  int state = 0;
  char prev_byte = 0;
  int dle_count = 0;
  int frame = 0;
  int frame_count = 0;
  int size_x = 0;
  int size_y = 0;
  int size_count = 0;
  int x_count = 0;
  int y_count = 0;

  bool first_iteration_ = true;

  uint8_t got_byte_char = 0;

  // init acquisition matrix with zeros
  uint8_t acquisition_matrix[rows_][cols_];
  for(int i = 0; i < rows_; i++)
  {
    for(int j = 0; j < cols_; j++) { 0 >> acquisition_matrix[i][j]; }
  }

  // main loop
  while(rclcpp::ok())
  {

    uint8_t * ptr = new uint8_t;
    int buf_size;

    *ptr = 0; // change value to 0

    std::cout << "number of bytes: " << serial_port_.available() << std::endl;
    if(serial_port_.available() > 0)
    {
      // rate.sleep();

      buf_size = serial_port_.read(ptr, 1);

      // byte read
      got_byte_char = *ptr;
      got_byte = (int)got_byte_char;

      std::cout << "value: " << got_byte << std::endl;
      std::cout << "address: " << (void *)ptr << std::endl;

      // got_byte = static_cast<int>(*ptr);
      if(state == 0)
      {
        // start
        if((prev_byte == DLE_MARKER) && (got_byte == STX_MARKER)) { state = 1; }
      }

      else if(state == 1)
      { // frame
        if((prev_byte != DLE_MARKER) && (got_byte == DLE_MARKER)) { dle_count++; }
        else if((prev_byte == DLE_MARKER) && (got_byte != DLE_MARKER) && (dle_count % 2 == 1))
        {
          dle_count = 0;
          state = 0;
        }
        else if((prev_byte == DLE_MARKER) && (got_byte == DLE_MARKER) && (dle_count % 2 == 0)) { dle_count++; }
        else
        {
          if(got_byte == DLE_MARKER) { dle_count++; }
          else { dle_count = 0; }
          frame |= got_byte << frame_count * 8;
          frame_count++;
          if(frame_count >= 4)
          {
            frame_count = 0;
            state = 2;
          }
        }
      }

      else if(state == 2)
      { // size
        if((prev_byte != DLE_MARKER) && (got_byte == DLE_MARKER)) { dle_count++; }
        else if((prev_byte == DLE_MARKER) && (got_byte != DLE_MARKER) && (dle_count % 2 == 1))
        {
          dle_count = 0;
          state = 0;
        }
        else if((prev_byte == DLE_MARKER) && (got_byte == DLE_MARKER) && (dle_count % 2 == 0)) { dle_count++; }
        else
        {
          if(got_byte == DLE_MARKER) { dle_count++; }
          else { dle_count = 0; }
          if(size_count == 0) { size_x = got_byte & 0xFF; }
          else if(size_count == 1) { size_y = got_byte & 0xFF; }
          size_count++;
          if(size_count >= 2)
          {
            size_count = 0;
            state = 3;
          }
        }
      }

      else if(state == 3)
      { // value
        if((prev_byte != DLE_MARKER) && (got_byte == DLE_MARKER)) { dle_count++; }
        else if((prev_byte == DLE_MARKER) && (got_byte != DLE_MARKER) && (dle_count % 2 == 1))
        {
          dle_count = 0;
          state = 0;
        }
        else if((prev_byte == DLE_MARKER) && (got_byte == DLE_MARKER) && (dle_count % 2 == 0)) { dle_count++; }
        else
        {
          if(got_byte == DLE_MARKER) { dle_count++; }
          else { dle_count = 0; }
          if((prev_byte == DLE_MARKER) && (got_byte == ETX_MARKER))
          {
            x_count = 0;
            y_count = 0;
            state = 0;
          }
          else
          {
            int unsigned_force = got_byte & 0xFF;
            acquisition_matrix[x_count][y_count] = (uint8_t)unsigned_force;

            x_count++;
            if(x_count == size_x)
            {

              x_count = 0;

              y_count++;
              if(y_count == size_y)
              {
                y_count = 0;
                state = 0;
              }
            }
          }
        }
      }

      prev_byte = got_byte;
    }
    else { std::cout << "Serial port no available" << std::endl; }

    // std_msgs::Empty msg;
    // pub_b.publish(msg);
    loop_rate.sleep();
    // convert acquisition matrix (std::array) to cv::Mat
    // does not copy the data
    cv::Mat acquisition_matrix_cv(rows_, cols_, CV_8UC1, &acquisition_matrix);

    // remove useless 11th rows and cols
    // THERE IS A BUG HERE
    cv::Mat matrix_raw = acquisition_matrix_cv.colRange(1, acquisition_matrix_cv.cols)
                             .rowRange(1, acquisition_matrix_cv.rows); //.clone();

    // transform to image message and publish
    std::cout << "Publishing image" << std::endl;

    // transform to image message and publish
    cv_bridge::CvImage cv_bridge_matrix_raw =
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, matrix_raw);
    cv_bridge_matrix_raw.toImageMsg(msg_matrix_raw);
    // msg_matrix_raw.header.stamp = ros::Time::now();
    pub_matrix_raw_->publish(msg_matrix_raw);

    if(first_iteration_)
    {
      first_iteration_ = false;
      std::cout << "Acquisition started" << std::endl;
    }
    std::cout << "loop" << std::endl;
    delete ptr;
  }
}

// main function
int main(int argc, char ** argv)
{
  int rate_b = 10000; // 1 Hz

  rclcpp::init(argc, argv);

  // spawn another thread
  boost::thread thread_b(do_stuff, &rate_b);

  rclcpp::Node::SharedPtr node;

  rclcpp::Rate loop_rate(10); // 10 Hz
  while(rclcpp::ok())
  {
    // std_msgs::Empty msg;
    // pub_a.publish(msg);
    loop_rate.sleep();

    // process any incoming messages in this thread
    rclcpp::spin_some(node);
  }

  // wait the second thread to finish
  thread_b.join();

  return 0;
}
