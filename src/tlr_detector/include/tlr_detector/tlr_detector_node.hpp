#ifndef TLR_DETECTOR_NODE_HPP_
#define TLR_DETECTOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ultralytics_ros/msg/yolo_result.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace tlr_detector
{

class TlrDetectorNode : public rclcpp::Node
{
public:
  TlrDetectorNode();
  virtual ~TlrDetectorNode();

private:
  void yolo_result_callback(const ultralytics_ros::msg::YoloResult::SharedPtr msg);

  rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr yolo_result_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tlr_result_publisher_;
};

}  // namespace tlr_detector

#endif  // TLR_DETECTOR_NODE_HPP_
