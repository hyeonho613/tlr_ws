#include "tlr_detector/tlr_detector_node.hpp"

namespace tlr_detector
{

TlrDetectorNode::TlrDetectorNode() : Node("tlr_detector_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting tlr_detector_node...");

  yolo_result_subscription_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
    "/yolo_result", 10, std::bind(&TlrDetectorNode::yolo_result_callback, this, std::placeholders::_1));

  tlr_result_publisher_ = this->create_publisher<std_msgs::msg::String>("/tlr_result", 10);

  RCLCPP_INFO(this->get_logger(), "Subscribing to /yolo_result and publishing to /tlr_result.");
}

TlrDetectorNode::~TlrDetectorNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down tlr_detector_node...");
}

void TlrDetectorNode::yolo_result_callback(const ultralytics_ros::msg::YoloResult::SharedPtr msg)
{
  std::string light_color = "unknown";

  for (const auto& detection : msg->detections.detections) {
    for (const auto& result : detection.results) {
        if (result.hypothesis.class_id == "green") {
            light_color = "green";
            break;
        } else if (result.hypothesis.class_id == "red") {
            light_color = "red";
            break;
        }
    }
    if (light_color != "unknown") {
      break;
    }
  }

  std_msgs::msg::String result_msg;
  result_msg.data = light_color;
  tlr_result_publisher_->publish(result_msg);
  RCLCPP_DEBUG(this->get_logger(), "Published traffic light result: %s", light_color.c_str());
}

}  // namespace tlr_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tlr_detector::TlrDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
