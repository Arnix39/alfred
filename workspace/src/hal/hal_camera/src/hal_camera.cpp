// Copyright (c) 2022 Arnix Robotix
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hal_camera.hpp"

using namespace std::chrono_literals;

Camera::Camera()
: rclcpp_lifecycle::LifecycleNode("hal_camera_node")
{
}

LifecycleCallbackReturn_t Camera::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  imagePublisher =
    this->create_publisher<sensor_msgs::msg::Image>("cameraImage", 10);

  imagePublisherTimer =
    create_wall_timer(200ms, std::bind(&Camera::captureAndPublishFrame, this));

  capture.open(0, 0);
  capture.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(320));
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(240));

  if (!capture.isOpened()) {
    RCLCPP_ERROR(get_logger(), "hal_camera node not able to open camera!");
    return LifecycleCallbackReturn_t::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "hal_camera node configured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Camera::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  imagePublisher->on_activate();

  RCLCPP_INFO(get_logger(), "hal_camera node activated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Camera::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  imagePublisher->on_deactivate();

  RCLCPP_INFO(get_logger(), "hal_camera node deactivated!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Camera::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  imagePublisher.reset();
  imagePublisherTimer.reset();
  capture.release();

  RCLCPP_INFO(get_logger(), "hal_camera node unconfigured!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Camera::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  imagePublisher.reset();
  imagePublisherTimer.reset();

  RCLCPP_INFO(get_logger(), "hal_camera node shutdown!");

  return LifecycleCallbackReturn_t::SUCCESS;
}

LifecycleCallbackReturn_t Camera::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return LifecycleCallbackReturn_t::FAILURE;
}

void Camera::captureAndPublishFrame(void)
{
  cv::Mat frame;
  cv::Mat frameFlipped;
  capture.read(frameFlipped);

  if (frameFlipped.empty()) {
    RCLCPP_ERROR(get_logger(), "Captured frame is empty!");
  } else {
    cv::flip(frameFlipped, frame, 1);
    sensor_msgs::msg::Image::SharedPtr imageMsg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    imagePublisher->publish(*imageMsg);
  }
}
