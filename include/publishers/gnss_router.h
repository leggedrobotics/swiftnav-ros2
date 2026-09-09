/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

/**
 * @brief Routes NavSatFix messages between the normal topic and a ground-truth topic.
 *
 * Simulates an unplugged GNSS antenna in software. While an outage is engaged the normal
 * topic goes completely silent -- which is what the rest of the stack must see, since
 * holistic_fusion and lodia_path_manager both treat "no message" as the loss condition --
 * while the same messages keep flowing on the ground-truth topic, so a bag recorded during
 * the run still contains the true position.
 *
 * The flag is atomic because the service callback runs on the node's executor thread while
 * publish() is called from the SBP processing thread.
 */
class GnssRouter {
 public:
  GnssRouter(rclcpp::Node* node, const std::string& ground_truth_topic,
             bool start_in_outage)
      : outage_(start_in_outage) {
    ground_truth_publisher_ =
        node->create_publisher<sensor_msgs::msg::NavSatFix>(ground_truth_topic,
                                                            10);
  }

  /**
   * @brief True when the normal topic is currently being suppressed.
   */
  bool inOutage() const { return outage_.load(); }

  /**
   * @brief Engage or clear the simulated outage.
   */
  void setOutage(bool outage) { outage_.store(outage); }

  /**
   * @brief Publish to the ground-truth topic.
   */
  void publishGroundTruth(const sensor_msgs::msg::NavSatFix& msg) {
    ground_truth_publisher_->publish(msg);
  }

 private:
  std::atomic<bool> outage_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>>
      ground_truth_publisher_;
};

using GnssRouterPtr = std::shared_ptr<GnssRouter>;
