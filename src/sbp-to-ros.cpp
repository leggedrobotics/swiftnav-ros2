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

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <logging/ros_logger.h>
#include <logging/sbp_to_ros2_logger.h>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/publisher_factory.h>
#include <publishers/publisher_manager.h>

#include <data_sources/sbp_data_sources.h>
#include <publishers/gnss_router.h>
#include <std_srvs/srv/set_bool.hpp>
#include <utils/config.h>
#include <utils/utils.h>

static const int64_t LOG_REPUBLISH_DELAY =
    TimeUtils::secondsToNanoseconds(2ULL);

/**
 * @brief Class that represents the ROS 2 driver node
 */
class SBPROS2DriverNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new SBPROS2DriverNode object
   */
  SBPROS2DriverNode() : Node("swiftnav_ros2_driver") {
    config_ = std::make_shared<Config>(this);
    logger_ = std::make_shared<ROSLogger>(LOG_REPUBLISH_DELAY);

    createDataSources();
    if (!data_source_) exit(EXIT_FAILURE);
    state_.set_reader(data_source_.get());
    state_.set_writer(data_source_.get());
    gnss_router_ = std::make_shared<GnssRouter>(
        this, config_->getGnssGroundTruthTopic(),
        config_->getStartWithGnssOutage());
    createPublishers();
    createGnssOutageService();

    sbptoros2_ = std::make_shared<SBPToROS2Logger>(
        &state_, logger_, config_->getLogSBPMessages(), config_->getLogPath());

    /* SBP Callback processing thread */
    sbp_thread_ = std::thread(&SBPROS2DriverNode::processSBP, this);
  }

  // Deleted methods
  SBPROS2DriverNode(const SBPROS2DriverNode&) = delete;
  SBPROS2DriverNode(SBPROS2DriverNode&&) = delete;
  SBPROS2DriverNode& operator=(const SBPROS2DriverNode&) = delete;
  SBPROS2DriverNode& operator=(SBPROS2DriverNode&&) = delete;

  /**
   * @brief Destroy the SBPROS2DriverNode object
   */
  ~SBPROS2DriverNode() {
    exit_requested_ = true;
    if (sbp_thread_.joinable()) sbp_thread_.join();
  }

  /**
   * @brief SBP messages processing thread
   */
  void processSBP() {
    while (!exit_requested_) {
      state_.process();
    }
  }

 private:
  /**
   * @brief Method for creating the data sources
   */
  void createDataSources() {
    data_source_ = dataSourceFactory(config_, logger_);
  }

  /**
   * @brief Method for creating the SBP to ROS2 publishers
   */
  void createPublishers() {
    auto frame = config_->getFrame();
    const auto publishers = config_->getPublishers();

    LOG_INFO(logger_, "Creating %u publishers", publishers.size());
    for (const auto& publisher : publishers) {
      LOG_INFO(logger_, "Adding publisher %s", publisher.c_str());
      pubs_manager_.add(publisherFactory(publisher, &state_, this, logger_,
                                         frame, config_, gnss_router_));
    }
  }

  /**
   * @brief Exposes ~/simulate_gnss_outage, replacing physically unplugging the antenna.
   *
   * While engaged, /navsatfix is silent and the same fixes go to the ground-truth topic,
   * so a mission can be flown "without GNSS" while the bag still records the truth.
   */
  void createGnssOutageService() {
    gnss_outage_service_ = this->create_service<std_srvs::srv::SetBool>(
        "~/simulate_gnss_outage",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          gnss_router_->setOutage(request->data);
          const std::string gt_topic = config_->getGnssGroundTruthTopic();
          response->success = true;
          response->message =
              request->data
                  ? "GNSS outage ENGAGED: /navsatfix suppressed, fixes on " +
                        gt_topic
                  : "GNSS outage CLEARED: fixes on /navsatfix";
          // Logged so /rosout carries the exact moment of the cut, which is a second,
          // independent marker for the offline video even if the ground-truth topic is
          // missed by the recorder.
          RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        });
    RCLCPP_INFO(this->get_logger(),
                "GNSS outage service ready; ground truth topic: %s (outage %s)",
                config_->getGnssGroundTruthTopic().c_str(),
                gnss_router_->inOutage() ? "ENGAGED" : "cleared");
  }

  sbp::State state_;           /** @brief SBP state object */
  std::thread sbp_thread_;     /** @brief SBP messages processing thread */
  bool exit_requested_{false}; /** @brief Thread stopping flag */
  std::shared_ptr<Config> config_;             /** @brief Node configuration */
  std::shared_ptr<SbpDataSource> data_source_; /** @brief data source object */
  std::shared_ptr<ROSLogger> logger_; /** @brief ROS 2 logging object */
  PublisherManager
      pubs_manager_; /** @brief Manager for all the active publishers */
  std::shared_ptr<SBPToROS2Logger>
      sbptoros2_; /** @brief SBP to ROS2 logging object */
  GnssRouterPtr gnss_router_; /** @brief Routes NavSatFix during a simulated outage */
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
      gnss_outage_service_; /** @brief ~/simulate_gnss_outage */
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SBPROS2DriverNode>());
  rclcpp::shutdown();

  return 0;
}
