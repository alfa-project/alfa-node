/*
 * Copyright 2023 ALFA Project. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ALFA_NODE_H
#define ALFA_NODE_H

// ROS includes
#include <linux/fs.h>
#include <linux/types.h>
#include <pthread.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Timming
#include <time.h>

#include <chrono>

// MATH
#include <cmath>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ALFA Includes
#include <errno.h>

#include "alfa_defines.hpp"
#include "alfa_msg/msg/alfa_alive_ping.hpp"
#include "alfa_msg/msg/alfa_metrics.hpp"
#include "alfa_msg/msg/config_message.hpp"
#include "alfa_msg/srv/alfa_configure.hpp"
#include "alfa_structs.hpp"

using namespace std;
// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(AlfaPoint,
                                  (float, x, x)(float, y, y)(float, z,
                                                             z)(std::uint32_t,
                                                                custom_field,
                                                                custom_field))

// AlfaNode class
class AlfaNode : public rclcpp::Node {
 public:
  AlfaNode(AlfaConfiguration conf, AlfaExtensionParameter *parameters,
           void (*handler_pointcloud)(AlfaNode *),
           void (*post_processing_pointcloud)(AlfaNode *));
  ~AlfaNode();

  float get_extension_parameter(string parameter_name);

  // Metric functions
  alfa_msg::msg::MetricMessage get_metric_message(int metric);

  // Publishing methods
  void publish_pointcloud(pcl::PointCloud<AlfaPoint>::Ptr pointcloud = nullptr,
                          std::uint32_t latency = 0);

  void publish_metrics(alfa_msg::msg::AlfaMetrics &metrics);

  // Pointcloud manipulation
  pcl::PointCloud<AlfaPoint>::Ptr get_input_pointcloud();
  pcl::PointCloud<AlfaPoint>::Ptr get_output_pointcloud();

  vector<AlfaPoint> get_input_pointcloud_as_vector();
  vector<AlfaPoint> get_output_pointcloud_as_vector();

  std::uint32_t get_input_pointcloud_size();
  std::uint32_t get_output_pointcloud_size();

  // Store and Load
  void store_pointcloud(int type,
                        pcl::PointCloud<AlfaPoint>::Ptr pointcloud = nullptr);
  void load_pointcloud(int type,
                       pcl::PointCloud<AlfaPoint>::Ptr pointcloud = nullptr);

  bool is_input_pointcloud_empty();
  bool is_output_pointcloud_empty();
  bool is_last_input_pointcloud_point();

  void push_point_output_pointcloud(AlfaPoint point);
  bool get_point_input_pointcloud(std::uint32_t position, AlfaPoint &point);
  AlfaPoint get_point_input_pointcloud(std::uint32_t position);
  AlfaPoint get_point_output_pointcloud(std::uint32_t position);
  bool get_point_input_pointcloud(AlfaPoint &point);
  bool reset_input_pointcloud_counter();
  bool set_custom_field_output_pointcloud(std::uint32_t position,
                                          std::uint32_t custom_field);

  // Multithreading
  void set_multi_thread(int n_threads, void (*func)(AlfaNode *), AlfaNode *);

  float get_debug_point(std::uint16_t);
  void set_debug_point(std::uint16_t, float, string);

 private:
  // Point clouds
  pcl::PointCloud<AlfaPoint>::Ptr input_pointcloud, output_pointcloud;
  AlfaConfiguration configuration;

  vector<AlfaExtensionParameter> extension_parameters;

  std::uint32_t point_counter;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_subscriber;
  rclcpp::Publisher<alfa_msg::msg::AlfaMetrics>::SharedPtr metrics_publisher;
  rclcpp::Publisher<alfa_msg::msg::AlfaAlivePing>::SharedPtr alive_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_publisher;

  // Files
  int fd1, fd2;

  // Memory mapping
  off_t unit_registers_address, pointcloud_ptr_address;

  // thread
  std::thread *ticker_thread, *pointcloud_publisher_thread, *alfa_main_thread;

  std::mutex input_mutex, output_mutex, output_counter_mutex, registers_mutex,
      ros_pointcloud_mutex, pcl2_frame_mutex, ros_pointcloud_condition_mutex,
      pcl2_frame_condition_mutex;

  std::condition_variable ros_pointcloud_condition, pcl2_frame_condition;

  void convert_msg_to_pointcloud();

  // callbacks
  void handler_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  // user-defined functions
  void (*handler_pointcloud)(AlfaNode *);
  void (*post_processing_pointcloud)(AlfaNode *);

  // thread handlers
  void ticker_alive();

  // metrics
  AlfaMetric handler_metric, full_processing_metric, publishing_metric,
      number_of_processed_points;

  alfa_msg::msg::MetricMessage debug_points_message[20];

  AlfaUnitRegisters *unit_registers;

  AlfaPointcloud pointcloud;

  std::deque<sensor_msgs::msg::PointCloud2> pcl2_frame;
  std::deque<sensor_msgs::msg::PointCloud2> ros_pointcloud;

  bool hardware_setup();

  // Metrics functions
  void metrics_setup();
  void metrics_update();
  void metrics_publish();

  alfa_msg::msg::MetricMessage get_handler_time();
  alfa_msg::msg::MetricMessage get_full_processing_time();

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle;
  rcl_interfaces::msg::SetParametersResult parameters_callback(
      const std::vector<rclcpp::Parameter> &parameters);

  void pointcloud_publisher_thread_handler();
  void alfa_main_thread_handler();

  void publish_pointcloud(sensor_msgs::msg::PointCloud2 &pointcloud);

  // Hardware storage and loading
  void store_pointcloud_cartesian(pcl::PointCloud<AlfaPoint>::Ptr pointcloud);
  void load_pointcloud_cartesian(pcl::PointCloud<AlfaPoint>::Ptr pointcloud);

  // Verbose functions
  void verbose_constr_chracteristics(string, string);
  void verbose_begin(string);
  void verbose_end(string);
  void verbose_ok(string, string);
  void verbose_fail(string, string);
  void verbose_info(string, string);
  void verbose_not_defined(string);
};

#endif  // ALFA_NODE_H