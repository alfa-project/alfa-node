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

#ifndef ALFA_STRUCTS_H
#define ALFA_STRUCTS_H

#include <pcl/point_types.h>
#include <chrono>
#include <string>

#include "alfa_msg/msg/alfa_metrics.hpp"

using namespace std;

struct AlfaPoint {
  PCL_ADD_POINT4D;  // quad-word XYZ
  std::uint32_t custom_field;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct AlfaHardwareSupport {
  bool hardware_driver;
  bool hardware_extension;
};

struct AlfaExtensionParameter {
  string parameter_name;
  double parameter_value;
};

struct AlfaUnitStatus {
  std::uint32_t cu;
  std::uint32_t exmu;
  std::uint32_t memmu;
  std::uint32_t monu;
  std::uint32_t siu;
  std::uint32_t ext;
};

struct AlfaUnitParameters {
  std::uint32_t cu;
  std::uint32_t exmu;
  std::uint32_t memmu;
  std::uint32_t monu;
  std::uint32_t siu;
};

struct AlfaUnitHardwareSignals {
  std::uint32_t ext_ready;
  std::uint32_t pointcloud_ready;
  std::uint32_t processing_done;
};

struct AlfaUnitSoftwareSignals {
  std::uint32_t pointcloud_ready;
  std::uint32_t processing_done;
  std::uint32_t pointcloud_size;
  std::uint32_t frame_done;
};

struct AlfaUnitDebugPoints {
  std::int32_t value[20];
};

struct AlfaUnitUserDefines {
  std::int32_t value[10];
};

struct AlfaPointcloud {
  std::uint32_t *size;
  std::uint64_t *ptr;
};

struct AlfaUnitRegisters {
  AlfaUnitStatus status;
  std::uint32_t nd0;
  AlfaUnitHardwareSignals hardware_signals;
  AlfaUnitDebugPoints debug_points;
  AlfaUnitParameters parameters;
  AlfaUnitSoftwareSignals software_signals;
  std::uint32_t nd1;
  AlfaUnitUserDefines user_defines;
};

struct AlfaConfiguration {
  string subscriber_topic;
  string node_name;
  std::uint32_t pointcloud_id;
  std::uint32_t extension_id;
  AlfaHardwareSupport hardware_support;
  std::uint32_t latency;
  std::uint32_t metrics_publishing_type;
  std::uint32_t custom_field_conversion_type;
  std::uint32_t number_of_debug_points;
};

struct AlfaMetric {
  std::chrono::_V2::system_clock::time_point start, stop;
  alfa_msg::msg::MetricMessage message;
};

#endif  // ALFA_STRUCTS_H