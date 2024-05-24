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

#include "alfa_node.hpp"

extern "C" {
#include <sys/ioctl.h>
}

// Define custom ioctl commands
#define ALFA_IOCTL_TRIGGER_READ _IO('k', 1)
#define ALFA_IOCTL_TRIGGER_WRITE _IO('k', 2)
#define ALFA_IOCTL_CHECK_MEM _IO('k', 3)

using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;

// Global flag to signal threads to exit
std::atomic<bool> g_running(true);

// Signal handler for SIGINT
void signalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << endl << "SIGINT received, exiting..." << std::endl;
    g_running = false;
  }
}

// Constructor
AlfaNode::AlfaNode(AlfaConfiguration conf,
                   AlfaExtensionParameter parameters[10],
                   void (*handler_pointcloud)(AlfaNode *) = NULL,
                   void (*post_processing_pointcloud)(AlfaNode *) = NULL)
    : Node(conf.node_name),
      point_counter(0),
      handler_pointcloud(handler_pointcloud),
      post_processing_pointcloud(post_processing_pointcloud) {
  // Set the configuration and the signal handler
  configuration = conf;
  std::signal(SIGINT, signalHandler);

  // Print the characteristics of the node
  verbose_constr_chracteristics(conf.subscriber_topic, conf.node_name);

#ifdef ALFA_VERBOSE
  verbose_begin("constructor");
#endif

  // If SIU is activated no subscriber is required to retrieve pointclouds
  if (!configuration.hardware_support.hardware_driver) {
    pointcloud_subscriber =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            conf.subscriber_topic,
            10,  // Subscribe to the desired topic
                 // std::bind(&AlfaNode::handler_callback, this, _1));
            std::bind(&AlfaNode::handler_callback, this, _1));
#ifdef ALFA_VERBOSE
    verbose_ok("constructor", "setup handler_callback");
#endif
  }

  // Create ALFA topics
  metrics_publisher = this->create_publisher<alfa_msg::msg::AlfaMetrics>(
      string(conf.node_name).append("_metrics"), 1);
  alive_publisher = this->create_publisher<alfa_msg::msg::AlfaAlivePing>(
      string(conf.node_name).append("_alive"), 1);
  pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      string(conf.node_name).append("_pointcloud"), 1);

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "create ALFA topics");
#endif

  // Create the pointcloud deque
  ros_pointcloud.clear();

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "ros pointcloud");
#endif

  input_pointcloud.reset(new pcl::PointCloud<AlfaPoint>);
  output_pointcloud.reset(new pcl::PointCloud<AlfaPoint>);

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "create pcl pointclouds");
#endif

  metrics_setup();

#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "set metrics");
#endif

  if (configuration.hardware_support.hardware_extension ||
      configuration.hardware_support.hardware_driver)
    if (hardware_setup() == 1) {
      verbose_fail("constructor", "hardware_setup");
      abort();
      return;
    };
  // Publisher thread
  pointcloud_publisher_thread =
      new std::thread(&AlfaNode::pointcloud_publisher_thread_handler, this);
#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "publisher");
#endif

  // Parameters config
  for (int i = 0; i < 10; i++) {
    if (parameters[i].parameter_name !=
        "") {  // If the parameter is actually defined
      this->extension_parameters.push_back(parameters[i]);
      this->declare_parameter(parameters[i].parameter_name,
                              (double)parameters[i].parameter_value);
      if (configuration.hardware_support.hardware_extension ||
          configuration.hardware_support.hardware_driver) {
        registers_mutex.lock();
        unit_registers->user_defines.value[i] =
            (extension_parameters[i].parameter_value * FIXED_POINT_MULTIPLIER);
        registers_mutex.unlock();
      }
    }
  }

  ticker_thread = new std::thread(
      &AlfaNode::ticker_alive,
      this);  // Start the ticker thread that sends the alive message
  alfa_main_thread = new std::thread(&AlfaNode::alfa_main_thread_handler, this);

  // Set thread affinity for pointcloud_publisher_thread
  cpu_set_t cpuset_publisher;
  CPU_ZERO(&cpuset_publisher);
  CPU_SET(2, &cpuset_publisher);  // Set the CPU core for this thread

  int result_publisher =
      pthread_setaffinity_np(pointcloud_publisher_thread->native_handle(),
                             sizeof(cpu_set_t), &cpuset_publisher);
  if (result_publisher != 0) {
    std::cerr
        << "Error setting thread affinity for pointcloud_publisher_thread: "
        << strerror(result_publisher) << std::endl;
    return;
  }

  // Set thread affinity for ticker_thread
  cpu_set_t cpuset_ticker;
  CPU_ZERO(&cpuset_ticker);
  CPU_SET(3, &cpuset_ticker);  // Set the CPU core for this thread

  int result_ticker = pthread_setaffinity_np(ticker_thread->native_handle(),
                                             sizeof(cpu_set_t), &cpuset_ticker);
  if (result_ticker != 0) {
    std::cerr << "Error setting thread affinity for ticker_thread: "
              << strerror(result_ticker) << std::endl;
    return;
  }

  // Set thread affinity for alfa_main_thread
  cpu_set_t cpuset_alfa_main;
  CPU_ZERO(&cpuset_alfa_main);
  CPU_SET(1, &cpuset_alfa_main);  // Set the CPU core for this thread

  int result_alfa_main = pthread_setaffinity_np(
      alfa_main_thread->native_handle(), sizeof(cpu_set_t), &cpuset_alfa_main);
  if (result_alfa_main != 0) {
    std::cerr << "Error setting thread affinity for alfa_main_thread: "
              << strerror(result_alfa_main) << std::endl;
    return;
  }

  pointcloud_publisher_thread->detach();
  ticker_thread->detach();
  alfa_main_thread->detach();
#ifdef ALFA_VERBOSE
  verbose_ok("constructor", "create threads");
  verbose_end("constructor");
#endif
}

AlfaNode::~AlfaNode() {
  delete this->ticker_thread;
  delete this->alfa_main_thread;
  delete this->pointcloud_publisher_thread;

  if (configuration.hardware_support.hardware_extension ||
      configuration.hardware_support.hardware_driver) {
    close(fd1);
    close(fd2);
  }
}

bool AlfaNode::hardware_setup() {
  bool return_value = 1;

  if (((fd1 = open("/dev/mem", O_RDWR | O_SYNC)) != -1) &&
      ((fd2 = open("/dev/alfa", O_RDWR | O_SYNC)) != -1)) {
    return_value = 0;
    unit_registers_address =
        HARDWARE_EXT_BASE_ADDR +
        configuration.extension_id * HARDWARE_EXT_BASE_PER_ID;
    pointcloud_ptr_address =
        configuration.pointcloud_id * POINTCLOUD_BASE_PER_ID;

    this->unit_registers =
        (AlfaUnitRegisters *)mmap(0x0, 0x1000, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, fd1, unit_registers_address);
    this->pointcloud.size =
        &(this->unit_registers->software_signals.pointcloud_size);
    this->pointcloud.ptr = (std::uint64_t *)mmap(
        0x0, POINTCLOUD_BASE_PER_ID, PROT_READ | PROT_WRITE, MAP_SHARED, fd2,
        pointcloud_ptr_address);
    // this->pointcloud.ptr_non_cachable = (std::uint64_t
    // *)mmap(MMAP_NON_CACHABLE, POINTCLOUD_BASE_PER_ID, PROT_READ |
    // PROT_WRITE, MAP_SHARED, fd2, pointcloud_ptr_address);
    if (this->pointcloud.ptr == MAP_FAILED)
      return_value = 1;
    else {
      //  Create parameter's callbacks
      parameters_callback_handle =
          this->add_on_set_parameters_callback(std::bind(
              &AlfaNode::parameters_callback, this, std::placeholders::_1));
    }
  }

#ifdef ALFA_VERBOSE
  if (return_value)
    verbose_fail("constructor", "mem pointers");
  else
    verbose_ok("constructor", "mem pointers");
#endif
  return return_value;
}

void AlfaNode::handler_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  if ((cloud->width * cloud->height) == 0) {
    return;
  } else {
    if (ros_pointcloud.size() > 3) {
      ros_pointcloud_mutex.lock();
      ros_pointcloud.pop_front();
      ros_pointcloud_mutex.unlock();
    } else {
      std::lock_guard<std::mutex> lk(ros_pointcloud_condition_mutex);
      ros_pointcloud_mutex.lock();
      ros_pointcloud.push_back(*cloud);
      ros_pointcloud_mutex.unlock();
      ros_pointcloud_condition.notify_one();
    }

#ifdef ALFA_VERBOSE
    verbose_info("handler_callback",
                 "Received a point cloud with: " +
                     std::to_string(cloud->width * cloud->height) + " size");
#endif
  }
}

#pragma GCC optimize("prefetch-loop-arrays")
void AlfaNode::convert_msg_to_pointcloud() {
  // Lock and retrieve the front element of the deque
  ros_pointcloud_mutex.lock();
  auto ros_pointcloud_temp = ros_pointcloud.front();
  ros_pointcloud.pop_front();
  ros_pointcloud_mutex.unlock();

  const auto &data = ros_pointcloud_temp.data;
  const auto &fields = ros_pointcloud_temp.fields;

  // Find the field in the message to convert into custom field
  int field_offset = -1;  // Initialize to an invalid value
  int field_offset_2 = -1;

  int field_type = 0;
  int field_type_2 = 0;

  auto custom_type = this->configuration.custom_field_conversion_type;

  // Common loop for finding the field offsets based on field names
  auto findFieldOffsetByName = [&](const std::string &fieldName = "",
                                   const std::string &fieldName2 = "") {
    for (const auto &field : fields) {
      if (fieldName == "")
        break;
      else if (field.name == fieldName) {
        field_offset = field.offset;
        field_type = field.datatype;
        break;
      }
    }

    for (const auto &field : fields) {
      if (fieldName == "")
        break;
      else if (field.name == fieldName2) {
        field_offset_2 = field.offset;
        field_type_2 = field.datatype;
        break;
      }
    }
  };

  switch (custom_type) {
    case CUSTOM_FIELD_USER:
      findFieldOffsetByName("USER");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is USER");
#endif
      break;

    case CUSTOM_FIELD_INTENSITY:
      findFieldOffsetByName("intensity");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is intensity");
#endif
      break;

    case CUSTOM_FIELD_LABEL:
      findFieldOffsetByName("label");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is label");
#endif
      break;

    case CUSTOM_FIELD_RGB:
      findFieldOffsetByName("rgb");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is rgb");
#endif
      break;

    case CUSTOM_FIELD_FILTER:
      findFieldOffsetByName("intensity");
#ifdef ALFA_VERBOSE
      verbose_info("main_thread",
                   "convert_msg_to_pointcloud: field name is INTENSITY");
#endif
      break;

    case CUSTOM_FIELD_INTENSITY_LABEL:
      findFieldOffsetByName("intensity", "label");
#ifdef ALFA_VERBOSE
      verbose_info(
          "main_thread",
          "convert_msg_to_pointcloud: field name is INTENSITY and LABEL");
#endif
      break;

    default:
      findFieldOffsetByName("intensity");
#ifdef ALFA_VERBOSE
      verbose_info(
          "main_thread",
          "convert_msg_to_pointcloud: field name is default: INTENSITY");
#endif
      break;
  }

#ifdef ALFA_VERBOSE
  verbose_info("main_thread",
               "convert_msg_to_pointcloud: offset calculation ok");
#endif
  // Clear the existing data in input_pointcloud and reserve memory
  input_pointcloud->clear();
  input_pointcloud->reserve(ros_pointcloud_temp.width *
                            ros_pointcloud_temp.height);

#ifdef ALFA_VERBOSE
  verbose_info("main_thread", "convert_msg_to_pointcloud: converting points");
  cout << "data.size(): " << data.size() << endl;
  cout << "ros_pointcloud_temp.point_step: " << ros_pointcloud_temp.point_step
       << endl;
#endif

  // Iterate through the data and populate custom point cloud
  for (size_t i = 0; i < data.size(); i += ros_pointcloud_temp.point_step) {
    AlfaPoint point;
    // Extract x, y, z from the data (assuming these fields are at offsets
    // 0, 4, and 8 respectively)
    memcpy(&(point.x), &data[i + fields[0].offset], sizeof(float));
    memcpy(&(point.y), &data[i + fields[1].offset], sizeof(float));
    memcpy(&(point.z), &data[i + fields[2].offset], sizeof(float));

    point.custom_field = 0;

    // Extract the custom field from the data
    auto fillCustomField = [&](const std::uint32_t &field_offset,
                               const std::uint32_t &field_type,
                               const bool &byte_selector) {
      switch (field_type) {
        case sensor_msgs::msg::PointField::FLOAT32: {  // Intensity
          float temp_float;
          memcpy(&temp_float, &data[i + field_offset], sizeof(float));
          uint8_t temp_8;
          // Handle intensity values in 0-1 range and 0-100 range
          if (temp_float <= 1) {
            temp_8 = static_cast<uint8_t>(temp_float * 255);
          } else
            temp_8 = static_cast<uint8_t>(temp_float * 255 / 100);
          if (byte_selector)
            point.custom_field |= temp_8 << 8;
          else
            point.custom_field |= temp_8;
          break;
        }

        case sensor_msgs::msg::PointField::UINT16: {  // Label
          std::uint16_t temp_uint16;
          memcpy(&temp_uint16, &data[i + field_offset], sizeof(std::uint16_t));
          uint8_t temp_8;
          // Handle labels values from 0 to 255
          if (temp_uint16 < 252)
            temp_8 = static_cast<uint8_t>(temp_uint16);
          else
            temp_8 = static_cast<uint8_t>(temp_uint16 - 100);
          if (byte_selector)
            point.custom_field |= temp_8 << 8;
          else
            point.custom_field |= temp_8;
          break;
        }

        default:
          point.custom_field = 0;
          break;
      }
    };

    // Fill the custom field
    if (field_offset != -1) fillCustomField(field_offset, field_type, true);
    if (field_offset_2 != -1)
      fillCustomField(field_offset_2, field_type_2, false);
    input_pointcloud->emplace_back(std::move(point));  // Use emplace_back
  }

#ifdef ALFA_VERBOSE
  verbose_info("main_thread", "convert_msg_to_pointcloud: done");
#endif
}

void AlfaNode::alfa_main_thread_handler() {
  while (rclcpp::ok() && g_running) {
    std::unique_lock<std::mutex> lk(ros_pointcloud_condition_mutex);
    ros_pointcloud_condition.wait(
        lk, [this] { return !ros_pointcloud.empty() || !g_running; });
    if (!g_running) {
      return;
    }

#ifdef ALFA_VERBOSE
    verbose_info("main_thread", "New pointcloud received");
#endif
    full_processing_metric.start = high_resolution_clock::now();
#ifdef ALFA_VERBOSE
    verbose_info("main_thread", "Converting ros2 msg to pcl");
#endif
    convert_msg_to_pointcloud();
    this->point_counter = 0;
    handler_metric.start = high_resolution_clock::now();
    output_pointcloud.reset(new pcl::PointCloud<AlfaPoint>);
#ifdef ALFA_VERBOSE
    verbose_info("main_thread", "Calling user-defined handler");
#endif
    (*handler_pointcloud)(this);

    if (!configuration.hardware_support.hardware_extension) {
      handler_metric.stop = high_resolution_clock::now();

#ifdef ALFA_VERBOSE
      verbose_info("main_thread", "Calling user-defined post processing");
#endif
      (*post_processing_pointcloud)(this);
#ifdef ALFA_VERBOSE
      verbose_info("main_thread", "Return from user-defined post processing");
#endif
      full_processing_metric.stop = high_resolution_clock::now();
    } else {
      this->unit_registers->software_signals.frame_done = 0;
      this->unit_registers->software_signals.pointcloud_ready = 1;
      // pcl::fromROSMsg(ros_pointcloud_temp, *output_pointcloud);
#ifdef ALFA_VERBOSE
      verbose_info("main_thread", "calling hardware processing");
#endif
      unsigned int watchdog_counter = 0;
      while (!(unit_registers->hardware_signals.processing_done &&
               unit_registers->software_signals.pointcloud_ready) &&
             watchdog_counter < 4500) {
        watchdog_counter++;
        std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      }

      registers_mutex.lock();
      this->unit_registers->software_signals.pointcloud_ready = 0;
      registers_mutex.unlock();
      handler_metric.stop = high_resolution_clock::now();

#ifdef ALFA_VERBOSE
      verbose_info("main_thread", "done");
      verbose_info("main_thread", "calling post processing");
#endif
      (*post_processing_pointcloud)(this);
      registers_mutex.lock();
      this->unit_registers->software_signals.frame_done = 1;
      registers_mutex.unlock();
      full_processing_metric.stop = high_resolution_clock::now();
    }
    metrics_update();
    metrics_publish();
  }
}

void AlfaNode::ticker_alive() {
  while (rclcpp::ok() && g_running) {
    alfa_msg::msg::AlfaAlivePing new_ping;
    new_ping.node_name = (string)this->get_name();
    new_ping.node_type = "extension";
    new_ping.config_service_name = (string)this->get_name() + "_settings";
    new_ping.config_tag = "Configuration";

    new_ping.default_configurations.clear();
    for (AlfaExtensionParameter parameter : this->extension_parameters) {
      alfa_msg::msg::ConfigMessage msg_parameter;

      msg_parameter.config_name = parameter.parameter_name;
      msg_parameter.config = parameter.parameter_value;

      new_ping.default_configurations.push_back(msg_parameter);
    }

    new_ping.current_status =
        (int)(this->configuration.hardware_support.hardware_driver +
              this->configuration.hardware_support.hardware_extension);
    alive_publisher->publish(new_ping);
    std::this_thread::sleep_for(std::chrono::milliseconds(ALIVE_TIMER_SLEEP));
  }
  ros_pointcloud_condition.notify_one();
  pcl2_frame_condition.notify_one();
  cout << endl << "ALFA Extension Shutdown..." << endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  rclcpp::shutdown();
}

// Returns the parameter's value using its name
float AlfaNode::get_extension_parameter(string parameter_name) {
#ifdef ALFA_VERBOSE
  verbose_info("get_extension_parameter",
               "getting parameter: " + parameter_name);
#endif
  return this->get_parameter(parameter_name).get_parameter_value().get<float>();
}

void AlfaNode::pointcloud_publisher_thread_handler() {
  while (rclcpp::ok() && g_running) {
    std::unique_lock<std::mutex> lk(pcl2_frame_condition_mutex);
    pcl2_frame_condition.wait(
        lk, [this] { return !pcl2_frame.empty() || !g_running; });

    if (!g_running) {
      return;
    }

    auto temp = high_resolution_clock::now();

    auto latency = std::chrono::milliseconds(configuration.latency);
    pcl2_frame_mutex.lock();
    auto pcl2_frame_temp = pcl2_frame.front();
    pcl2_frame.pop_front();
    pcl2_frame_mutex.unlock();

    /*while (start_temp + latency > high_resolution_clock::now())
    {
            std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    }*/
    publish_pointcloud(pcl2_frame_temp);
    publishing_metric.start = temp;
    publishing_metric.stop = high_resolution_clock::now();
  }
}

// Method for publishing a PointCloud2 type pointcloud
void AlfaNode::publish_pointcloud(sensor_msgs::msg::PointCloud2 &pointcloud) {
#ifdef ALFA_VERBOSE
  verbose_info("publish_pointcloud", "publishing pointcloud");
#endif
  pointcloud.header.frame_id =
      (string)this->get_name() +
      "_pointcloud";  // Create the pointcloud2 header to publish
  pointcloud.header.stamp = this->now();  // Get current time
  pointcloud_publisher->publish(
      pointcloud);  // Publish the point cloud in the ROS topic
}

// Method for publishing a PCL type pointcloud
void AlfaNode::publish_pointcloud(pcl::PointCloud<AlfaPoint>::Ptr pointcloud,
                                  std::uint32_t latency) {
  if (pointcloud == nullptr) pointcloud = this->output_pointcloud;
  this->configuration.latency = latency;
  sensor_msgs::msg::PointCloud2 pcl2_frame_temp;
  pcl::toROSMsg(
      *pointcloud,
      pcl2_frame_temp);  // convert the pcl object to the pointcloud2 one
  std::lock_guard<std::mutex> lk(pcl2_frame_condition_mutex);
  pcl2_frame_mutex.lock();
  pcl2_frame.push_back(pcl2_frame_temp);
  pcl2_frame_mutex.unlock();
  pcl2_frame_condition.notify_one();
}

// Method for publishing a AlfaMetrics object
void AlfaNode::publish_metrics(alfa_msg::msg::AlfaMetrics &metrics) {
#ifdef ALFA_VERBOSE
  verbose_info("publish_metrics", "publishing metrics");
#endif
  metrics_publisher->publish(metrics);  // publish the metrics
}

pcl::PointCloud<AlfaPoint>::Ptr AlfaNode::get_input_pointcloud() {
#ifdef ALFA_VERBOSE
  verbose_info("get_input_pointcloud", "getting input pointcloud");
#endif
  return this->input_pointcloud;
}

pcl::PointCloud<AlfaPoint>::Ptr AlfaNode::get_output_pointcloud() {
#ifdef ALFA_VERBOSE
  verbose_info("get_output_pointcloud", "getting output pointcloud");
#endif
  return this->output_pointcloud;
}

std::vector<AlfaPoint> AlfaNode::get_input_pointcloud_as_vector() {
#ifdef ALFA_VERBOSE
  verbose_info("get_input_pointcloud", "getting input pointcloud");
#endif
  std::vector<AlfaPoint> r_input_pointcloud;

  for (const auto &point : *input_pointcloud)
    r_input_pointcloud.push_back(point);

  return r_input_pointcloud;
}

std::vector<AlfaPoint> AlfaNode::get_output_pointcloud_as_vector() {
#ifdef ALFA_VERBOSE
  verbose_info("get_output_pointcloud", "getting output pointcloud");
#endif
  std::vector<AlfaPoint> r_output_pointcloud;

  for (const auto &point : *output_pointcloud)
    r_output_pointcloud.push_back(point);

  return r_output_pointcloud;
}

void AlfaNode::store_pointcloud(int type,
                                pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (pointcloud == nullptr) pointcloud = this->input_pointcloud;
  switch (type) {
    case LOAD_STORE_CARTESIAN:
      store_pointcloud_cartesian(pointcloud);
      break;
    case LOAD_STORE_SPHERICAL:
      // store_pointcloud_spherical(pointcloud);
      break;

    default:
      store_pointcloud_cartesian(pointcloud);
      break;
  }
}

void AlfaNode::store_pointcloud_cartesian(
    pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (configuration.hardware_support.hardware_extension) {
    // Pointer to the memory where point cloud data will be stored
    std::uint64_t *mem = this->pointcloud.ptr;

    // Calculate the size of the point cloud
    const size_t num_points = pointcloud->size();

    // Reserve memory for the point cloud data
    std::vector<std::int16_t> a16_points(num_points *
                                         4);  // Assuming 4 elements per point

    // Convert and pack points efficiently
    for (size_t i = 0; i < num_points; ++i) {
      const AlfaPoint &point = (*pointcloud)[i];
      std::int16_t *a16_point = &a16_points[i * 4];

      // Perform all conversions at once to minimize cache misses
      a16_point[0] = static_cast<std::int16_t>(
          std::round(point.x * FIXED_POINT_MULTIPLIER));
      a16_point[1] = static_cast<std::int16_t>(
          std::round(point.y * FIXED_POINT_MULTIPLIER));
      a16_point[2] = static_cast<std::int16_t>(
          std::round(point.z * FIXED_POINT_MULTIPLIER));
      a16_point[3] = static_cast<std::int16_t>(point.custom_field);
    }

    // Copy the packed data to the memory
    memcpy(mem, a16_points.data(), sizeof(std::int16_t) * 4 * num_points);

    // Update the point cloud size
    (*this->pointcloud.size) = num_points;

    // Synchronize memory with hardware
    msync(mem, sizeof(std::int16_t) * 4 * num_points, MS_SYNC);

    // Trigger write using IOCTL (assuming fd2 is a valid file descriptor)
    size_t size = (*this->pointcloud.size) * sizeof(std::uint64_t);
    if (ioctl(fd2, ALFA_IOCTL_TRIGGER_WRITE, &size) == -1) {
      verbose_fail("store_pointcloud_cartesian", "ioctl failed");
      while (1);
    }

#ifdef ALFA_VERBOSE
    verbose_info("store_pointcloud_cartesian",
                 "stored " + std::to_string(num_points) + " points");
    verbose_info("store_pointcloud_cartesian", "last point");
    verbose_info(
        "store_pointcloud_cartesian",
        "fixed point multiplier: " + std::to_string(FIXED_POINT_MULTIPLIER));
    verbose_info("store_pointcloud_cartesian",
                 "x: " + std::to_string(a16_points[(num_points - 1) * 4]));
    verbose_info("store_pointcloud_cartesian",
                 "y: " + std::to_string(a16_points[(num_points - 1) * 4 + 1]));
    verbose_info("store_pointcloud_cartesian",
                 "z: " + std::to_string(a16_points[(num_points - 1) * 4 + 2]));
#endif
  } else
    verbose_not_defined("store_pointcloud_cartesian");
}

void AlfaNode::load_pointcloud(int type,
                               pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (pointcloud == nullptr) pointcloud = this->output_pointcloud;
  switch (type) {
    case LOAD_STORE_CARTESIAN:
      load_pointcloud_cartesian(pointcloud);
      break;
    case LOAD_STORE_SPHERICAL:
      // load_pointcloud_spherical(pointcloud);
      break;

    default:
      load_pointcloud_cartesian(pointcloud);
      break;
  }
}

// Hardware loading----------------------------------
#pragma GCC optimize("prefetch-loop-arrays")
void AlfaNode::load_pointcloud_cartesian(
    pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (configuration.hardware_support.hardware_extension) {
    auto custom_type = this->configuration.custom_field_conversion_type;

    size_t size = (*this->pointcloud.size) * sizeof(std::uint64_t);
    if (ioctl(fd2, ALFA_IOCTL_TRIGGER_READ, &size) == -1) {
      verbose_fail("load_pointcloud_cartesian", "ioctl failed");
      while (1);
    }

    // Define constants
    const std::uint32_t POINT_STRIDE = 4;  // Assuming 4 elements per point

    // Calculate the number of points
    const std::uint32_t num_points = (*this->pointcloud.size);

    // Reserve memory for the point cloud to avoid reallocations
    pointcloud->reserve(num_points);

    // Pointer to the custom data (skip the first 3 elements)
    std::int16_t *custom_ptr = (std::int16_t *)this->pointcloud.ptr + 3;

    // Index variables
    std::uint32_t i = 0;

    // Clear the point cloud
    pointcloud->clear();

    switch (custom_type) {
      case CUSTOM_FIELD_FILTER:
        for (const auto &point : *input_pointcloud) {
          if (custom_ptr[i * POINT_STRIDE] == FILTER_VALUE) {
            pointcloud->emplace_back(point);  // Use emplace_back for efficiency
          }
          i++;
        }
        break;

      default:
        for (const auto &point : *input_pointcloud) {
          pointcloud->emplace_back(point);  // Use emplace_back for efficiency
        }
        break;
    }

#ifdef ALFA_VERBOSE
    verbose_info("load_pointcloud_cartesian",
                 "loaded " + std::to_string(num_points) + " points");
    verbose_info("load_pointcloud_cartesian", "last point");
    verbose_info(
        "load_pointcloud_cartesian",
        "fixed point multiplier: " + std::to_string(FIXED_POINT_MULTIPLIER));
    if (!pointcloud->empty()) {
      const AlfaPoint &lastPoint = pointcloud->back();
      verbose_info("load_pointcloud_cartesian",
                   "x: " + std::to_string(lastPoint.x));
      verbose_info("load_pointcloud_cartesian",
                   "y: " + std::to_string(lastPoint.y));
      verbose_info("load_pointcloud_cartesian",
                   "z: " + std::to_string(lastPoint.z));
    }
#endif
  } else
    verbose_not_defined("load_pointcloud_cartesian");
}

float AlfaNode::get_debug_point(std::uint16_t number) {
  if (number <= configuration.number_of_debug_points)
    if (configuration.hardware_support.hardware_extension)
      return (static_cast<float>(unit_registers->debug_points.value[number]));
    else
      return debug_points_message[number].metric;
  else {
    verbose_not_defined("get_debug_point");
    return 1;
  }
}

void AlfaNode::set_debug_point(std::uint16_t number, float value,
                               string tag = "") {
  if (number <= configuration.number_of_debug_points &&
      !configuration.hardware_support.hardware_extension) {
    debug_points_message[number].metric = value;
    if (tag != "") debug_points_message[number].metric_name = tag;
  } else
    verbose_not_defined("get_debug_point");
}

// Parameters Callback ----------------------------------
rcl_interfaces::msg::SetParametersResult AlfaNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  if (configuration.hardware_support.hardware_extension) {
    result.successful = true;
    result.reason = "success";

    for (const auto &param :
         parameters)  // Go through all the changed parameters
    {
      for (std::uint16_t i = 0; i < extension_parameters.size();
           i++)  // Go through all current parameters, detect the changed
                 // one and change in memory
      {
        if (param.get_name() == extension_parameters[i].parameter_name) {
          unit_registers->user_defines.value[i] =
              static_cast<int32_t>(param.as_double() * FIXED_POINT_MULTIPLIER);
        }
      }
    }

    return result;
  } else {
    verbose_not_defined("parameters_callback");
    result.successful = false;
    return result;
  }
}

// Pointcloud manipulation----------------------------------

// Returns the size of the input_pointcloud member
std::uint32_t AlfaNode::get_input_pointcloud_size() {
  return (std::uint32_t)input_pointcloud->size();
}

// Returns the size of the output_pointcloud member
std::uint32_t AlfaNode::get_output_pointcloud_size() {
  return (std::uint32_t)output_pointcloud->size();
}

// Returns true if the input_pointcloud is empty
bool AlfaNode::is_input_pointcloud_empty() {
  if (get_input_pointcloud_size() == 0)
    return true;
  else
    return false;
}

// Returns true if the output_pointcloud is empty
bool AlfaNode::is_output_pointcloud_empty() {
  if (get_output_pointcloud_size() == 0)
    return true;
  else
    return false;
}

// Returns true if the point_counter is pointing to the last point of the
// input_pointcloud, meaning the next call of
// get_point_sequentially_input_pointcloud(), will get the last point
bool AlfaNode::is_last_input_pointcloud_point() {
  if (is_input_pointcloud_empty() ||
      point_counter >= (get_input_pointcloud_size() - 1))
    return true;
  else
    return false;
}

void AlfaNode::push_point_output_pointcloud(AlfaPoint point) {
  output_mutex.lock();
  output_pointcloud->push_back(point);
  output_mutex.unlock();
}

// Get specific point from input cloud, using the point argument. Returns true
// if successfull, false otherwise
bool AlfaNode::get_point_input_pointcloud(std::uint32_t position,
                                          AlfaPoint &point) {
  if (position < get_input_pointcloud_size() - 1) {
    point = (*input_pointcloud)[position];
    return true;
  }

  return false;
}

// Get specific point from input cloud, using the point index. Returns the point
// if successfull, the last point otherwise
AlfaPoint AlfaNode::get_point_input_pointcloud(std::uint32_t position) {
  if (position <= get_input_pointcloud_size() - 1) {
    return (*input_pointcloud)[position];
  } else
    return (*input_pointcloud)[get_input_pointcloud_size() - 1];
}

// Multiple calls, give points from the input_pointcloud squentially, return
// true if there are points left, false otherwise
bool AlfaNode::get_point_input_pointcloud(AlfaPoint &point) {
  if (point_counter < get_input_pointcloud_size()) {
    input_mutex.lock();
    point = (*input_pointcloud)[point_counter++];
    input_mutex.unlock();

    return true;
  }

  return false;
}

// Set specific custom field value in the output_pointcloud position, returns
// true if successfull, false otherwise
bool AlfaNode::set_custom_field_output_pointcloud(std::uint32_t position,
                                                  std::uint32_t value) {
  if (position < (get_output_pointcloud_size() - 1)) {
    (*output_pointcloud)[position].custom_field = value;
    return true;
  }

  return false;
}

bool AlfaNode::reset_input_pointcloud_counter() {
  if (is_input_pointcloud_empty()) return false;

  point_counter = 0;
  return true;
}

void AlfaNode::set_multi_thread(int n_threads, void (*func)(AlfaNode *),
                                AlfaNode *ptr) {
  vector<std::thread *> thread_list;
  thread_list.clear();

  for (int i = 0; i < n_threads; i++)
    thread_list.push_back(new std::thread(func, ptr));

  for (int i = 0; i < n_threads; i++) thread_list[i]->join();
}

// Metric functions
void AlfaNode::metrics_update() {
  auto duration_full_processing = duration_cast<microseconds>(
      full_processing_metric.stop - full_processing_metric.start);
  auto duration_handler =
      duration_cast<microseconds>(handler_metric.stop - handler_metric.start);
  auto duration_publishing = duration_cast<microseconds>(
      publishing_metric.stop - publishing_metric.start);

  /*FIXME*/ if (duration_handler.count() < 200000)
    handler_metric.message.metric = duration_handler.count();
  publishing_metric.message.metric = duration_publishing.count();
  full_processing_metric.message.metric =
      duration_full_processing.count() + duration_publishing.count();
  number_of_processed_points.message.metric = input_pointcloud->size();
}

void AlfaNode::metrics_setup() {
  // Metrics
  handler_metric.message.units = "us";
  handler_metric.message.metric_name = "Handler processing time";
  handler_metric.message.metric = 0;

  full_processing_metric.message.units = "us";
  full_processing_metric.message.metric_name = "Full processing time";
  full_processing_metric.message.metric = 0;

  number_of_processed_points.message.units = "points";
  number_of_processed_points.message.metric_name = "Processed Points";

  for (int i = 0; i < 20; i++) {
    debug_points_message[i].units = "";
    debug_points_message[i].metric_name = "Debug point " + std::to_string(i);
    debug_points_message[i].metric = 0;
  }
}

void AlfaNode::metrics_publish() {
  if (configuration.metrics_publishing_type != NO_METRICS) {
    alfa_msg::msg::AlfaMetrics output_metrics;

    if (configuration.metrics_publishing_type & FULL_PROCESSING)
      output_metrics.metrics.push_back(full_processing_metric.message);

    if (configuration.metrics_publishing_type & HANDLER_PROCESSING) {
      output_metrics.metrics.push_back(handler_metric.message);
    }

    if (configuration.metrics_publishing_type & FRAMES_INFO) {
      output_metrics.metrics.push_back(number_of_processed_points.message);
    }

    if (configuration.metrics_publishing_type & DEBUG_POINTS) {
      for (unsigned int i = 0; i < configuration.number_of_debug_points; i++) {
        debug_points_message[i].metric = static_cast<float>(get_debug_point(i));

        output_metrics.metrics.push_back(debug_points_message[i]);
      }
    }
    publish_metrics(output_metrics);
  }
}

alfa_msg::msg::MetricMessage AlfaNode::get_metric_message(int metric) {
  switch (metric) {
    case HANDLER_TIME:
      return get_handler_time();
      break;

    case FULL_PROCESSING_TIME:
      return get_full_processing_time();
      break;

    default:
      return get_handler_time();
      break;
  }
  return get_handler_time();
}

alfa_msg::msg::MetricMessage AlfaNode::get_handler_time() {
#ifdef ALFA_VERBOSE
  verbose_info("get_handler_time", "getting handler time");
#endif
  return handler_metric.message;
}

alfa_msg::msg::MetricMessage AlfaNode::get_full_processing_time() {
#ifdef ALFA_VERBOSE
  verbose_info("get_full_processing_time", "getting full processing time");
#endif
  return full_processing_metric.message;
}

// Verbose functions
void AlfaNode::verbose_constr_chracteristics(string subscriber_topic,
                                             string node_name) {
  // Print the characteristics of the node
  std::cout << "--------------------------------------------------------"
            << std::endl;
  std::cout << "Starting ALFA node with the following settings:" << std::endl;
  std::cout << "Subscriber topic: " << subscriber_topic << std::endl;
  std::cout << "Name of the node: " << node_name << std::endl;
  std::cout << "Extension ID: " << configuration.extension_id << std::endl;
  std::cout << "Pointcloud ID: " << configuration.pointcloud_id << std::endl;
  std::cout << "Hardware Driver (SIU): "
            << configuration.hardware_support.hardware_driver << std::endl;
  std::cout << "Hardware Extension: "
            << configuration.hardware_support.hardware_extension << std::endl;
  std::cout << "--------------------------------------------------------"
            << std::endl;
}

void AlfaNode::verbose_begin(string function) {
  std::cout << "ALFA:" << function << "-> begin" << std::endl;
}

void AlfaNode::verbose_end(string function) {
  std::cout << "ALFA:" << function << "-> end" << std::endl;
}

void AlfaNode::verbose_ok(string function, string message) {
  std::cout << "ALFA:" << function << "-> " << message << " OK" << std::endl;
}

void AlfaNode::verbose_fail(string function, string message) {
  std::cout << "ALFA:" << function << "-> " << message << " FAIL" << std::endl;
}

void AlfaNode::verbose_info(string function, string message) {
  std::cout << "ALFA:" << function << "-> " << message << std::endl;
}

void AlfaNode::verbose_not_defined(string function) {
  std::cout << "ALFA:" << function << "-> HARDWARE NOT DEFINED!!" << std::endl;
}
