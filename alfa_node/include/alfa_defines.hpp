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

#ifndef ALFA_DEFINES_H
#define ALFA_DEFINES_H

#define ALIVE_TIMER_SLEEP 1000  // Time between alive messages
#define MAX_NUMBER_EXTENSIONS 4
#define MAX_NUMBER_POINTCLOUD 4
#define NUM_THREADS 4
#define OUTPUT_FRAME_BUFFER 2

// Custom Field types
#define CUSTOM_FIELD_USER 0
#define CUSTOM_FIELD_INTENSITY 1
#define CUSTOM_FIELD_LABEL 2
#define CUSTOM_FIELD_RGB 3
#define CUSTOM_FIELD_FILTER 4
#define CUSTOM_FIELD_INTENSITY_LABEL 5

// Filter labels
#define FILTER_VALUE 2

// Load/Store types
#define LOAD_STORE_CARTESIAN 0
#define LOAD_STORE_SPHERICAL 1

// Metrics configuration
#define NO_METRICS 0x0
#define FULL_PROCESSING 0x1
#define HANDLER_PROCESSING 0x2
#define FRAMES_INFO 0x4
#define DEBUG_POINTS 0x10
#define ALL_METRICS 0x1F

// Metrics defines
#define HANDLER_TIME 1
#define POST_PROCESSING_TIME 2
#define FULL_PROCESSING_TIME 3
#define FRAMES_LOST 4
#define WORST_HANDLER_TIME 5
#define WORST_POST_PROCESSING_TIME 6
#define WORST_FULL_PROCESSING_TIME 7

// UNIT defines
#define UNIT_STATUS_CU 0x00
#define UNIT_STATUS_EXMU 0x04
#define UNIT_STATUS_MEMMU 0x08
#define UNIT_STATUS_MONU 0x0C
#define UNIT_STATUS_SIU 0x10
#define UNIT_STATUS_EXT 0x14

#define UNIT_ND 0x18

#define UNIT_SIGNALS_HARDWARE_EXT_READY 0x1C
#define UNIT_SIGNALS_HARDWARE_POINTCLOUD_READY 0x20
#define UNIT_SIGNALS_HARDWARE_PROCESSING_DONE 0x24

#define UNIT_DEBUG_POINT_0 0x28
#define UNIT_DEBUG_POINT_1 0x2C
#define UNIT_DEBUG_POINT_2 0x30
#define UNIT_DEBUG_POINT_3 0x34
#define UNIT_DEBUG_POINT_4 0x38
#define UNIT_DEBUG_POINT_5 0x3C
#define UNIT_DEBUG_POINT_6 0x40
#define UNIT_DEBUG_POINT_7 0x44
#define UNIT_DEBUG_POINT_8 0x48
#define UNIT_DEBUG_POINT_9 0x4C
#define UNIT_DEBUG_POINT_10 0x50
#define UNIT_DEBUG_POINT_11 0x54
#define UNIT_DEBUG_POINT_12 0x58
#define UNIT_DEBUG_POINT_13 0x5C
#define UNIT_DEBUG_POINT_14 0x60
#define UNIT_DEBUG_POINT_15 0x64
#define UNIT_DEBUG_POINT_16 0x68
#define UNIT_DEBUG_POINT_17 0x6C
#define UNIT_DEBUG_POINT_18 0x70
#define UNIT_DEBUG_POINT_19 0x74

#define UNIT_PARAMETERS_CU 0x78
#define UNIT_PARAMETERS_MEMMU 0x7C
#define UNIT_PARAMETERS_EXMU 0x80
#define UNIT_PARAMETERS_MONU 0x84
#define UNIT_PARAMETERS_SIU 0x88

#define UNIT_SIGNALS_SOFTWARE_POINTCLOUD_READY 0x8C
#define UNIT_SIGNALS_SOFTWARE_PROCESSING_DONE 0x90
#define UNIT_SIGNALS_POINTCLOUD_SIZE 0x94
#define UNIT_SIGNALS_HARDWARE_FRAME_DONE 0x98

#define UNIT_USER_DEFINE_0 0x98
#define UNIT_USER_DEFINE_1 0x9C
#define UNIT_USER_DEFINE_2 0xA0
#define UNIT_USER_DEFINE_3 0xA4
#define UNIT_USER_DEFINE_4 0xA8
#define UNIT_USER_DEFINE_5 0xAC
#define UNIT_USER_DEFINE_6 0xB0
#define UNIT_USER_DEFINE_7 0xB4
#define UNIT_USER_DEFINE_8 0xB8
#define UNIT_USER_DEFINE_9 0xBC

// POINT CLOUD MEM
#define POINTCLOUD_BASE_ADDR 0x4000000
#define POINTCLOUD_BASE_ADDR_NON_CACHE 0x5000000
#define POINTCLOUD_BASE_PER_ID 0x200000
#define POINTCLOUD_MAX_POINT_SIZE POINTCLOUD_BASE_PER_ID / 8

#define HARDWARE_EXT_BASE_ADDR 0x80000000
#define HARDWARE_EXT_BASE_PER_ID 0x10000000

#define MAP_HUGE_2MB (21 << MAP_HUGE_SHIFT)

#define FIXED_POINT_MULTIPLIER 100.0f

// ALFA LABELS (Patch to fit in 8 bits)

/*------------------------------
     Paris-Lille 3D Labels
------------------------------*/

// GROUND-RELATED LABELS
#define PL_GROUND 200        // Paris-Lille 3D -> 202000000, but 200 on ALFA
#define PL_ROAD 202          // Paris-Lille 3D -> 202020000, but 202 on ALFA
#define PL_SIDEWALK 203      // Paris-Lille 3D -> 202030000, but 203 on ALFA
#define PL_CURB 204          // Paris-Lille 3D -> 202040000, but 204 on ALFA
#define PL_ISLAND 205        // Paris-Lille 3D -> 202050000, but 205 on ALFA
#define PL_VEGETATION 206    // Paris-Lille 3D -> 202060000, but 206 on ALFA
#define PL_OTHER_GROUND 201  // Paris-Lille 3D -> 202010000, but 201 on ALFA

/*------------------------------
     SemanticKITTI Labels
------------------------------*/

// UNLABELED
#define UNLABELED 0

// OUTLIER
#define OUTLIER 1

/* GROUND-RELATED */
#define ROAD 40
#define PARKING 44
#define SIDEWALK 48
#define OTHER_GROUND 49
#define LANE_MARKING 60

/* STRUCTURES */
#define BUILDING 50
#define OTHER_STRUCTURE 52

/* VEHICLE */
#define CAR 10
#define BICYCLE 11
#define BUS 13
#define MOTORCYCLE 15
#define ON_RAILS 16
#define TRUCK 18
#define OTHER_VEHICLE 20
#define MOVING_CAR 152            // SemanticKITTI -> 252, but 152 on ALFA
#define MOVING_ON_RAILS 156       // SemanticKITTI -> 256, but 156 on ALFA
#define MOVING_BUS 157            // SemanticKITTI -> 257, but 157 on ALFA
#define MOVING_TRUCK 158          // SemanticKITTI -> 258, but 158 on ALFA
#define MOVING_OTHER_VEHICLE 159  // SemanticKITTI -> 259, but 159 on ALFA

/* NATURE */
#define VEGETATION 70
#define TRUNK 71
#define TERRAIN 72

/* HUMAN */
#define PERSON 30
#define BICYCLIST 31
#define MOTORCYCLIST 32
#define MOVING_BICYCLIST 153     // SemanticKITTI -> 253, but 153 on ALFA
#define MOVING_PERSON 154        // SemanticKITTI -> 254, but 154 on ALFA
#define MOVING_MOTORCYCLIST 155  // SemanticKITTI -> 255, but 155 on ALFA

/* OBJECT */
#define FENCE 51
#define POLE 80
#define TRAFFIC_SIGN 81
#define OTHER_OBJECT 99

#endif  // ALFA_DEFINES_H