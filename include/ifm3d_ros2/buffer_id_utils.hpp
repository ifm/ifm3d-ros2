// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS2_CONSTANTS_HPP_
#define IFM3D_ROS2_CONSTANTS_HPP_

#include <map>
#include <string>
#include <vector>

#include <ifm3d/fg.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/buffer.h>

namespace ifm3d_ros2
{
namespace buffer_id_utils
{
/**
 * To differentiate between different data connections
 */
enum data_stream_type
{
  rgb_2d,
  tof_3d,
  ods,
};

/**
 * To differentiate between different ROS message types
 */
enum message_type
{
  compressed_image,
  extrinsics,
  intrinsics,
  inverse_intrinsics,
  occupancy_grid,
  pointcloud,
  raw_image,
  rgb_info,
  tof_info,
  zones,
  not_implemented,
};

/**
 * @brief map to lookup buffer_id from human-readable strings
 *
 * It is not declared const because the operator[] of std::map would not be available.
 * String are taken from the ifm3d SDK (modules/pybind11/src/bindings/frame.h)
 */
extern std::map<std::string, ifm3d::buffer_id> buffer_id_map;

/**
 * @brief mapping buffer_ids to data_stream_type where they are available
 *
 * Using a multimap to allow for multiple entries per buffer_id,
 * as some buffer might be available for different data_stream_type
 * It is not declared const because equal_range() of std::multimap would not be available.
 */
extern std::multimap<ifm3d::buffer_id, data_stream_type> data_stream_type_map;

/**
 * @brief mapping buffer_ids to message type enum
 *
 * To differentiate between different data formats of buffer,
 * e.g. to decide on which ROS message type shall be used.
 * It is not declared const because the operator[] of std::map would not be available.
 */
extern std::map<ifm3d::buffer_id, message_type> message_type_map;

/**
 * @brief mapping buffer_ids to topic names
 *
 * To allow for easily readable topic names, usage of common ROS terms
 * and backwards compatibility.
 * It is not declared const because the operator[] of std::map would not be available.
 */
extern std::map<ifm3d::buffer_id, std::string> topic_name_map;

/**
 * @brief Lookup buffer_id associated with a string, using the buffer_id_map
 *
 * @param string lookup term
 * @param buffer_id the found buffer_id
 * @return true if string is valid
 * @return false if no buffer_id is associated with the string
 */
bool convert(const std::string& string, ifm3d::buffer_id& buffer_id);

/**
 * @brief Lookup string associated with a buffer_id, using the buffer_id_map
 *
 * @param buffer_id lookup term
 * @param string the found strinf
 * @return true if buffer_id is valid
 * @return false there is no entry in the map for the given buffer_id
 */
bool convert(const ifm3d::buffer_id& buffer_id, std::string& string);

/**
 * @brief Returns the subset of the provided buffer_ids, which are compatible with a given data_stream_type
 *
 * @param input_ids set of buffer_id to be filtered
 * @param type data_stream_type as filter criterion
 * @return std::vector<ifm3d::buffer_id> subset of input_ids which is available for given type
 */
std::vector<ifm3d::buffer_id> buffer_ids_for_data_stream_type(const std::vector<ifm3d::buffer_id>& input_ids,
                                                              const data_stream_type& type);

/**
 * Helper to create one string from a vector of strings
 */
std::string vector_to_string(const std::vector<std::string>& vector);

/**
 * Helper to create one string from a vector of buffer_ids
 */
std::string vector_to_string(const std::vector<ifm3d::buffer_id>& vector);

}  // namespace buffer_id_utils

}  // namespace ifm3d_ros2

#endif