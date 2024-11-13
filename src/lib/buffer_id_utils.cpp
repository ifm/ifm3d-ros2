// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2024 ifm electronic, gmbh
 */

#include <map>
#include <string>
#include <vector>

#include <ifm3d/fg.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/buffer.h>

#include <ifm3d_ros2/buffer_id_utils.hpp>

namespace ifm3d_ros2
{
namespace buffer_id_utils
{
std::map<std::string, ifm3d::buffer_id> buffer_id_map = {
  { "RADIAL_DISTANCE_IMAGE", ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE },
  { "NORM_AMPLITUDE_IMAGE", ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE },
  { "AMPLITUDE_IMAGE", ifm3d::buffer_id::AMPLITUDE_IMAGE },
  { "GRAYSCALE_IMAGE", ifm3d::buffer_id::GRAYSCALE_IMAGE },
  { "RADIAL_DISTANCE_NOISE", ifm3d::buffer_id::RADIAL_DISTANCE_NOISE },
  { "REFLECTIVITY", ifm3d::buffer_id::REFLECTIVITY },
  { "CARTESIAN_X_COMPONENT", ifm3d::buffer_id::CARTESIAN_X_COMPONENT },
  { "CARTESIAN_Y_COMPONENT", ifm3d::buffer_id::CARTESIAN_Y_COMPONENT },
  { "CARTESIAN_Z_COMPONENT", ifm3d::buffer_id::CARTESIAN_Z_COMPONENT },
  { "CARTESIAN_ALL", ifm3d::buffer_id::CARTESIAN_ALL },
  { "UNIT_VECTOR_ALL", ifm3d::buffer_id::UNIT_VECTOR_ALL },
  { "MONOCHROM_2D_12BIT", ifm3d::buffer_id::MONOCHROM_2D_12BIT },
  { "MONOCHROM_2D", ifm3d::buffer_id::MONOCHROM_2D },
  { "JPEG_IMAGE", ifm3d::buffer_id::JPEG_IMAGE },
  { "CONFIDENCE_IMAGE", ifm3d::buffer_id::CONFIDENCE_IMAGE },
  { "DIAGNOSTIC", ifm3d::buffer_id::DIAGNOSTIC },
  { "JSON_DIAGNOSTIC", ifm3d::buffer_id::JSON_DIAGNOSTIC },
  { "EXTRINSIC_CALIB", ifm3d::buffer_id::EXTRINSIC_CALIB },
  { "INTRINSIC_CALIB", ifm3d::buffer_id::INTRINSIC_CALIB },
  { "INVERSE_INTRINSIC_CALIBRATION", ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION },
  { "TOF_INFO", ifm3d::buffer_id::TOF_INFO },
  { "RGB_INFO", ifm3d::buffer_id::RGB_INFO },
  { "JSON_MODEL", ifm3d::buffer_id::JSON_MODEL },
  { "ALGO_DEBUG", ifm3d::buffer_id::ALGO_DEBUG },
  { "O3R_ODS_OCCUPANCY_GRID", ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID },
  { "O3R_ODS_INFO", ifm3d::buffer_id::O3R_ODS_INFO },
  { "XYZ", ifm3d::buffer_id::XYZ },
  { "EXPOSURE_TIME", ifm3d::buffer_id::EXPOSURE_TIME },
  { "ILLUMINATION_TEMP", ifm3d::buffer_id::ILLUMINATION_TEMP }
};

std::multimap<ifm3d::buffer_id, ifm3d_ros2::buffer_id_utils::data_stream_type> data_stream_type_map = {
  { ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::AMPLITUDE_IMAGE, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::GRAYSCALE_IMAGE, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::RADIAL_DISTANCE_NOISE, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::REFLECTIVITY, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::CARTESIAN_X_COMPONENT, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::CARTESIAN_Y_COMPONENT, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::CARTESIAN_Z_COMPONENT, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::CARTESIAN_ALL, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::UNIT_VECTOR_ALL, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::MONOCHROM_2D_12BIT, ifm3d_ros2::buffer_id_utils::data_stream_type::rgb_2d },
  { ifm3d::buffer_id::MONOCHROM_2D, ifm3d_ros2::buffer_id_utils::data_stream_type::rgb_2d },
  { ifm3d::buffer_id::JPEG_IMAGE, ifm3d_ros2::buffer_id_utils::data_stream_type::rgb_2d },
  { ifm3d::buffer_id::CONFIDENCE_IMAGE, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::DIAGNOSTIC, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::DIAGNOSTIC, ifm3d_ros2::buffer_id_utils::data_stream_type::rgb_2d },
  { ifm3d::buffer_id::JSON_DIAGNOSTIC, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::JSON_DIAGNOSTIC, ifm3d_ros2::buffer_id_utils::data_stream_type::rgb_2d },
  { ifm3d::buffer_id::EXTRINSIC_CALIB, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::INTRINSIC_CALIB, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::TOF_INFO, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::RGB_INFO, ifm3d_ros2::buffer_id_utils::data_stream_type::rgb_2d },
  { ifm3d::buffer_id::JSON_MODEL, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::ALGO_DEBUG, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID, ifm3d_ros2::buffer_id_utils::data_stream_type::ods },
  { ifm3d::buffer_id::O3R_ODS_INFO, ifm3d_ros2::buffer_id_utils::data_stream_type::ods },
  { ifm3d::buffer_id::XYZ, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::EXPOSURE_TIME, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d },
  { ifm3d::buffer_id::ILLUMINATION_TEMP, ifm3d_ros2::buffer_id_utils::data_stream_type::tof_3d }
};

std::map<ifm3d::buffer_id, ifm3d_ros2::buffer_id_utils::message_type> message_type_map = {
  { ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE, ifm3d_ros2::buffer_id_utils::message_type::raw_image },
  { ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE, ifm3d_ros2::buffer_id_utils::message_type::raw_image },
  { ifm3d::buffer_id::AMPLITUDE_IMAGE, ifm3d_ros2::buffer_id_utils::message_type::raw_image },
  { ifm3d::buffer_id::GRAYSCALE_IMAGE, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::RADIAL_DISTANCE_NOISE, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::REFLECTIVITY, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::CARTESIAN_X_COMPONENT, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::CARTESIAN_Y_COMPONENT, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::CARTESIAN_Z_COMPONENT, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::CARTESIAN_ALL, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::UNIT_VECTOR_ALL, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::MONOCHROM_2D_12BIT, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::MONOCHROM_2D, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::JPEG_IMAGE, ifm3d_ros2::buffer_id_utils::message_type::compressed_image },
  { ifm3d::buffer_id::CONFIDENCE_IMAGE, ifm3d_ros2::buffer_id_utils::message_type::raw_image },
  { ifm3d::buffer_id::DIAGNOSTIC, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::JSON_DIAGNOSTIC, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::EXTRINSIC_CALIB, ifm3d_ros2::buffer_id_utils::message_type::extrinsics },
  { ifm3d::buffer_id::INTRINSIC_CALIB, ifm3d_ros2::buffer_id_utils::message_type::intrinsics },
  { ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION, ifm3d_ros2::buffer_id_utils::message_type::inverse_intrinsics },
  { ifm3d::buffer_id::TOF_INFO, ifm3d_ros2::buffer_id_utils::message_type::tof_info },
  { ifm3d::buffer_id::RGB_INFO, ifm3d_ros2::buffer_id_utils::message_type::rgb_info },
  { ifm3d::buffer_id::JSON_MODEL, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::ALGO_DEBUG, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID, ifm3d_ros2::buffer_id_utils::message_type::occupancy_grid },
  { ifm3d::buffer_id::O3R_ODS_INFO, ifm3d_ros2::buffer_id_utils::message_type::zones },
  { ifm3d::buffer_id::XYZ, ifm3d_ros2::buffer_id_utils::message_type::pointcloud },
  { ifm3d::buffer_id::EXPOSURE_TIME, ifm3d_ros2::buffer_id_utils::message_type::not_implemented },
  { ifm3d::buffer_id::ILLUMINATION_TEMP, ifm3d_ros2::buffer_id_utils::message_type::not_implemented }
};

std::map<ifm3d::buffer_id, std::string> topic_name_map = {
  { ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE, "distance" },
  { ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE, "amplitude" },
  { ifm3d::buffer_id::AMPLITUDE_IMAGE, "raw_amplitude" },
  { ifm3d::buffer_id::GRAYSCALE_IMAGE, "GRAYSCALE_IMAGE" },
  { ifm3d::buffer_id::RADIAL_DISTANCE_NOISE, "RADIAL_DISTANCE_NOISE" },
  { ifm3d::buffer_id::REFLECTIVITY, "REFLECTIVITY" },
  { ifm3d::buffer_id::CARTESIAN_X_COMPONENT, "CARTESIAN_X_COMPONENT" },
  { ifm3d::buffer_id::CARTESIAN_Y_COMPONENT, "CARTESIAN_Y_COMPONENT" },
  { ifm3d::buffer_id::CARTESIAN_Z_COMPONENT, "CARTESIAN_Z_COMPONENT" },
  { ifm3d::buffer_id::CARTESIAN_ALL, "CARTESIAN_ALL" },
  { ifm3d::buffer_id::UNIT_VECTOR_ALL, "UNIT_VECTOR_ALL" },
  { ifm3d::buffer_id::MONOCHROM_2D_12BIT, "MONOCHROM_2D_12BIT" },
  { ifm3d::buffer_id::MONOCHROM_2D, "MONOCHROM_2D" },
  { ifm3d::buffer_id::JPEG_IMAGE, "rgb" },
  { ifm3d::buffer_id::CONFIDENCE_IMAGE, "confidence" },
  { ifm3d::buffer_id::DIAGNOSTIC, "DIAGNOSTIC" },
  { ifm3d::buffer_id::JSON_DIAGNOSTIC, "JSON_DIAGNOSTIC" },
  { ifm3d::buffer_id::EXTRINSIC_CALIB, "extrinsics" },
  { ifm3d::buffer_id::INTRINSIC_CALIB, "intrinsic_calib" },
  { ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION, "inverse_intrinsic_calibration" },
  { ifm3d::buffer_id::TOF_INFO, "tof_info" },
  { ifm3d::buffer_id::RGB_INFO, "rgb_info" },
  { ifm3d::buffer_id::JSON_MODEL, "JSON_MODEL" },
  { ifm3d::buffer_id::ALGO_DEBUG, "ALGO_DEBUG" },
  { ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID, "ods_occupancy_grid" },
  { ifm3d::buffer_id::O3R_ODS_INFO, "ods_info" },
  { ifm3d::buffer_id::XYZ, "cloud" },
  { ifm3d::buffer_id::EXPOSURE_TIME, "EXPOSURE_TIME" },
  { ifm3d::buffer_id::ILLUMINATION_TEMP, "ILLUMINATION_TEMP" }
};

bool convert(const std::string& string, ifm3d::buffer_id& buffer_id)
{
  if (!buffer_id_map.count(string))
  {
    // key does not exist
    return false;
  }

  buffer_id = buffer_id_map[string];
  return true;
}

bool convert(const ifm3d::buffer_id& buffer_id, std::string& string)
{
  // Iterate over all map entries
  for (auto const& [key, value] : buffer_id_map)
  {
    if (value == buffer_id)
    {
      string = key;
      return true;
    }
  }

  // buffer_id not found
  return false;
}

std::vector<ifm3d::buffer_id> buffer_ids_for_data_stream_type(const std::vector<ifm3d::buffer_id>& input_ids,
                                                              const ifm3d_ros2::buffer_id_utils::data_stream_type& type)
{
  typedef std::multimap<ifm3d::buffer_id, ifm3d_ros2::buffer_id_utils::data_stream_type>::iterator mm_iterator;

  std::vector<ifm3d::buffer_id> ret_vector;

  for (ifm3d::buffer_id input_id : input_ids)
  {
    // Get iterators for multimap subsection of given buffer_id
    // Remember, multimaps are always sorted by their key
    std::pair<mm_iterator, mm_iterator> result =
        ifm3d_ros2::buffer_id_utils::data_stream_type_map.equal_range(input_id);

    // Look for matching data_streamtype, iterating over the subsection
    for (mm_iterator it = result.first; it != result.second; it++)
    {
      if (it->second == type)
      {
        ret_vector.push_back(input_id);
      }
    }
  }

  return ret_vector;
}

std::string vector_to_string(const std::vector<std::string>& vector)
{
  if (vector.empty())
  {
    return std::string("");
  }

  std::ostringstream stream;
  const std::string delimiter = ", ";

  std::copy(vector.begin(), vector.end(), std::ostream_iterator<std::string>(stream, delimiter.c_str()));

  const std::string& output = stream.str();

  return output.substr(0, output.length() - delimiter.length());
}

/**
 * Helper to create one string from a vector of buffer_ids
 */
std::string vector_to_string(const std::vector<ifm3d::buffer_id>& vector)
{
  if (vector.empty())
  {
    return std::string("");
  }

  std::ostringstream stream;
  const std::string delimiter = ", ";

  for (const auto& id : vector)
  {
    std::string string;
    convert(id, string);
    stream << string << delimiter;
  }

  const std::string& output = stream.str();

  return output.substr(0, output.length() - delimiter.length());
}

}  // namespace buffer_id_utils

}  // namespace ifm3d_ros2
