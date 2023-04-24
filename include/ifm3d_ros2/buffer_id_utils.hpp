#ifndef IFM3D_ROS2_CONSTANTS_HPP_
#define IFM3D_ROS2_CONSTANTS_HPP_

#include <map>
#include <string>
#include <vector>

#include "ifm3d/fg/frame.h"

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
};

/**
 * To differentiate between different ROS message types
 */
enum message_type
{
  intrinsics,
  compressed_image,
  extrinsics,
  inverse_intrinsics,
  pointcloud,
  raw_image,
  rgb_info,
  tof_info,
  not_implemented,
};

/**
 * @brief map to lookup buffer_id from human-readable strings
 *
 * It is not declared const because the operator[] of std::map would not be available.
 * String are taken from the ifm3d SDK (modules/pybind11/src/bindings/frame.h)
 */
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

/**
 * @brief mapping buffer_ids to data_stream_type where they are available
 *
 * Using a multimap to allow for multiple entries per buffer_id,
 * as some buffer might be available for different data_stream_type
 * It is not declared const because equal_range() of std::multimap would not be available.
 */
std::multimap<ifm3d::buffer_id, data_stream_type> data_stream_type_map = {
  { ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE, data_stream_type::tof_3d },
  { ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE, data_stream_type::tof_3d },
  { ifm3d::buffer_id::AMPLITUDE_IMAGE, data_stream_type::rgb_2d },
  { ifm3d::buffer_id::GRAYSCALE_IMAGE, data_stream_type::rgb_2d },
  { ifm3d::buffer_id::RADIAL_DISTANCE_NOISE, data_stream_type::tof_3d },
  { ifm3d::buffer_id::REFLECTIVITY, data_stream_type::tof_3d },
  { ifm3d::buffer_id::CARTESIAN_X_COMPONENT, data_stream_type::tof_3d },
  { ifm3d::buffer_id::CARTESIAN_Y_COMPONENT, data_stream_type::tof_3d },
  { ifm3d::buffer_id::CARTESIAN_Z_COMPONENT, data_stream_type::tof_3d },
  { ifm3d::buffer_id::CARTESIAN_ALL, data_stream_type::tof_3d },
  { ifm3d::buffer_id::UNIT_VECTOR_ALL, data_stream_type::tof_3d },
  { ifm3d::buffer_id::MONOCHROM_2D_12BIT, data_stream_type::rgb_2d },
  { ifm3d::buffer_id::MONOCHROM_2D, data_stream_type::rgb_2d },
  { ifm3d::buffer_id::JPEG_IMAGE, data_stream_type::rgb_2d },
  { ifm3d::buffer_id::CONFIDENCE_IMAGE, data_stream_type::tof_3d },
  { ifm3d::buffer_id::DIAGNOSTIC, data_stream_type::tof_3d },
  { ifm3d::buffer_id::JSON_DIAGNOSTIC, data_stream_type::tof_3d },
  { ifm3d::buffer_id::EXTRINSIC_CALIB, data_stream_type::tof_3d },
  { ifm3d::buffer_id::EXTRINSIC_CALIB, data_stream_type::rgb_2d },  // TODO assuming ExCal for both
  { ifm3d::buffer_id::INTRINSIC_CALIB, data_stream_type::tof_3d },
  { ifm3d::buffer_id::INTRINSIC_CALIB, data_stream_type::rgb_2d },
  { ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION, data_stream_type::tof_3d },
  { ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION, data_stream_type::rgb_2d },
  { ifm3d::buffer_id::TOF_INFO, data_stream_type::tof_3d },
  { ifm3d::buffer_id::RGB_INFO, data_stream_type::rgb_2d },
  { ifm3d::buffer_id::JSON_MODEL, data_stream_type::tof_3d },
  { ifm3d::buffer_id::ALGO_DEBUG, data_stream_type::tof_3d },
  { ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID, data_stream_type::tof_3d },
  { ifm3d::buffer_id::O3R_ODS_INFO, data_stream_type::tof_3d },
  { ifm3d::buffer_id::XYZ, data_stream_type::tof_3d },
  { ifm3d::buffer_id::EXPOSURE_TIME, data_stream_type::tof_3d },
  { ifm3d::buffer_id::ILLUMINATION_TEMP, data_stream_type::tof_3d }
};

/**
 * @brief mapping buffer_ids to message type enum
 *
 * To differentiate between different data formats of buffer,
 * e.g. to decide on which ROS message type shall be used.
 * It is not declared const because the operator[] of std::map would not be available.
 */
std::map<ifm3d::buffer_id, message_type> message_type_map = {
  { ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE, message_type::raw_image },
  { ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE, message_type::raw_image },
  { ifm3d::buffer_id::AMPLITUDE_IMAGE, message_type::raw_image },
  { ifm3d::buffer_id::GRAYSCALE_IMAGE, message_type::not_implemented },
  { ifm3d::buffer_id::RADIAL_DISTANCE_NOISE, message_type::not_implemented },
  { ifm3d::buffer_id::REFLECTIVITY, message_type::not_implemented },
  { ifm3d::buffer_id::CARTESIAN_X_COMPONENT, message_type::not_implemented },
  { ifm3d::buffer_id::CARTESIAN_Y_COMPONENT, message_type::not_implemented },
  { ifm3d::buffer_id::CARTESIAN_Z_COMPONENT, message_type::not_implemented },
  { ifm3d::buffer_id::CARTESIAN_ALL, message_type::not_implemented },
  { ifm3d::buffer_id::UNIT_VECTOR_ALL, message_type::not_implemented },
  { ifm3d::buffer_id::MONOCHROM_2D_12BIT, message_type::not_implemented },
  { ifm3d::buffer_id::MONOCHROM_2D, message_type::not_implemented },
  { ifm3d::buffer_id::JPEG_IMAGE, message_type::compressed_image },
  { ifm3d::buffer_id::CONFIDENCE_IMAGE, message_type::raw_image },
  { ifm3d::buffer_id::DIAGNOSTIC, message_type::not_implemented },
  { ifm3d::buffer_id::JSON_DIAGNOSTIC, message_type::not_implemented },
  { ifm3d::buffer_id::EXTRINSIC_CALIB, message_type::extrinsics },
  { ifm3d::buffer_id::INTRINSIC_CALIB, message_type::intrinsics },
  { ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION, message_type::inverse_intrinsics },
  { ifm3d::buffer_id::TOF_INFO, message_type::tof_info },
  { ifm3d::buffer_id::RGB_INFO, message_type::rgb_info },
  { ifm3d::buffer_id::JSON_MODEL, message_type::not_implemented },
  { ifm3d::buffer_id::ALGO_DEBUG, message_type::not_implemented },
  { ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID, message_type::not_implemented },
  { ifm3d::buffer_id::O3R_ODS_INFO, message_type::not_implemented },
  { ifm3d::buffer_id::XYZ, message_type::pointcloud },
  { ifm3d::buffer_id::EXPOSURE_TIME, message_type::not_implemented },
  { ifm3d::buffer_id::ILLUMINATION_TEMP, message_type::not_implemented }
};

/**
 * @brief mapping buffer_ids to topic names
 *
 * To allow for easily readable topic names, usage of common ROS terms
 * and backwards compatibility.
 * It is not declared const because the operator[] of std::map would not be available.
 */
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
  { ifm3d::buffer_id::INTRINSIC_CALIB, "INTRINSIC_CALIB" },
  { ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION, "INVERSE_INTRINSIC_CALIBRATION" },
  { ifm3d::buffer_id::TOF_INFO, "TOF_INFO" },
  { ifm3d::buffer_id::RGB_INFO, "RGB_INFO" },
  { ifm3d::buffer_id::JSON_MODEL, "JSON_MODEL" },
  { ifm3d::buffer_id::ALGO_DEBUG, "ALGO_DEBUG" },
  { ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID, "O3R_ODS_OCCUPANCY_GRID" },
  { ifm3d::buffer_id::O3R_ODS_INFO, "O3R_ODS_INFO" },
  { ifm3d::buffer_id::XYZ, "cloud" },
  { ifm3d::buffer_id::EXPOSURE_TIME, "EXPOSURE_TIME" },
  { ifm3d::buffer_id::ILLUMINATION_TEMP, "ILLUMINATION_TEMP" }
};

/**
 * @brief Lookup buffer_id associated with a string, using the buffer_id_map
 *
 * @param string lookup term
 * @param buffer_id the found buffer_id
 * @return true if string is valid
 * @return false if no buffer_id is associated with the string
 */
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

/**
 * @brief Lookup string associated with a buffer_id, using the buffer_id_map
 *
 * @param buffer_id lookup term
 * @param string the found strinf
 * @return true if buffer_id is valid
 * @return false there is no entry in the map for the given buffer_id
 */
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

/**
 * @brief Returns the subset of the provided buffer_ids, which are compatible with a given data_stream_type
 *
 * @param input_ids set of buffer_id to be filtered
 * @param type data_stream_type as filter criterion
 * @return std::vector<ifm3d::buffer_id> subset of input_ids which is available for given type
 */
std::vector<ifm3d::buffer_id> buffer_ids_for_data_stream_type(const std::vector<ifm3d::buffer_id>& input_ids,
                                                              const data_stream_type& type)
{
  typedef std::multimap<ifm3d::buffer_id, data_stream_type>::iterator mm_iterator;

  std::vector<ifm3d::buffer_id> ret_vector;

  for (ifm3d::buffer_id input_id : input_ids)
  {
    // Get iterators for multimap subsection of given buffer_id
    // Remember, multimaps are always sorted by their key
    std::pair<mm_iterator, mm_iterator> result = data_stream_type_map.equal_range(input_id);

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

/**
 * Helper to create one string from a vector of strings
 */
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

#endif