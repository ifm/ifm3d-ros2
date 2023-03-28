#ifndef IFM3D_ROS2_CONSTANTS_HPP_
#define IFM3D_ROS2_CONSTANTS_HPP_

#include <map>
#include <string>

#include "ifm3d/fg/frame.h"

namespace ifm3d_ros2
{
namespace buffer_id_utils
{
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
  { "O3R_DISTANCE_IMAGE_INFO", ifm3d::buffer_id::O3R_DISTANCE_IMAGE_INFO },
  { "O3R_RGB_IMAGE_INFO", ifm3d::buffer_id::O3R_RGB_IMAGE_INFO },
  { "JSON_MODEL", ifm3d::buffer_id::JSON_MODEL },
  { "ALGO_DEBUG", ifm3d::buffer_id::ALGO_DEBUG },
  { "O3R_ODS_OCCUPANCY_GRID", ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID },
  { "O3R_ODS_INFO", ifm3d::buffer_id::O3R_ODS_INFO },
  { "XYZ", ifm3d::buffer_id::XYZ },
  { "EXPOSURE_TIME", ifm3d::buffer_id::EXPOSURE_TIME },
  { "ILLUMINATION_TEMP", ifm3d::buffer_id::ILLUMINATION_TEMP }
};

/**
 * @brief map to lookup topic names from buffer_ids
 *
 * It is not declared const because the operator[] of std::map would not be available.
 */
std::map<ifm3d::buffer_id, std::string> topic_name_map = {
  { ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE, "RADIAL_DISTANCE_IMAGE" },
  { ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE, "NORM_AMPLITUDE_IMAGE" },
  { ifm3d::buffer_id::AMPLITUDE_IMAGE, "AMPLITUDE_IMAGE" },
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
  { ifm3d::buffer_id::JPEG_IMAGE, "JPEG_IMAGE" },
  { ifm3d::buffer_id::CONFIDENCE_IMAGE, "CONFIDENCE_IMAGE" },
  { ifm3d::buffer_id::DIAGNOSTIC, "DIAGNOSTIC" },
  { ifm3d::buffer_id::JSON_DIAGNOSTIC, "JSON_DIAGNOSTIC" },
  { ifm3d::buffer_id::EXTRINSIC_CALIB, "EXTRINSIC_CALIB" },
  { ifm3d::buffer_id::INTRINSIC_CALIB, "INTRINSIC_CALIB" },
  { ifm3d::buffer_id::INVERSE_INTRINSIC_CALIBRATION, "INVERSE_INTRINSIC_CALIBRATION" },
  { ifm3d::buffer_id::O3R_DISTANCE_IMAGE_INFO, "O3R_DISTANCE_IMAGE_INFO" },
  { ifm3d::buffer_id::O3R_RGB_IMAGE_INFO, "O3R_RGB_IMAGE_INFO" },
  { ifm3d::buffer_id::JSON_MODEL, "JSON_MODEL" },
  { ifm3d::buffer_id::ALGO_DEBUG, "ALGO_DEBUG" },
  { ifm3d::buffer_id::O3R_ODS_OCCUPANCY_GRID, "O3R_ODS_OCCUPANCY_GRID" },
  { ifm3d::buffer_id::O3R_ODS_INFO, "O3R_ODS_INFO" },
  { ifm3d::buffer_id::XYZ, "XYZ" },
  { ifm3d::buffer_id::EXPOSURE_TIME, "EXPOSURE_TIME" },
  { ifm3d::buffer_id::ILLUMINATION_TEMP, "ILLUMINATION_TEMP" }
};

const std::vector<ifm3d::buffer_id> image_ids = { ifm3d::buffer_id::CONFIDENCE_IMAGE,       //
                                                  ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,  //
                                                  ifm3d::buffer_id::RADIAL_DISTANCE_NOISE,  //
                                                  ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,   //
                                                  ifm3d::buffer_id::AMPLITUDE_IMAGE };
const std::vector<ifm3d::buffer_id> compressed_image_ids = { ifm3d::buffer_id::JPEG_IMAGE };
const std::vector<ifm3d::buffer_id> plc_ids = {};
const std::vector<ifm3d::buffer_id> extrinsics_ids = { ifm3d::buffer_id::EXTRINSIC_CALIB };

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
};

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
 * Helper to create one string from a vector of strings
 */
std::string vector_to_string(const std::vector<std::string>& vector)
{
  std::ostringstream stream;
  const char* const delimiter = ", ";

  std::copy(vector.begin(), vector.end(), std::ostream_iterator<std::string>(stream, delimiter));

  return stream.str();
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

  return output.substr(0, output.length() - 1 - delimiter.length());
}

}  // namespace buffer_id_utils

}  // namespace ifm3d_ros2

#endif