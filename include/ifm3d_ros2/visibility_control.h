// -*- c++ -*-
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IFM3D_ROS2__VISIBILITY_CONTROL_H_
#define IFM3D_ROS2__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define IFM3D_ROS2_EXPORT __attribute__((dllexport))
#define IFM3D_ROS2_IMPORT __attribute__((dllimport))
#else
#define IFM3D_ROS2_EXPORT __declspec(dllexport)
#define IFM3D_ROS2_IMPORT __declspec(dllimport)
#endif
#ifdef IFM3D_ROS2_BUILDING_DLL
#define IFM3D_ROS2_PUBLIC IFM3D_ROS2_EXPORT
#else
#define IFM3D_ROS2_PUBLIC IFM3D_ROS2_IMPORT
#endif
#define IFM3D_ROS2_PUBLIC_TYPE IFM3D_ROS2_PUBLIC
#define IFM3D_ROS2_LOCAL
#else
#define IFM3D_ROS2_EXPORT __attribute__((visibility("default")))
#define IFM3D_ROS2_IMPORT
#if __GNUC__ >= 4
#define IFM3D_ROS2_PUBLIC __attribute__((visibility("default")))
#define IFM3D_ROS2_LOCAL __attribute__((visibility("hidden")))
#else
#define IFM3D_ROS2_PUBLIC
#define IFM3D_ROS2_LOCAL
#endif
#define IFM3D_ROS2_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // IFM3D_ROS2__VISIBILITY_CONTROL_H_
