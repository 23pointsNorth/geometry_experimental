// Copyright 2014 Open Source Robotics Foundation, Inc.
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

/** \author Tully Foote */

#ifndef TF2_TIME_H
#define TF2_TIME_H

#include <ros/duration.h>
#include <ros/time.h>

#include <chrono>
#include <iomanip>

namespace tf2 {

typedef std::chrono::system_clock::time_point TimePoint;
typedef std::chrono::system_clock::duration Duration;

// Display functions as there is no default display
// TODO: find a proper way to handle display
inline std::string displayTimePoint(const TimePoint& stamp)
{
  // Below would only work with GCC 5.0 and above
  //return std::put_time(&stamp, "%c");
  std::time_t time = std::chrono::system_clock::to_time_t(stamp);
  return std::ctime(&time);
}
inline double displayDuration(const Duration& duration)
{
  return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
}

inline tf2::TimePoint chrono_from_rostime(const ros::Time& time)
{
  return tf2::TimePoint(std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nsec));
}

inline tf2::Duration chrono_from_rostime(const ros::Duration& time)
{
  return std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nsec);
}

inline tf2::TimePoint chrono_from_double(double time)
{
  ros::Time tmp;
  tmp.fromSec(time);
  return chrono_from_rostime(tmp);
}

inline ros::Time rostime_from_chrono(const tf2::TimePoint& time)
{
  ros::Time res;
  res.sec = std::chrono::time_point_cast<std::chrono::seconds>(time).time_since_epoch().count();
  res.nsec = std::chrono::time_point_cast<std::chrono::nanoseconds>(time).time_since_epoch().count() - res.sec*1e9;

  return res;
}

inline ros::Duration rostime_from_chrono(const tf2::Duration& time)
{
  ros::Duration res;
  res.sec = std::chrono::duration_cast<std::chrono::seconds>(time).count();
  res.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(time).count() - res.sec*1e9;

  return res;
}

}

#endif //TF2_TIME_H
