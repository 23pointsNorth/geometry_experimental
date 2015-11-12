/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf2/buffer_core.h>

#include <console_bridge/console.h>

#include <boost/lexical_cast.hpp>

int main(int argc, char** argv)
{
  uint32_t num_levels = 10;
  if (argc > 1)
  {
    num_levels = boost::lexical_cast<uint32_t>(argv[1]);
  }

  tf2::BufferCore bc;
  tf2::Transform t;
  t.setIdentity();
  t.setOrigin(tf2::Vector3(0,0,1));
  bc.setTransform(t, tf2::TimePoint(std::chrono::seconds(1)), "root", "0", "me");
  bc.setTransform(t, tf2::TimePoint(std::chrono::seconds(2)), "root", "0", "me");

  for (uint32_t i = 1; i < num_levels/2; ++i)
  {
    for (uint32_t j = 1; j < 3; ++j)
    {
      std::stringstream parent_ss;
      parent_ss << (i - 1);
      std::stringstream child_ss;
      child_ss << i;

      bc.setTransform(t, tf2::TimePoint(std::chrono::seconds(j)), parent_ss.str(), child_ss.str(), "me");
    }
  }

  std::stringstream ss;
  ss << num_levels/2;
  bc.setTransform(t, tf2::TimePoint(std::chrono::seconds(1)), "root", ss.str(), "me");
  bc.setTransform(t, tf2::TimePoint(std::chrono::seconds(2)), "root", ss.str(), "me");

  for (uint32_t i = num_levels/2 + 1; i < num_levels; ++i)
  {
    for (uint32_t j = 1; j < 3; ++j)
    {
      std::stringstream parent_ss;
      parent_ss << (i - 1);
      std::stringstream child_ss;
      child_ss << i;

      bc.setTransform(t, tf2::TimePoint(std::chrono::seconds(j)), parent_ss.str(), child_ss.str(), "me");
    }
  }

  //logInfo_STREAM(bc.allFramesAsYAML());

  std::string v_frame0 = boost::lexical_cast<std::string>(num_levels - 1);
  std::string v_frame1 = boost::lexical_cast<std::string>(num_levels/2 - 1);
  logInform("%s to %s", v_frame0.c_str(), v_frame1.c_str());
  tf2::Transform out_t;

  const uint32_t count = 1000000;
  logInform("Doing %d %d-level tests", count, num_levels);

  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::chrono::duration<double> dur;
#if 01
  {
    start = std::chrono::system_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      out_t = bc.lookupTransform<tf2::Transform>(v_frame1, v_frame0, tf2::TimePoint()).get();
    }
    end = std::chrono::system_clock::now();
    dur = end-start;
    //ROS_INFO_STREAM(out_t);
    logInform("lookupTransform at Time(0) took %f for an average of %.9f", dur.count(), dur.count() / (double)count);
  }
#endif

#if 01
  {
    start = std::chrono::system_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      out_t = bc.lookupTransform<tf2::Transform>(v_frame1, v_frame0, tf2::TimePoint(std::chrono::seconds(1))).get();
    }
    end = std::chrono::system_clock::now();
    dur = end - start;
    //ROS_INFO_STREAM(out_t);
    logInform("lookupTransform at Time(1) took %f for an average of %.9f", dur.count(), dur.count() / (double)count);
  }
#endif

#if 01
  {
    start = std::chrono::system_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      out_t = bc.lookupTransform<tf2::Transform>(v_frame1, v_frame0, tf2::TimePoint(std::chrono::milliseconds(1500))).get();
    }
    end = std::chrono::system_clock::now();
    dur = end - start;
    //ROS_INFO_STREAM(out_t);
    logInform("lookupTransform at Time(1.5) took %f for an average of %.9f", dur.count(), dur.count() / (double)count);
  }
#endif

#if 01
  {
    start = std::chrono::system_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      out_t = bc.lookupTransform<tf2::Transform>(v_frame1, v_frame0, tf2::TimePoint(std::chrono::seconds(2))).get();
    }
    end = std::chrono::system_clock::now();
    dur = end - start;
    //ROS_INFO_STREAM(out_t);
    logInform("lookupTransform at Time(2) took %f for an average of %.9f", dur.count(), dur.count() / (double)count);
  }
#endif

#if 01
  {
    start = std::chrono::system_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      bc.canTransform(v_frame1, v_frame0, tf2::TimePoint());
    }
    end = std::chrono::system_clock::now();
    dur = end - start;
    //ROS_INFO_STREAM(out_t);
    logInform("canTransform at Time(0) took %f for an average of %.9f", dur.count(), dur.count() / (double)count);
  }
#endif

#if 01
  {
    start = std::chrono::system_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      bc.canTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::seconds(1)));
    }
    end = std::chrono::system_clock::now();
    dur = end - start;
    //ROS_INFO_STREAM(out_t);
    logInform("canTransform at Time(1) took %f for an average of %.9f", dur.count(), dur.count() / (double)count);
  }
#endif

#if 01
  {
    start = std::chrono::system_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      bc.canTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::milliseconds(1500)));
    }
    end = std::chrono::system_clock::now();
    dur = end - start;
    //ROS_INFO_STREAM(out_t);
    logInform("canTransform at Time(1.5) took %f for an average of %.9f", dur.count(), dur.count() / (double)count);
  }
#endif

#if 01
  {
    start = std::chrono::system_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      bc.canTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::seconds(2)));
    }
    end = std::chrono::system_clock::now();
    dur = end - start;
    //ROS_INFO_STREAM(out_t);
    logInform("canTransform at Time(2) took %f for an average of %.9f", dur.count(), dur.count() / (double)count);
  }
#endif
}
