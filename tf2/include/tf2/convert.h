/*
 * Copyright (c) 2013, Open Source Robotics Foundation
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

/** \author Tully Foote */

#ifndef TF2_CONVERT_H
#define TF2_CONVERT_H

#include "time.h"
#include "LinearMath/Transform.h"

namespace tf2 {

class BufferCore;

/**
 * This proxy class can help storing a geometric transform, the frames it transforms from and to, and its timestamp
 * You need to specialize this class for any dataype you want tf2 to work with: get/set Transform is
 * the only function you have to implement.
 * If the TransformClass you are based on can store the frame, or the child_frame_id or the timestamp (as does a
 * ROS geometry_msgs/TransformStamped), you also need to overwrite the corresponding get/set methods to use
 * your type.
 */
template <typename T>
class TransformProxy {
public:
  inline TransformProxy() {};
  inline TransformProxy(const T& t) : t_(t) {};
  inline TransformProxy(const T& t, const TimePoint &time, const std::string &frame_id,
                        const std::string &child_frame_id) : TransformProxy(t)
  {
    time_ = time;
    frame_id_ = frame_id;
    child_frame_id_ = child_frame_id;
  };

  // Does not return a reference as it might be the internally created reference
  inline T get() const {return t_;};
  inline void set(const T &t) {t_ = t;};

  // Need to be implemented by the specialization package
  void getTransform(double &qx, double &qy, double &qz, double &qw, double &x, double &y, double &z) const;
  void setTransform(double qx, double qy, double qz, double qw, double x, double y, double z);

  // Can be overriden by the specialization package
  inline TimePoint getTime() const {return time_;};
  inline void setTime(const TimePoint &time) {time_ = time;};

  // Can be overriden by the specialization package
  inline std::string getFrameId() const {return frame_id_;};
  inline void setFrameId(const std::string &frame_id) {frame_id_ = frame_id;};

  // Can be overriden by the specialization package
  inline std::string getChildFrameId() const {return child_frame_id_;};
  inline void setChildFrameId(const std::string &child_frame_id) {child_frame_id_ = child_frame_id;};

  friend tf2::BufferCore;
private:
  /** a transform of the type of interest*/
  T t_;

  /** The time of the transform: if t_ can include that, use it and do not use time_.
   * In that case, you will need to overwrite get/set Time
   */
  TimePoint time_;
  /** The frame the transform transforms to: if t_ can include that, use it and do not use frame_id.
   * In that case, you will need to overwrite get/set FrameId
   */
  std::string frame_id_;
  /** The frame the transform transforms from: if t_ can include that, use it and do not use child_frame_id
   * In that case, you will need to overwrite get/set ChildFrameId
   */
  std::string child_frame_id_;

  /**
   * This function is only for convenience for BufferCore. It returns the transform as a tf2::Transform
   * \return the geometric part of the transform as a tf2::Transform
   */
  inline tf2::Transform getTf2Transform() const
  {
    double qx, qy, qz, qw, x, y, z;
    getTransform(qx, qy, qz, qw, x, y, z);
    return tf2::Transform(tf2::Quaternion(qx, qy, qz, qw), tf2::Vector3(x, y, z));
  }
  /**
   * This function is only for convenience for BufferCore. It sets the geometric transform when given as tf2::Transform
   * \param t the geometric part of the transform to set as a tf2::Transform
   */
  inline void setTf2Transform(const tf2::Transform &t)
  {
    setTransform(t.getRotation().x(), t.getRotation().y(), t.getRotation().z(), t.getRotation().w(),
                 t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
  }
};

}

#endif //TF2_CONVERT_H
