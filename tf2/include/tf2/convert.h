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

#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/impl/convert.h>

namespace tf2 {

/**\brief The templated function expected to be able to do a transform
 *
 * This is the method which tf2 will use to try to apply a transform for any given datatype.   
 * \param data_in The data to be transformed.
 * \param data_out A reference to the output data.  Note this can point to data in and the method should be mutation safe.
 * \param transform The transform to apply to data_in to fill data_out.  
 * 
 * This method needs to be implemented by client library developers
 */
template <class T>
  void doTransform(const T& data_in, T& data_out, const geometry_msgs::TransformStamped& transform);

/**\brief Get the timestamp from data 
 * \param t The data input.
 * \return The timestamp associated with the data. 
 */
template <class T>
  TimePoint getTimestamp(const T& t);

/**\brief Get the frame_id from data 
 * \param t The data input.
 * \return The frame_id associated with the data. 
 */
template <class T>
  const std::string& getFrameId(const T& t);



/* An implementation for Stamped<P> datatypes */
template <class P>
  const TimePoint& getTimestamp(const tf2::Stamped<P>& t)
  {
    return t.stamp_;
  }

/* An implementation for Stamped<P> datatypes */
template <class P>
  const std::string& getFrameId(const tf2::Stamped<P>& t)
  {
    return t.frame_id_;
  }

/** Function that converts from one type to a ROS message type. It has to be
 * implemented by each data type in tf2_* (except ROS messages) as it is
 * used in the "convert" function.
 * \param a an object of whatever type
 * \return the conversion as a ROS message
 */
template<typename A, typename B>
  B toMsg(const A& a);

/** Function that converts from a ROS message type to another type. It has to be
 * implemented by each data type in tf2_* (except ROS messages) as it is used
 * in the "convert" function.
 * \param a a ROS message to convert from
 * \param b the object to convert to
 */
template<typename A, typename B>
  void fromMsg(const A&, B& b);

/** Function that converts any type to any type (messages or not).
 * Matching toMsg and from Msg conversion functions need to exist.
 * If they don't exist or do not apply (for example, if your two
 * classes are ROS messages), just write a specialization of the function.
 * \param a an object to convert from
 * \param b the object to convert to
 */
template <class A, class B>
  void convert(const A& a, B& b)
  {
    //printf("In double type convert\n");
    impl::Converter<ros::message_traits::IsMessage<A>::value, ros::message_traits::IsMessage<B>::value>::convert(a, b);
  }

template <class A>
  void convert(const A& a1, A& a2)
  {
    //printf("In single type convert\n");
    if(&a1 != &a2)
      a2 = a1;
  }

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
