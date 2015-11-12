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

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>

namespace tf2
{

template<>
inline void
tf2::TransformProxy<tf2::Transform>::getTransform(double &qx, double &qy, double &qz, double &qw, double &x, double &y,
                                                  double &z) const
{
  qx = t_.getRotation().x();
  qy = t_.getRotation().y();
  qz = t_.getRotation().z();
  qw = t_.getRotation().w();
  x = t_.getOrigin().x();
  y = t_.getOrigin().y();
  z = t_.getOrigin().z();
}

template<>
inline void
tf2::TransformProxy<tf2::Transform>::setTransform(double qx, double qy, double qz, double qw, double x, double y,
                                                  double z)
{
  t_.setRotation(tf2::Quaternion(qx, qy, qz, qw));
  t_.setOrigin(tf2::Vector3(x, y, z));
}

template class tf2::TransformProxy<tf2::Transform>;
}
