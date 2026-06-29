#pragma once

#include "utils/header/type_utils.hpp"

namespace traj_opt
{

struct PerchingSurfaceState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t{0.0};
  general_utils::Vec3f position{general_utils::Vec3f::Zero()};
  general_utils::Vec3f velocity{general_utils::Vec3f::Zero()};
  general_utils::Vec3f acceleration{general_utils::Vec3f::Zero()};
  general_utils::Vec3f surface_x{general_utils::Vec3f::UnitX()};
  general_utils::Vec3f surface_y{general_utils::Vec3f::UnitY()};
  general_utils::Vec3f surface_z{general_utils::Vec3f::UnitZ()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

} // namespace traj_opt
