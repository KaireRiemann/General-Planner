#pragma once

#include "utils/header/type_utils.hpp"

namespace traj_opt
{

struct PerchingSurfaceState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double t{0.0};
  super_utils::Vec3f position{super_utils::Vec3f::Zero()};
  super_utils::Vec3f velocity{super_utils::Vec3f::Zero()};
  super_utils::Vec3f acceleration{super_utils::Vec3f::Zero()};
  super_utils::Vec3f surface_x{super_utils::Vec3f::UnitX()};
  super_utils::Vec3f surface_y{super_utils::Vec3f::UnitY()};
  super_utils::Vec3f surface_z{super_utils::Vec3f::UnitZ()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

} // namespace traj_opt
