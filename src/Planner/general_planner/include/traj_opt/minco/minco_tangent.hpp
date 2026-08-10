#pragma once

// MINCOTangent is defined with MINCOTrajectory so it can use the trajectory's
// fixed-size boundary and coefficient types.  This forwarding header keeps
// the tangent API discoverable without duplicating the template definition.

#include "traj_opt/minco/minco_trajectory.hpp"

namespace minco
{

template <int DIM, int S>
using MincoTangent = typename MINCOTrajectory<DIM, S>::MINCOTangent;

} // namespace minco
