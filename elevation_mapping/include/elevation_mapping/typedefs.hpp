/*
 * typedefs.hpp
 *
 *  Created on: Sep 26, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#pragma once

#include <kindr/Core>
#include <grid_map_core/TypeDefs.hpp>

namespace elevation_mapping {

using Position2 = grid_map::Position;
using Position3 = kindr::Position3D;
using RotationMatrix = kindr::RotationMatrixPD;
using Transform = kindr::HomTransformQuatD;

}
