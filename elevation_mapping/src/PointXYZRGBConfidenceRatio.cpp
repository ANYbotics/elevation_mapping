/*
 * PointXYZRGBConfidenceRatio.cpp
 *
 *  Created on: Nov 26, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *   This file contains explicit definitions of algorithms used with our custom pcl type.
 */

#define PCL_NO_PRECOMPILE
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

template class pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::PCLBase<pcl::PointXYZRGBConfidenceRatio>;    // NOLINT(cppcoreguidelines-special-member-functions)
template class pcl::VoxelGrid<pcl::PointXYZRGBConfidenceRatio>;  // NOLINT(cppcoreguidelines-special-member-functions)
template void pcl::removeNaNFromPointCloud<pcl::PointXYZRGBConfidenceRatio>(
    const pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>& cloud_in, pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>& cloud_out,
    std::vector<int, std::allocator<int> >& index);
template class pcl::ExtractIndices<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio>;

std::ostream& operator<<(std::ostream& os, const pcl::PointXYZRGBConfidenceRatio& p) {
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << static_cast<int>(p.r) << ","  // NOLINT(cppcoreguidelines-pro-type-union-access)
     << static_cast<int>(p.g)                                                            // NOLINT(cppcoreguidelines-pro-type-union-access)
     << ","                                                                              // NOLINT(cppcoreguidelines-pro-type-union-access)
     << static_cast<int>(p.b) << " - " << p.confidence_ratio << ")";                     // NOLINT(cppcoreguidelines-pro-type-union-access)
  return (os);
}
