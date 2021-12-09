/*
 * PointXYZRGBConfidenceRatio.hpp
 *
 *  Created on: Nov 26, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *   This file defines our custom pcl type, ie including a confidence_ratio.
 *   Adapted from https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/impl/point_types.hpp
 */

#pragma once

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct _PointXYZRGBConfidenceRatio {  // NOLINT(cppcoreguidelines-pro-type-union-access)
  PCL_ADD_POINT4D;  // NOLINT(cppcoreguidelines-pro-type-union-access, readability-const-return-type, modernize-avoid-c-arrays) This adds
                    // the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_RGB;      // NOLINT(cppcoreguidelines-pro-type-union-access, readability-const-return-type)
  union {
    struct {
      float confidence_ratio;  // NOLINT(readability-identifier-naming)
    };
    float data_c[4];  // NOLINT(readability-identifier-naming, modernize-avoid-c-arrays)
  };
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment
#pragma GCC diagnostic pop

struct PointXYZRGBConfidenceRatio : public _PointXYZRGBConfidenceRatio {
  inline explicit PointXYZRGBConfidenceRatio(const _PointXYZRGBConfidenceRatio& p) : _PointXYZRGBConfidenceRatio() {
    // XZY
    x = p.x;         // NOLINT(cppcoreguidelines-pro-type-union-access)
    y = p.y;         // NOLINT(cppcoreguidelines-pro-type-union-access)
    z = p.z;         // NOLINT(cppcoreguidelines-pro-type-union-access)
    data[3] = 1.0f;  // NOLINT(cppcoreguidelines-pro-type-union-access)

    // RGB
    rgba = p.rgba;  // NOLINT(cppcoreguidelines-pro-type-union-access)

    // Confidence
    confidence_ratio = p.confidence_ratio;  // NOLINT(cppcoreguidelines-pro-type-union-access)
  }

  inline explicit PointXYZRGBConfidenceRatio(float _confidence_ratio = 1.f)
      : PointXYZRGBConfidenceRatio(0.f, 0.f, 0.f, 0, 0, 0, _confidence_ratio) {}

  inline PointXYZRGBConfidenceRatio(std::uint8_t _r, std::uint8_t _g, std::uint8_t _b)
      : PointXYZRGBConfidenceRatio(0.f, 0.f, 0.f, _r, _g, _b) {}

  inline PointXYZRGBConfidenceRatio(float _x, float _y, float _z) : PointXYZRGBConfidenceRatio(_x, _y, _z, 0, 0, 0) {}

  inline PointXYZRGBConfidenceRatio(float _x, float _y, float _z, std::uint8_t _r, std::uint8_t _g, std::uint8_t _b,
                                    float _confidence_ratio = 1.f)
      : _PointXYZRGBConfidenceRatio() {
    x = _x;          // NOLINT(cppcoreguidelines-pro-type-union-access)
    y = _y;          // NOLINT(cppcoreguidelines-pro-type-union-access)
    z = _z;          // NOLINT(cppcoreguidelines-pro-type-union-access)
    data[3] = 1.0f;  // NOLINT(cppcoreguidelines-pro-type-union-access)

    r = _r;   // NOLINT(cppcoreguidelines-pro-type-union-access)
    g = _g;   // NOLINT(cppcoreguidelines-pro-type-union-access)
    b = _b;   // NOLINT(cppcoreguidelines-pro-type-union-access)
    a = 255;  // NOLINT(cppcoreguidelines-pro-type-union-access)

    confidence_ratio = _confidence_ratio;  // NOLINT(cppcoreguidelines-pro-type-union-access)
  }

  friend std::ostream& operator<<(std::ostream& os, const PointXYZRGBConfidenceRatio& p);
};

PCL_EXPORTS std::ostream& operator<<(std::ostream& os, const PointXYZRGBConfidenceRatio& p);

}  // namespace pcl

namespace elevation_mapping {
using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>;
}  // namespace elevation_mapping

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_PointXYZRGBConfidenceRatio,  // NOLINT(modernize-avoid-c-arrays, readability-const-return-type) here
                                                                     // we assume a XYZ + "confidence_ratio" (as fields)
                                  (float, x, x)                      // NOLINT
                                  (float, y, y)                      // NOLINT
                                  (float, z, z)                      // NOLINT
                                  (std::uint32_t, rgba, rgba)        // NOLINT
                                  (float, confidence_ratio, confidence_ratio))  // NOLINT

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZRGBConfidenceRatio, pcl::_PointXYZRGBConfidenceRatio)
