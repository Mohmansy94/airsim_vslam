// Feature.h
// Description: Declaration of the Feature class which represents a 2D image keypoint 
// used in SLAM systems and may be associated with a 3D MapPoint after triangulation.

#pragma once

#ifndef SLAM_FEATURE_H
#define SLAM_FEATURE_H

#include <memory>                     // For smart pointers
#include <opencv2/features2d.hpp>    // For cv::KeyPoint
#include "slam/common_include.h"   // Common project-specific includes

namespace slam {

// Forward declarations to avoid circular dependencies
struct Frame;
struct MapPoint;

/**
 * @brief The Feature class encapsulates a keypoint detected in an image.
 * 
 * This class serves as a lightweight wrapper around a 2D feature (keypoint)
 * detected by feature extraction algorithms such as ORB or SIFT.
 * 
 * After triangulation (i.e., estimating 3D point from multiple 2D views), 
 * a Feature can be linked to a 3D MapPoint representing the real-world location.
 */
struct Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure proper alignment for Eigen data types

    /// Type alias for shared pointer to Feature
    using Ptr = std::shared_ptr<Feature>;

    /// The frame in which this feature was detected (non-owning weak reference to avoid cyclic dependency)
    std::weak_ptr<Frame> frame_;         

    /// The position of the feature in 2D image coordinates (e.g., pixel location)
    cv::KeyPoint position_;              

    /// Optional link to the corresponding 3D MapPoint if this feature was triangulated
    std::weak_ptr<MapPoint> map_point_;  

    /// Flag to indicate if this feature has been marked as an outlier (e.g., during bundle adjustment)
    bool is_outlier_ = false;            

    /// Flag to indicate if this feature was detected in the left stereo image.
    /// If false, it was detected in the right image.
    bool is_on_left_image_ = true;       

public:
    /// Default constructor
    Feature() = default;

    /**
     * @brief Construct a new Feature object given a frame and keypoint.
     * 
     * @param frame Shared pointer to the frame in which the keypoint was detected.
     * @param kp The 2D keypoint corresponding to the detected feature.
     */
    Feature(const std::shared_ptr<Frame>& frame, const cv::KeyPoint& kp)
        : frame_(frame), position_(kp) {}
};

}  // namespace slam

#endif  // SLAM_FEATURE_H
