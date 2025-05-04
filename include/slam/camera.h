#pragma

#ifndef SLAM_CAMERA_H
#define SLAM_CAMERA_H

#include "slam/common_include.h"

namespace slam
{
    /**
 * @brief Class representing a pinhole stereo camera model.
 * 
 * This class provides intrinsic and extrinsic parameters of a stereo camera and
 * utility functions to perform coordinate transformations between different
 * coordinate systems (world, camera, and image plane).
 */

 class Camera
 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Ptr = std::shared_ptr<Camera>

    // intinsic parameters

    double fx_ = 0.0;  // focal length in x-direction
    double fy_ = 0.0;  // focal length in y-direction
    double cx_ = 0.0;  // principal point offset in x-direction
    double cy_ = 0.0;  // principal point offset in y-direction
    double baseline_ = 0.0; // stereo baseline in meters


    // extrinsic parameters
    SE3 pose_ ;    // transformation from w2c
    SE3 pose_inv_;  // precomputed inverse of pose

    Camera() = default;


    Camera(double fx, double fy, double cx, double baseline, const SE3 &pose)
           : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
            pose_inv_ = pose_.inverse() // precompute inverse for performance
        }

        // get the camera pose (extrincic)
        SE3 pose() const {return pose_;}

        // return the 3x3 intrinsic matrix

        Mat33 k() const {
            Mat22 k; 
            k << fx_, 0, cx_,
                  0, fy_, cy_,
                  0, 0, 1;
            return k  
        }
        


        // convert a 3D point from world coordinates to camera coordinates

        Vec3 world2camera(const Vec3 &p_world, const SE3 &T_c_w);

        // convert a point from camera coordinates to worls coordinates
        
        Vec3 camera2world(const Vec3 &p_camera, const SE3 &T_c_w);

        // project a 3d point in camera coordinates to 2d pixel coordinates 

        Vec2 camera2pixel(const Vec3 &p_camera);

        // project a 2d pixel coordinate (with depth) to camera coorinates

        Vec3 pixel2camera(const Vec2 &p_pixel, double depth =1.0);

        // convert a pixel coordinate with depth to world coordinates

        Vec2 pixel2world(const Vec3 &p_pixel, const SE3 &T_c_w, double depth =1.0);

        // progect a 3d world point directly to 2d pixel coordinates

        Vec2 world2pixel(const Vec3 &p_world, const SE3 &T_c_w);

};
 


} // namespace slam

#endif