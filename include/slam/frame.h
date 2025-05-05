# pragma

#ifndef SLAM_FRAME_H
#define SLAME_FRAME_H

#include"slam/camera.h"

#include"slam/common_include.h"

namespace slam
{
 
struct MapPoint;
struct Feature;


// Each frame stors stereo images, extract features, and a pose. 
// some frames are marked as keyframes
struct Frame {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using Ptr = std::shared_ptr<Frame>;
    

    // unique id for this frame
    unsigned long id_ = 0;


    // Whether is this frame a keyframe or not

    bool is_keyframe_ = false;

    // timestamp of the frame 

    double time_stamp_ =0.0;

    // pose of the camera (transform from w2c
    
    SE3 pose_;

    // mutex to protect access to the pose(for multithreading)
    std::mutex pose_mutex_;

    // stereo images

    cv::Mat left_img_, right_img_;

    // Extract features in the left images 
    std::vector<std::shared_ptr<Feature>> features_left_;


    // Extract features in the right images
    std::vector<std::shared_ptr<Feature>> features_right_;

public:

    // default constractor
    Frame() == default;

    // constractor with initialization 
    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);


    SE3 Pose() {
        std::unique_lock<std::mutex> lock(pose_mutex_);
        return pose_;
    }

    /// Thread-safe setter for the pose
    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lock(pose_mutex_);
        pose_ = pose;
    }

    /// Mark this frame as a keyframe and assign a keyframe ID
    void SetKeyFrame();

    /// Factory method to create a new frame with unique ID
    static std::shared_ptr<Frame> CreateFrame();
 
};


} // namespace slam



#endif